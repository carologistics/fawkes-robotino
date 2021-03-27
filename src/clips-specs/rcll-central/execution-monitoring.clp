;---------------------------------------------------------------------------
;  exploration.clp - Robotino agent for exploration phase
;
;  Copyright  2018 Mostafa Gooma, Tarik Viehmann, Daniel Habering
;
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


;A timeout for an action
(deftemplate action-timer
  (slot plan-id (type SYMBOL))
  (slot action-id(type NUMBER))
  (slot timeout-duration)
  (multislot start-time)
  (slot status)
)

(defglobal
  ?*MONITORING-SALIENCE* = 1
  ?*COMMON-TIMEOUT-DURATION* = 15
  ?*MPS-DOWN-TIMEOUT-DURATION* = 120
  ?*HOLDING-MONITORING* = 60
)

;
; =============================== Timeouts ===============================
;
(defrule execution-monitoring-create-action-timeout
" For every state of an action that should not take long (pending, waiting for sensed effects),
  create a timeout to prohibit getting stuck at these volatile states
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
      (id ?id)
      (state ?status&~FORMULATED&~FAILED&~FINAL)
      (action-name ?action-name)
      (param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ; TODO: Only Production goals
  (test (neq ?goal-id BEACONACHIEVE))
  (not (action-timer (plan-id ?plan-id) (action-id ?id) (status ?status)))
  (wm-fact (key refbox game-time) (values $?now))
  =>
  (assert (action-timer (plan-id ?plan-id)
              (action-id ?id)
              (timeout-duration ?*COMMON-TIMEOUT-DURATION*)
              (status ?status)
              (start-time ?now)))
)


(defrule execution-monitoring-detect-timeout
" If an action was longer than its timeout-duration in a volatile state like pending or pending-sensed-effect
  reason that this action got stuck and set it to failed
"
  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	         (id ?id) (state ?status)
	         (action-name ?action-name)
	         (param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox game-time) (values $?now))
  ?pt <- (action-timer (plan-id ?plan-id) (status ?status)
            (action-id ?id)
            (start-time $?st)
            (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
  =>
  (printout t "Action "  ?action-name " timedout after " ?status  crlf)
  (do-for-all-facts ((?da domain-atomic-precondition)) (and (eq ?da:grounded-with ?id)
                                                            (eq ?da:plan-id ?plan-id)
                                                            (eq ?da:goal-id ?goal-id)
                                                            ?da:grounded)
    (if (and ?da:is-satisfied (any-factp ((?dp domain-precondition))
                                                (and (eq ?dp:name ?da:part-of)
                                                     (eq ?dp:goal-id ?goal-id)
                                                     (eq ?dp:plan-id ?plan-id)
                                                     (eq ?dp:grounded-with ?id)
                                                     (eq ?dp:type negation))))
        then
          (printout error "Precondition (not " ?da:predicate ?da:param-values ") is not satisfied." crlf)
    )
    (if (and (not ?da:is-satisfied) (not (any-factp ((?dp domain-precondition))
                                                (and (eq ?dp:name ?da:part-of)
                                                     (eq ?dp:goal-id ?goal-id)
                                                     (eq ?dp:plan-id ?plan-id)
                                                     (eq ?dp:grounded-with ?id)
                                                     (eq ?dp:type negation)))))
        then
          (printout error "Precondition " ?da:predicate ?da:param-values " is not satisfied" crlf)
    )
  )
  (modify ?p (state FAILED) (error-msg "Unsatisfied precondition"))
  (retract ?pt)
)


(defrule execution-monitoring-remove-timer
" If an action is in a different state than when creating a timer,
  then we can safely remove the timer, since it got not stuck in the previous state
"
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state ?status)
	   (action-name ?action-name)
	   (param-values $?param-values))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pt <- (action-timer (plan-id ?plan-id) (action-id ?id) (status ?st& : (neq ?st ?status)))
  =>
  (retract ?pt)
)

;copied from RCLL agent
;======================================Retries=========================================
;

(deffunction should-retry (?an ?error)
  ;(if (or (eq ?error "Conveyor Align Failed") (eq ?error "Drive To Machine Point Failed")) then
  ;  (return TRUE)
  ;)
  ;(if (eq ?error "Unsatisfied precondition") then (return TRUE))
  ;(if (and (or (eq ?an wp-put) (eq ?an wp-put-slide-cc))
  ;         (any-factp ((?if RobotinoSensorInterface))
  ;                    (and (not (nth$ 1 ?if:digital_in)) (nth$ 2 ?if:digital_in)))) then
  ;  (return TRUE)
  ;)
  ;(if (or (eq ?an move) (eq ?an go-wait)) then
  ;  (return TRUE)
  ;)
  (return FALSE)
)

(defrule execution-monitoring-start-retry-action
" For some actions it can be feasible to retry them in case of a failure, e.g. if
  the align failed. If an action failed and the error-msg inquires, that we should
  retry, add a action-retried counter and set the action to FORMULATED
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (goal (id ?goal-id) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?pa <- (plan-action
              (id ?id)
              (action-name ?an)
              (plan-id ?plan-id)
              (goal-id ?goal-id)
              (state FAILED)
              (error-msg ?error)
              (param-values $?param-values))
  (test (eq TRUE (should-retry ?an ?error)))
  (wm-fact (key domain fact self args? r ?r))
  (not (wm-fact (key monitoring action-retried args? r ?r a ?an id ?id2&:(eq ?id2 (sym-cat ?id)) m ? g ?goal-id)))
  =>
  (bind ?mps nil)
  (do-for-fact ((?do domain-object)) (and (member$ ?do:name ?param-values) (eq ?do:type mps))
    (bind ?mps ?do:name)
  )
  (assert
    (wm-fact (key monitoring action-retried args? r ?r a ?an id (sym-cat ?id) m ?mps g ?goal-id) (value 0))
  )
  (printout error "Start retrying " ?an " " ?plan-id crlf)
)


(defrule execution-monitoring-retry-action
" If the retry of an action is failed again, increment the counter and restart the action
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pa <- (plan-action
              (id ?id)
              (action-name ?an)
              (plan-id ?plan-id)
              (goal-id ?goal-id)
              (state FAILED)
              (error-msg ?error)
              (param-values $?param-values))
  (wm-fact (key domain fact self args? r ?r))
  (test (eq TRUE (should-retry ?an ?error)))
  ?wm <- (wm-fact (key monitoring action-retried args? r ?r a ?an id ?id2&:(eq ?id2 (sym-cat ?id)) m ? g ?goal-id)
          (value ?tries&:(< ?tries ?*MAX-RETRIES-PICK*)))
  =>
  (bind ?tries (+ 1 ?tries))
  (modify ?pa (state FORMULATED))
  (printout error "Restarted: " ?tries crlf)
  (modify ?wm (value ?tries))
)

(defrule execution-monitoring-retry-action-failed
  (declare (salience 1000))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED) (class ?class) (params $?params))
  (plan-action
        (id ?id)
        (goal-id ?goal-id)
        (plan-id ?plan-id)
        (action-name ?an)
        (state FAILED)
        (error-msg ?error))
  ?wm <- (wm-fact (key monitoring action-retried args? r ?r a ?an id ?id2&:(eq ?id2 (sym-cat ?id)) m ? g ?goal-id)
                (value ?tries&:(= ?tries ?*MAX-RETRIES-PICK*)))
  =>
  (assert
    (wm-fact (key monitoring shame args?))
  )
  (printout error "Reached max retries" crlf)
)

(defrule execution-monitoring-clear-action-retried
  (declare (salience ?*MONITORING-SALIENCE*))
  ?wm <- (wm-fact (key monitoring action-retried args? r ?r a ? id ? m ? g ?goal-id))
  (goal (id ?goal-id) (mode EVALUATED))
  =>
  (retract ?wm)
)

