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
  ?*COMMON-TIMEOUT-DURATION* = 30
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
      (state ?status&~FORMULATED&~RUNNING&~FAILED&~FINAL)
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
