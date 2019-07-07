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


(deffunction should-retry (?an ?error)
  (if (or (eq ?error "Conveyor Align Failed") (eq ?error "Drive To Machine Point Failed")) then
    (return TRUE)
  )
  (if (eq ?error "Unsatisfied precondition") then (return FALSE))
  (if (and (or (eq ?an wp-put) (eq ?an wp-put-slide-cc))
           (any-factp ((?if RobotinoSensorInterface))
                      (and (not (nth$ 1 ?if:digital_in)) (nth$ 2 ?if:digital_in)))) then
    (return TRUE)
  )
  (if (or (eq ?an move) (eq ?an go-wait)) then
    (return TRUE)
  )
  (return FALSE)
)

;
; ============================== MPS State Monitoring ==============================
;

(defrule execution-monitoring-spawn-missing-wp-on-ready-at-output
" If an mps is READY-AT-OUTPUT and an action has a pending effect for it,
  but there is no workpiece object at the output of this machine, generate a new workpiece
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (domain-pending-sensed-fact
    (goal-id ?goal-id)
    (action-id ?action-id)
    (name mps-state)
    (param-values ?mps READY-AT-OUTPUT)
    (type POSITIVE))
  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
  (not (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT)))
  =>
  (printout warn "Monitoring: MPS state READY-AT-OUTPUT but no WP at output, Yet action " ?action-id " in Goal " ?goal-id
    "expected it!" crlf)
  (bind ?wp-gen  (sym-cat WP- (gensym)))
  (assert (domain-object (name (sym-cat WP- (gensym))) (type workpiece))
          (wm-fact (key domain fact wp-at args? wp ?wp-gen m ?mps side OUTPUT))
          (wm-fact (key domain fact wp-usable args? wp ?wp-gen))
  )
  (do-for-fact ((?wm wm-fact)) (and (wm-key-prefix ?wm:key (create$ domain fact mps-side-free))
                                    (eq ?mps (wm-key-arg ?wm:key m))
                                    (eq OUTPUT (wm-key-arg ?wm:key s)))
    (retract ?wm)
  )
  (printout warn "A WP has been Generated at the OUTPUT side" crlf)
)

(defrule execution-monitoring-skipped-ready-at-output
" Execution Monitoring MPS state
  When a bot waits for an mps to be ready-at-output, and its idle, reason that it must have been ready at output in between
  This is only reasonable, when the fact, that the bots waits for ready-at-output includes, that the mps actually was prepared correctly. Otherwise this will break things
"
  (declare (salience ?*MONITORING-SALIENCE*))
  ?dp <- (domain-pending-sensed-fact (action-id ?action-id)
            (plan-id ?plan-id)
            (goal-id ?goal-id)
            (name mps-state)
            (param-values ?mps READY-AT-OUTPUT))
  ?pa <- (plan-action (id ?action-id)
            (plan-id ?plan-id)
            (goal-id ?goal-id)
            (state SENSED-EFFECTS-WAIT))
  (wm-fact (key domain fact mps-state args m ?mps s IDLE))
  =>
  (printout warn "Monitoring: Waited for READY-AT-OUTPUT at " ?mps " and detected IDLE. Remove pending-sensed fact" crlf)
  (retract ?dp)
)

(defrule execution-monitoring-broken-mps-fail-pending-action
" If an action is pending and depends on a broken mps, we can instantly set it to failed,
  since the precondition will never be satisfied.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  (goal (id ?goal-id) (mode DISPATCHED))
  ?pa <- (plan-action (id ?action-id)
                      (action-name ?an)
                      (goal-id ?goal-id)
                      (param-values $? ?mps $?)
                      (state PENDING))
  (domain-atomic-precondition (operator ?an) (grounded-with ?action-id) (is-satisfied FALSE))
  =>
  (modify ?pa (state EXECUTION-FAILED))
)

(defrule execution-monitoring-broken-mps-add-fail-goal-flag
" If an action makes use of a mps that is broken (and has the mps-state as precondition),
  mark the corresponding goal to be failed.
  Reason: A broken mps looses all stored parts, therefore making the current goal unable to achieve
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?g <- (goal (id ?goal-id) (mode DISPATCHED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id ?id) (plan-id ?plan-id) (goal-id ?goal-id)
     (state FORMULATED|PENDING)
     (param-values $? ?mps $?)
     (action-name ?an))
  (domain-atomic-precondition (operator ?an) (predicate mps-state) (param-values ?mps ?state))
  (not (wm-fact (key monitoring fail-goal args? g ?goal-id)))
  =>
  (assert (wm-fact (key monitoring fail-goal args? g ?goal-id) (type UNKNOWN)))
)


(defrule execution-monitoring-broken-mps-fail-goal
" If the current dispatched goal is marked to be failed and no action is running,
  set the goal to finished and failed
"
  (declare (salience ?*MONITORING-SALIENCE*))
  ?fg <-(wm-fact (key monitoring fail-goal args? g ?goal-id) (type UNKNOWN))
  ?g <- (goal (id ?goal-id) (mode DISPATCHED))
  (not (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (state ~FORMULATED&~PENDING&~FINAL&~FAILED)))
  =>
  (printout t "Fail goal " ?goal-id " because it is unsatisfiable" crlf)
  (retract ?fg)
  (modify ?g (mode FINISHED) (outcome FAILED))
)


(defrule execution-monitoring-broken-mps-remove-facts
" If a mps is broken, all stored parts and stati are lost.
  Therefore, remove all corresponding facts and mark every workpiece at the machine for cleanup, since its lost
  Afterwards reset the facts of the mps to the initial state, depending on the type of the mps
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  (wm-fact (key domain fact mps-type args? m ?mps t ?type))
  =>
  (printout t "MPS " ?mps " was broken, cleaning up facts" crlf)
  (do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?mps (wm-key-args ?wf:key)) FALSE)
                                         (or
                                            (wm-key-prefix ?wf:key (create$ domain fact bs-prepared-color))
                                            (wm-key-prefix ?wf:key (create$ domain fact ds-prepared-gate))
                                            (wm-key-prefix ?wf:key (create$ domain fact bs-prepared-side))
                                            (wm-key-prefix ?wf:key (create$ domain fact cs-prepared-for))
                                            (wm-key-prefix ?wf:key (create$ domain fact cs-buffered))
                                            (wm-key-prefix ?wf:key (create$ domain fact cs-can-perform))
                                            (wm-key-prefix ?wf:key (create$ domain fact rs-filled-with))
                                            (wm-key-prefix ?wf:key (create$ domain fact rs-prepared-color))
                                            (and (wm-key-prefix ?wf:key (create$ mps-handling prepare))
                                                 (member$ ?mps (wm-key-path ?wf:key)))
                                            (and (wm-key-prefix ?wf:key (create$ mps-handling process))
                                                 (member$ ?mps (wm-key-path ?wf:key)))
                                         )
                                     )
    (retract ?wf)
    (printout t "Exec-Monotoring: Broken Machine " ?wf:key crlf " domain facts flushed!"  crlf)
  )

  (switch ?type
    (case CS then
      (assert (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP) (type BOOL) (value TRUE)))
    )
    (case RS then
      (assert (wm-fact (key domain fact rs-filled-with args? m ?mps n ZERO) (type BOOL) (value TRUE)))
    )
  )
  (do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?mps (wm-key-args ?wf:key)) FALSE)
                                              (wm-key-prefix ?wf:key (create$ domain fact wp-at)))
           (assert (wm-fact (key monitoring cleanup-wp args? wp (wm-key-arg ?wf:key wp))))
  )
  (if (not (any-factp ((?wf wm-fact)) (and (wm-key-prefix ?wf:key (create$ domain fact mps-side-free))
                                           (eq (wm-key-arg ?wf:key m) ?mps)
                                           (eq (wm-key-arg ?wf:key side) INPUT))))
      then
    (assert (wm-fact (key domain fact mps-side-free args? m ?mps side INPUT)))
  )
  (if (not (any-factp ((?wf wm-fact)) (and (wm-key-prefix ?wf:key (create$ domain fact mps-side-free))
                                           (eq (wm-key-arg ?wf:key m) ?mps)
                                           (eq (wm-key-arg ?wf:key side) OUTPUT))))
      then
    (assert (wm-fact (key domain fact mps-side-free args? m ?mps side OUTPUT)))
  )
)


(defrule execution-monitoring-broken-mps-reject-goals
" Every goal that was formulated, that depends on a currently broken goal,
  was formulated with outdated facts about the mps. Therfore, reject it.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?g <- (goal (id ?goal-id) (mode FORMULATED|SELECTED|EXPANDED) (params $? ?mps $?))
  (plan (id ?plan-id) (goal-id ?goal-id))
  =>
  (modify ?g (mode FINISHED) (outcome REJECTED))
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

(defrule execution-monitoring-enhance-timer-on-mps-wait-for-side-to-clear
" If an action is pending for a certain mps-side to be free and the mps is currently in a non final state (processing, down, ...),
  enhance the timeout to give the mps enough time to process the workpiece that is still at the side
"
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state PENDING)
	   (action-name wp-put)
	   (param-values $? ?mps $? ?side $?))
  (domain-atomic-precondition (operator ?an) (grounded-with ?id) (predicate mps-side-free) (param-values ?mps ?side))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key domain fact mps-state args? m ?mps s ?s&~IDLE&~READY-AT-OUTPUT))
  ?pt <- (action-timer (plan-id ?plan-id)
            (action-id ?id)
            (start-time $?starttime)
            (timeout-duration ?timeout&:(neq ?timeout ?*MPS-DOWN-TIMEOUT-DURATION*)))
  =>
  (printout t "Detected that " ?mps " is " ?s " while wp-put is waiting for the side to clear. Enhance timeout-timer" crlf)
  (modify ?pt (timeout-duration ?*MPS-DOWN-TIMEOUT-DURATION*))
)

;
;======================================Retries=========================================
;

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
  (printout error "Start retrying" crlf)
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

(defrule execution-monitoring-remove-forbid
  (declare (salience ?*MONITORING-SALIENCE*))
  ?wm <- (wm-fact (key monitoring forbid-goal args? c ?class mps ?mps))
  (goal (id ?goal-id) (sub-type SIMPLE) (class ?c&:(production-leaf-goal ?c)) (params $?p) (outcome COMPLETED) (mode FINISHED))
  (test (not (member$ ?mps ?p)))
  =>
  (retract ?wm)
)

(defrule execution-monitoring-reject-forbidden-goal
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (wm-fact (key monitoring forbid-goal args? c ?class mps ?mps))
  ?g <- (goal (id ?goal-id) (params $? ?mps $?) (mode ?m&:(and (not (eq ?m DISPATCHED)) (not (eq ?m FINISHED)) (not (eq ?m RETRACTED)) (not (eq ?m EVALUATED)) )))
  =>
  (printout t "Goal " ?class " was tried and failed before at " ?mps ". Reject" crlf)
  (modify ?g (outcome REJECTED) (mode RETRACTED))
)

;
;======================================Misc==============================
;

(defrule execution-monitoring-wp-get-finally-failed
" If a wp-get failed, we have to reset the mps to make sure, that there is no workpiece
  pushed into the mps
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (plan-action
        (plan-id ?plan-id)
        (action-name ?an&wp-get)
        (state FAILED)
        (param-values $? ?wp $? ?mps $?)
        (error-msg ?error))
  (test (eq FALSE (should-retry ?an ?error)))
  (domain-obj-is-of-type ?mps mps)
  (domain-obj-is-of-type ?wp workpiece)
  (domain-fact (name mps-state) (param-values ?mps WAIT-IDLE|IDLE))
  (domain-fact (name wp-at) (param-values ?wp ?mps ?side))
  =>
  (printout error "wp-get failed not by aligning: lost " ?wp crlf)
  (assert
    (wm-fact (key monitoring cleanup-wp args? wp ?wp))
    (wm-fact (key monitoring safety-discard))
  )
)

(defrule execution-monitoring-cleanup-wp-facts
"If a workpiece is lost, it is marked for cleanup.
 In this rule all corresponding fact of a marked workpiece are removed
"
  ?cleanup <- (wm-fact (key monitoring cleanup-wp args? wp ?wp))
  =>
  (do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?wp (wm-key-args ?wf:key)) FALSE)
                                         (or
                                           (wm-key-prefix ?wf:key (create$ domain fact wp-at))
                                           (wm-key-prefix ?wf:key (create$ domain fact wp-usable))
	                                         (wm-key-prefix ?wf:key (create$ domain fact wp-on-shelf))
                                           (wm-key-prefix ?wf:key (create$ monitoring cleanup-wp))
                                         )
                                    )
    (retract ?wf)
    (printout t "WP-fact " ?wf:key crlf " domain fact flushed!"  crlf)
  )
  (assert (wm-fact (key wp-unused args? wp ?wp)))
  (retract ?cleanup)
)


(defrule execution-monitoring-bs-switch-sides
" If an agent tries to lock a side of a base station before a dispense and this side is already locked,
  switch the side, since the base station can be operated from both sides
"
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id)
	          (id ?id)
	          (action-name location-lock)
	          (state RUNNING)
	          (param-values ?bs ?side))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (action-name bs-dispense))
  ?li <- (lock-info (name ?name) (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id) (status WAITING))
  (test (eq ?name (sym-cat ?bs - ?side)))
  ; Do not switch sides if the other side is blocked, too.
  (not (mutex (name ?lock&:(eq ?lock (sym-cat ?bs - (if (eq ?side INPUT) then OUTPUT else INPUT)))) (state LOCKED)))
  =>
  (retract ?li)
  (modify ?pa (state PENDING))
  (delayed-do-for-all-facts
    ((?p plan-action))
    (and (eq ?p:goal-id ?goal-id) (eq ?p:plan-id ?plan-id) (neq FALSE (member$ ?bs ?p:param-values)) (neq FALSE (member$ ?side ?p:param-values)))
    (printout t "Execution monitoring: Adapting " ?p:action-name crlf)
    (modify ?p (param-values (replace-member$ ?p:param-values (if (eq ?side INPUT) then OUTPUT else INPUT) ?side)))
  )
)
