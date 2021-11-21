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
  ; The waiting timeout duration needs to be smaller than the common one above!
  ?*WAITING-TIMEOUT-DURATION* = 25
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
	(test (neq ?goal-id BEACONACHIEVE))
	(not (action-timer (plan-id ?plan-id) (action-id ?id) (status ?status)))
	(wm-fact (key refbox game-time) (values $?now))
	=>
	(bind ?timeout-duration ?*COMMON-TIMEOUT-DURATION*)
	(if (eq ?status WAITING)
	 then
		(bind ?timeout-duration ?*WAITING-TIMEOUT-DURATION*)
	)
	(assert (action-timer (plan-id ?plan-id)
	            (action-id ?id)
	            (timeout-duration ?timeout-duration)
	            (status ?status)
	            (start-time ?now)))
)

(defrule execution-monitoring-detect-timeout
" If an action was longer than its timeout-duration in a volatile state like pending or pending-sensed-effect
  reason that this action got stuck. Print a notification, set state to failed and retract the timer.
"
  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	         (id ?id) (state ?status)
	         (action-name ?action-name)
	         (param-values $?param-values)
           (precondition ?grounding-id))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox game-time) (values $?now))
  ?pt <- (action-timer (plan-id ?plan-id) (status ?status)
            (action-id ?id)
            (start-time $?st)
            (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
  =>
  (printout t "Action "  ?action-name " timed out after " ?status  crlf)
  (modify ?p (state FAILED) (error-msg "Unsatisfied precondition"))
  (retract ?pt)
)

(defrule execution-monitoring-detect-timeout-cause
" If an action FAILED and has an unsatisfied precondition error, match all unsatisifed predicates
  and print the cause. This rule might execute multiple times per timed out formula.
"
  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	         (id ?id) (state FAILED)
	         (action-name ?action-name)
	         (param-values $?param-values)
           (precondition ?grounding-id) (error-msg "Unsatisfied precondition"))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (pddl-formula (id ?formula-id) (type atom))
  (grounded-pddl-formula (formula-id ?formula-id) (grounding ?grounding-id) (is-satisfied ?sat))
  (pddl-predicate (part-of ?formula-id) (predicate ?predicate))
  (or
    (test (and ?sat (domain-is-formula-negative ?formula-id)))
    (test (and (not ?sat) (not (domain-is-formula-negative ?formula-id))))
  )
  =>
  (if ?sat
    then
      (printout error  "Action "  ?action-name " precondition (not " ?predicate ") -- which might be nested -- is not satisfied." crlf)
    else
      (printout error "Action "  ?action-name " precondition " ?predicate " -- which might be nested -- is not satisfied." crlf)
  )
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
