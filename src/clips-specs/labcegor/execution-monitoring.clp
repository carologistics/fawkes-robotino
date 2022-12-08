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

(deffunction fail-action (?action ?error-msg)
	(do-for-fact ((?sae skill-action-execinfo))
		(and (eq ?sae:goal-id (fact-slot-value ?action goal-id))
		     (eq ?sae:plan-id (fact-slot-value ?action plan-id))
		     (eq ?sae:action-id (fact-slot-value ?action id))
		)
		(do-for-fact ((?skill skill))
			(and (eq ?skill:id ?sae:skill-id)
			     (eq ?skill:skiller ?sae:skiller)
			)
			(retract ?skill)
		)
		(retract ?sae)
	)
	(return (modify ?action (state EXECUTION-FAILED) (error-msg ?error-msg)))
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
	(test (neq ?goal-id BEACONACHIEVE))
	(not (action-timer (plan-id ?plan-id) (action-id ?id) (status ?status)))
	(wm-fact (key refbox game-time) (values $?now))
	=>
	(bind ?timeout-duration ?*COMMON-TIMEOUT-DURATION*)
	(if (eq ?status WAITING)
	 then
		(bind ?timeout-duration ?*WAITING-TIMEOUT-DURATION*)
	)
	(if (eq ?status RUNNING)
	 then
		(bind ?timeout-duration ?*RUNNING-TIMEOUT-DURATION*)
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
           (skiller ?skiller)
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
  (if (eq ?status RUNNING)
   then
    (printout t "   Aborting action " ?action-name " on interface" ?skiller crlf)
    (bind ?m (blackboard-create-msg (str-cat "SkillerInterface::" ?skiller) "StopExecMessage"))
    (blackboard-send-msg ?m)
    (modify ?p (state FAILED) (error-msg "Stuck on RUNNING"))
   else
    (modify ?p (state FAILED) (error-msg "Unsatisfied precondition"))
  )
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

;(defrule execution-monitoring-wp-put-slide-cc
;" Adapting params of wp-put-slide-cc if the number of rings of rs-before differs
;  from the number of rings the rs is filled with "
;  (declare (salience ?*MONITORING-SALIENCE*))
;  ?pa <- (plan-action (state PENDING)
;               (action-name wp-put-slide-cc)
;               (id ?action-id)
;               (param-values ?robot ?wp ?rs ?rs-before ?rs-after)
;                      (precondition ?grounding-id)
;  )
;  (grounded-pddl-predicate (grounding ?grounding-id) (is-satisfied FALSE))
;  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-num&:(not (eq ?rs-num ?rs-before))))
;  (wm-fact (key domain fact rs-inc args? summand ?rs-num sum ?rs-num-after))
;  =>
;  (printout t "execuction monitoring wp-put-slide-cc: rs-before is unequal to rs-filled-with" crlf)
;  (modify ?pa (param-values ?robot ?wp ?rs ?rs-num ?rs-num-after))
;)

(defrule execution-monitoring-reset-abort-timer-machine-down
  "Restart timer for prepare actions if the machine is down"
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state RUNNING)
                      (action-name prepare-bs|
                                   prepare-cs|
                                   prepare-ds|
                                   prepare-rs|
                                   prepare-ss)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  ?at <- (timer (name ?nat&:(eq ?nat
                                (sym-cat prepare- ?goal-id - ?plan-id
                                         - ?id -abort-timer)))
	        (time $?t&:(timeout ?now ?t (- ?*ABORT-PREPARE-PERIOD* ?*ABORT-PREPARE-DOWN-RESET*)))
                (seq ?seq))
  (wm-fact (key domain fact mps-state args? m ?mps s DOWN))
  =>
  (modify ?at (time ?now))
  (printout t "Action prepare-" ?mps " timer extended due to down machine" crlf)
)

(defrule execution-monitoring-reset-action-timer-machine-down
  "Restart timer for prepare actions if the machine is down"
  (time $?now)
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state PENDING|RUNNING)
                      (action-name prepare-bs|
                                   prepare-cs|
                                   prepare-ds|
                                   prepare-rs|
                                   prepare-ss|
                                   cs-mount-cap|
                                   cs-buffer-cap|
                                   rs-mount-ring1|
                                   rs-mount-ring2|
                                   rs-mount-ring3)
                      (param-names $?param-names)
                      (param-values $?param-values))
  ?at <- (action-timer (plan-id ?plan-id) (status ?status)
            (action-id ?id)
            (start-time $?st)
            (timeout-duration ?timeout&:(timeout ?now ?st (- ?timeout ?*ABORT-PREPARE-DOWN-RESET*))))
  (domain-obj-is-of-type ?mps&:(eq ?mps (plan-action-arg m
                                                         ?param-names
                                                         ?param-values))
                         mps)
  (wm-fact (key domain fact mps-state args? m ?mps s DOWN))
  =>
  (modify ?at (timeout-duration (* ?timeout 2)))
  (printout t "Action prepare-" ?mps " timer extended due to down machine" crlf)
)

;(defrule execution-monitoring-fix-rs-mount-counter
;  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
;                      (state PENDING) (executable FALSE)
;                      (action-name rs-mount-ring1|
;                                   rs-mount-ring2|
;                                   rs-mount-ring3)
;                      (param-values ?rs $?o-args ?rs-before ?rs-after ?rs-req))
;  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-new&:(neq ?rs-new ?rs-before)))
;  (wm-fact (key domain fact rs-sub args? minuend ?rs-new
;                                         subtrahend ?rs-req
;                                         difference ?bases-remain))
;  =>
;  (modify ?pa (param-values ?rs $?o-args ?rs-new ?bases-remain ?rs-req))
;)
