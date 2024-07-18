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
;A timeout for a move action
(deftemplate progress-timer
  (slot plan-id (type SYMBOL))
  (slot action-id (type NUMBER))
  (slot timeout-duration)
  (multislot start-time)
  (multislot last-pose)
  (slot counter (type NUMBER))
)
;A timeout for assigned goals in mode FORMULATED
(deftemplate selection-timer
  (slot goal-id (type SYMBOL))
  (slot robot (type SYMBOL))
  (slot timeout-duration)
  (multislot start-time)
)
;A timeout for goals who were retried too many times
(deftemplate goal-retry-wait-timer
  (slot goal-id (type SYMBOL))
  (slot timeout-duration)
  (multislot start-time)
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
	(goal (id ?goal-id) (mode DISPATCHED) (class ?goal-class))
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
	(if (member$ ?action-name (create$ cs-mount-cap cs-retrieve-cap rs-mount-ring1 rs-mount-ring2 rs-mount-ring3))
	 then
		(bind ?timeout-duration ?*PREPARE-WAIT-TIMEOUT-DURATION*)
	)
	(if (eq ?goal-class MOVE-OUT-OF-WAY)
	 then
		(bind ?timeout-duration ?*MOVE-OUT-OF-WAY-TIMEOUT-DURATION*)
	)
	(assert (action-timer (plan-id ?plan-id)
	            (action-id ?id)
	            (timeout-duration ?timeout-duration)
	            (status ?status)
	            (start-time ?now)))
)

(defrule execution-monitoring-retry-stuck-on-waiting
" If an action remains in state WAITING for too long, the skill call over the
  remote blackboard probably got lost. Hence, retry by setting the action
  back to pending.
  If the timeout is too small, it may cause skill calls to arrive twice,
  which aborts the call arriving first. When network delays are frequent, this
  may cause loops of aborted skills.
"
	?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	         (id ?id) (state WAITING)
	         (action-name ?action-name)
	         (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(wm-fact (key game state) (value RUNNING))
	(wm-fact (key refbox game-time) (values $?now))
	?pt <- (action-timer (plan-id ?plan-id) (status WAITING)
	                     (action-id ?id)
	                     (start-time $?st)
	                     (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
	?exec-info <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id))
	=>
	(printout t "Action "  ?action-name " stuck on WAITING, retry"  crlf)
	(modify ?p (state PENDING))
	(retract ?exec-info)
	(retract ?pt)
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
    (bind ?p (modify ?p (state FAILED) (error-msg "Stuck on RUNNING")))
   else
    (bind ?p (modify ?p (state FAILED) (error-msg "Unsatisfied precondition")))
  )
  (fail-action ?p "Unsatisfied precondition")
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

(defrule execution-monitoring-reset-timer-on-move-when-unapproachable
" Resetting the timeout-timer if a move action is pending while waiting for its target mps-side to be approachable.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state PENDING)
	   (action-name move)
	   (param-values ?r ?from ?from-side ?to ?to-side $?)
	   )
  (wm-fact (key domain fact at args? r ?r m ?from side ?from-side))
  (not (wm-fact (key domain fact mps-side-approachable args? m ?to side ?to-side)))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  (wm-fact (key refbox game-time) (values $?now))
  ?pt <- (action-timer (plan-id ?plan-id)
            (action-id ?id)
            (start-time $?st)
            (timeout-duration ?timeout&:(timeout ?now ?st ?timeout))
         )
  =>
  (printout t "Detected that " ?goal-id " is waiting with move until "
               ?to " " ?to-side " is approachable. Reset timeout-timer" crlf)
  (modify ?pt (start-time ?now))
)

(defrule execution-monitoring-set-timeout-goal-selection
" Set a timeout when a goal is assigned to a robot and formulated to avoid
  being stuck on formulated.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (goal (id ?goal-id) (mode FORMULATED) (sub-type SIMPLE))
  (wm-fact (key central agent robot args? r ?robot))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))

  (not (selection-timer (goal-id ?goal-id) (robot ?robot)))
  (wm-fact (key refbox game-time) (values $?now))
  =>
  (assert
    (selection-timer
      (goal-id ?goal-id)
	    (robot ?robot)
	    (timeout-duration ?*GOAL-SELECTION-TIMEOUT*)
	    (start-time ?now)
    )
  )
)

(defrule execution-monitoring-trigger-timeout-goal-selection
" The timeout triggered, remove the assignment of the robot and set it to
  waiting again.
"
  ?g <- (goal (id ?goal-id) (mode FORMULATED))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))

  (wm-fact (key refbox game-time) (values $?now))
  (selection-timer (goal-id ?goal-id) (robot ?robot)
    (start-time $?st)
    (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
  =>
  (printout error  "Goal "  ?goal-id " timed out on selection!" crlf)
  (set-robot-to-waiting ?robot)
  ; remove restrictions
  (delayed-do-for-all-facts ((?ogm goal-meta) (?og goal))
      (and
        (eq ?og:id ?ogm:goal-id)
        (eq ?og:mode FORMULATED)
        (eq ?ogm:restricted-to ?robot)
        (or
          (eq ?og:class MOUNT-RING)
          (eq ?og:class MOUNT-CAP)
          (eq ?og:class DELIVER)
        )
      )
      (modify ?ogm (restricted-to nil))
  )
  (delayed-do-for-all-facts ((?ogm goal-meta))
        (eq ?ogm:assigned-to ?robot)
      (modify ?ogm (assigned-to nil))
  )
  (delayed-do-for-all-facts ((?timer selection-timer)) (eq ?timer:robot ?robot)
    (retract ?timer)
  )
)

(defrule execution-monitoring-remove-timeout-goal-selection
" The robot is executing a goal, so we can remove the timeout.
"
  ?g <- (goal (id ?goal-id) (mode ?mode))
  (goal-meta (goal-id ?goal-id) (assigned-to ?assigned-robot))

  (wm-fact (key refbox game-time) (values $?now))
  ?timer <- (selection-timer (goal-id ?goal-id) (robot ?robot)
    (start-time $?st)
		(timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
  (test (or (neq ?mode FORMULATED) (neq ?assigned-robot ?robot)))
  =>
  (retract ?timer)
)


;======================================Movement=========================================

(defrule execution-monitoring-suspend-plan-to-insert-wait
	(declare (salience ?*MONITORING-SALIENCE*))
	(goal (id ?goal-id) (class ?class))
	?plan <- (plan (id ?plan-id) (goal-id ?goal-id) (suspended FALSE))
	?plan-action <- (plan-action (id ?action-id)
	             (goal-id ?goal-id)
				 (plan-id ?plan-id)
				 (skiller ?skiller)
				 (state FORMULATED)
				 (action-name move)
				 (param-values ?robot ?robot-at ?robot-at-side ?robot-to ?robot-to-side))
	(not (plan-action (id ?prev-action-id&:(eq (- ?action-id 1) ?prev-action-id)) (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED|PENDING|WAITING|RUNNING)))
	(or
		(plan-action (skiller ?other-skiller&:(neq ?skiller ?other-skiller)) (action-name move) (state ~FORMULATED&~FINAL) (param-values ?other-robot&:(neq ?robot ?other-robot) ? ? ?robot-to ?robot-to-side))
		(and
			(domain-fact (name at) (param-values ?other-robot&:(neq ?robot ?other-robot) ?robot-to ?robot-to-side))
			(not (plan-action (action-name move) (state RUNNING) (param-values ?other-robot $? ?robot-to ?robot-to-side)))
			(not (plan-action (action-name go-wait) (state RUNNING) (param-values ?other-robot $? ?other-target&:(eq ?other-target (wait-pos ?robot-to ?robot-to-side)))))
		)
	)
	=>
	(bind ?interleaved-id (sym-cat ?class -INTERLEAVED-PLAN-))
	(bind ?interleaved-plan (plan-assert-sequential ?interleaved-id ?goal-id ?robot
	    (plan-assert-action go-wait ?robot ?robot-at ?robot-at-side (wait-pos ?robot-to ?robot-to-side))
	    (plan-assert-action wait ?robot (wait-pos ?robot-to ?robot-to-side) WAIT)
	))
	(modify ?plan-action (param-values ?robot (wait-pos ?robot-to ?robot-to-side) WAIT ?robot-to ?robot-to-side))
	(modify ?plan (suspended TRUE) (suspension-reason (fact-slot-value ?interleaved-plan plan-id)))
)

(defrule execution-monitoring-suspend-plan-to-insert-wait-after-restart
	(declare (salience ?*MONITORING-SALIENCE*))
	(goal (id ?goal-id) (class ?class))
	(goal-meta (goal-id ?goal-id) (retries ~0))
	?plan <- (plan (id ?plan-id) (goal-id ?goal-id) (suspended FALSE))
	?plan-action <- (plan-action (id ?action-id)
	             (goal-id ?goal-id)
				 (plan-id ?plan-id)
				 (skiller ?skiller)
				 (state FORMULATED)
				 (action-name move)
				 (param-values ?robot ?robot-at&:(not (str-index "WAIT-" ?robot-at)) ?robot-at-side ?robot-to ?robot-to-side))
	(not (plan-action (id ?prev-action-id&:(eq (- ?action-id 1) ?prev-action-id)) (goal-id ?goal-id) (plan-id ?plan-id)))
	=>
	(bind ?interleaved-id (sym-cat ?class -INTERLEAVED-PLAN-))
	(bind ?interleaved-plan (plan-assert-sequential ?interleaved-id ?goal-id ?robot
	    (plan-assert-action go-wait ?robot ?robot-at ?robot-at-side (wait-pos ?robot-to ?robot-to-side))
	))
	(modify ?plan-action (param-values ?robot (wait-pos ?robot-to ?robot-to-side) WAIT ?robot-to ?robot-to-side))
	(modify ?plan (suspended TRUE) (suspension-reason (fact-slot-value ?interleaved-plan plan-id)))
)

(defrule execution-monitoring-stop-plan-suspension
	?plan <- (plan (id ?plan-id) (goal-id ?goal-id) (suspended TRUE) (suspension-reason ?interleaved-plan-id))
	?interleaved-plan <- (plan (id ?interleaved-plan-id))
	(not (plan-action (plan-id ?interleaved-plan-id) (goal-id ?goal-id) (state ~FINAL)))
	=>
	(modify ?plan (suspended FALSE))
	(retract ?interleaved-plan)
)


(defrule execution-monitoring-save-last-wait-position
" Save the last wait position of the robot to a fact so that we can avoid moving
 to this position again to not block a spot too long.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (goal (id ?goal-id) (class MOVE-OUT-OF-WAY) (mode DISPATCHED) (params target-pos ?pos))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))
  (not (wm-fact (key monitoring robot last-wait-position args? r ?robot)))
  =>
  (assert (wm-fact (key monitoring robot last-wait-position args? r ?robot) (value ?pos)))
)

(defrule execution-monitoring-retract-last-wait-position
" Retract the last wait position fact once the robot executes a different goal
  or waits at a new position.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  ?wf <- (wm-fact (key monitoring robot last-wait-position args? r ?robot) (value ?pos))

  (goal (id ?goal-id) (class ?class) (mode DISPATCHED) (params $?params))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))

  (test
    (or
      (neq ?class MOVE-OUT-OF-WAY)
      (and
        (eq ?class MOVE-OUT-OF-WAY)
        (not (member$ ?pos ?params))
      )
    )
  )
  =>
  (retract ?wf)
)

(defrule execution-monitoring-set-timeout-move-action-progress
" Set a timeout that measures the progress of actions that move the agent. Should the agent not make progress
  beyond a certain delta threshold within the specified time span, the action will be failed.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (goal (id ?goal-id))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))

  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state RUNNING)
	   (action-name move|go-wait)
  )

  (Position3DInterface (id ?if-id&:(eq ?if-id (remote-if-id ?robot "Pose"))) (translation $?pose))

  (not (progress-timer (plan-id ?plan-id) (action-id ?id)))
  (wm-fact (key refbox game-time) (values $?now))
  =>
  (assert
    (progress-timer
      (plan-id ?plan-id)
	  (action-id ?id)
	  (timeout-duration ?*MOVE-PROGRESS-TIMEOUT*)
	  (start-time ?now)
	  (last-pose ?pose)
	  (counter 0)
    )
  )
)

(defrule execution-monitoring-update-timeout-move-action-progress
" Update the progress-timer of a move action
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (goal (id ?goal-id))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))

  ?p <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state RUNNING)
       (skiller ?skiller)
	   (action-name ?action-name)
  )

  (wm-fact (key refbox game-time) (values $?now))

  (Position3DInterface (id ?if-id&:(eq ?if-id (remote-if-id ?robot "Pose"))) (translation $?pose))

  ?pt <- (progress-timer (plan-id ?plan-id)
					     (action-id ?id)
						 (start-time $?st)
						 (timeout-duration ?timeout&:(timeout ?now ?st ?timeout))
   						 (last-pose $?last-pose)
						 (counter ?counter))
  ?at <- (action-timer (plan-id ?plan-id) (status ?status) (action-id ?id) (timeout-duration ?at-timeout))
  =>
  (printout t "Checking progress of move action "  ?action-name  crlf)

  (bind ?last-x (nth$ 1 ?last-pose))
  (bind ?last-y (nth$ 2 ?last-pose))
  (bind ?curr-x (nth$ 1 ?pose))
  (bind ?curr-y (nth$ 2 ?pose))
  (bind ?delta (+ (abs (- ?last-y ?curr-y)) (abs (- ?last-x ?curr-x))))

  (if (> ?delta 0.5) then
  	(printout t "Robot " ?robot " made sufficient progress (" ?delta "m) on "  ?action-name  crlf)
	(modify ?pt (counter 0) (last-pose ?pose) (start-time ?now))
	(modify ?at (timeout-duration (+ ?at-timeout ?*MOVE-PROGRESS-TIMEOUT*)))
  else
	(if (>= ?counter ?*MOVE-PROGRESS-COUNTER*) then
	  (printout t "   Aborting action " ?action-name " on interface after stuck on small delta" ?skiller crlf)
	  (bind ?m (blackboard-create-msg (str-cat "SkillerInterface::" ?skiller) "StopExecMessage"))
      (bind ?p (modify ?p (state FAILED) (error-msg "Stuck on RUNNING")))
      (fail-action ?p "Unsatisfied precondition")
	else
  	  (printout t "Robot " ?robot " did not make sufficient progress on "  ?action-name  crlf)
	  (modify ?pt (counter (+ 1 ?counter)) (start-time ?now))
	)
  )
)

(defrule execution-monitoring-remove-timeout-move-action-progress
" Remove the progress-timer of a move action if the action is not running anymore
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (goal (id ?goal-id))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))

  (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	   (id ?id) (state ~RUNNING)
  )
  ?pt <- (progress-timer (plan-id ?plan-id) (action-id ?id))
  =>
  (retract ?pt)
)

;======================================Retries=========================================
;

(deffunction should-retry (?an ?error)
	(if (or (eq ?error "Conveyor Align Failed")
	        (eq ?error "Drive To Machine Point Failed")
	        (eq ?error "Motor Move failed")
	    )
	 then
	  (return TRUE)
	)
	(if (or (eq ?error "Unsatisfied precondition")
	        (eq ?error "Invalid parameters")
	        (eq ?error "Object not found") ; this one needs to be added to the respective skill
	    )
	 then
	  (return FALSE)
	)
	(if (or (eq ?an move) (eq ?an go-wait) (eq ?an wp-check)) then
	  (return TRUE)
	)
	(return FALSE)
)

(defrule execution-monitoring-set-mps-side-approachable
	(wm-fact (key domain fact mps-type args? m ?mps $?))
	(wm-fact (key domain objects-by-type mps-side) (is-list TRUE)
	         (values $? ?side&:(member$ ?side (create$ INPUT OUTPUT)) $?))
	(not (domain-fact (name at) (param-values ? ?mps ?side)))
	(not (domain-fact (name mps-side-approachable) (param-values ?mps ?side)))
	=>
	(assert (domain-fact (name mps-side-approachable) (param-values ?mps ?side)))
)

(defrule execution-monitoring-start-retry-action
" For some actions it can be feasible to retry them in case of a failure, e.g. if
  the align failed. If an action failed and the error-msg inquires, that we should
  retry, add a action-retried counter and set the action to FORMULATED
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(goal (id ?goal-id) (mode DISPATCHED) (class ~MOVE-OUT-OF-WAY))
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
	(wm-fact (key central agent robot args? r ?r))
	(not (wm-fact (key central agent robot-lost args? r ?r)))
	(not (wm-fact (key monitoring robot-in-maintenance args? r ?r)))
	(not (wm-fact (key monitoring action-retried args? r ?r a ?an id ?id2&:(eq ?id2 (sym-cat ?id)) m ? g ?goal-id)))
	=>
	(bind ?mps nil)
	(do-for-fact ((?do domain-object)) (and (member$ ?do:name ?param-values) (eq ?do:type mps))
	  (bind ?mps ?do:name)
	)
	(assert
	  (wm-fact (key monitoring action-retried args? r ?r a ?an id (sym-cat ?id) m ?mps g ?goal-id) (value 0))
	)
	(modify ?pa (state FORMULATED) (error-msg ""))
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
	(wm-fact (key central agent robot args? r ?r))
	(not (wm-fact (key central agent robot-lost args? r ?r)))
	(not (wm-fact (key monitoring robot-in-maintenance args? r ?r)))
	(test (eq TRUE (should-retry ?an ?error)))
	?wm <- (wm-fact (key monitoring action-retried args? r ?r a ?an id ?id2&:(eq ?id2 (sym-cat ?id)) m ? g ?goal-id)
	        (value ?tries&:(< ?tries 3)));?*MAX-RETRIES-PICK*
	=>
	(bind ?tries (+ 1 ?tries))
	(modify ?pa (state FORMULATED) (error-msg ""))
	(printout error "Restarted: " ?tries crlf)
	(modify ?wm (value ?tries))
)

(defrule execution-monitoring-repeated-goal-retry-timeout
" If a goal fails a pre-determined number of times, give it a timeout for executability."
	(declare (salience ?*MONITORING-SALIENCE*))
	(goal (id ?goal-id) (mode FINISHED|EVALUATED|RETRACTED) (outcome FAILED))
	?gm <- (goal-meta (goal-id ?goal-id) (retries ?retries&:(> ?retries ?*GOAL-RETRY-MAX*)) (assigned-to ?robot))
	(wm-fact (key refbox game-time) (values $?now))
	=>
	(printout error "Goal " ?goal-id " was retried " ?*GOAL-RETRY-MAX* " times, give it a timeout of " ?*GOAL-RETRY-TIMEOUT* "s." crlf)
	(assert (goal-retry-wait-timer (goal-id ?goal-id) (timeout-duration ?*GOAL-RETRY-TIMEOUT*) (start-time ?now)))
	(assert (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))
	(modify ?gm (retries 0))
)

(defrule execution-monitoring-repeated-goal-retry-timeout-over
	(wm-fact (key refbox game-time) (values $?now))
	?gt <- (goal-retry-wait-timer (goal-id ?goal-id) (start-time $?st) (timeout-duration ?td&:(timeout ?now ?st ?td)))
	?wf <- (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot))
	=>
	(retract ?gt ?wf)
)


; ----------------------- HANDLE WP CHECK FAIL  --------------------------------

(defrule execution-monitoring-wp-check-there-after-wp-put-retry
" If a wp-check action with query THERE fails after a wp-put,
  retry the wp-put as we might have had a gripper issue.
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))

	?pa-check <- (plan-action (id ?id-check) (goal-id ?goal-id) (plan-id ?plan-id)
				 (action-name wp-check)
				 (param-values ?robot ?wp ?mps ?side THERE)
				 (state FAILED))
	?pa-put <- (plan-action (id ?id-put&:(eq (- ?id-check 1) ?id-put)) (goal-id ?goal-id) (plan-id ?plan-id)
				 (action-name wp-put)
				 (param-values ?robot ?wp ?mps ?side ?complexity))
	?wp-atf <- (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?side))

	(not (wm-fact (key monitoring retry-after-sensing args? r ?robot a wp-put id ?id-put-sym&:(eq ?id-put-sym (sym-cat ?id-put)) m ?mps g ?goal-id)))
	?can-hold <- (domain-fact (name can-hold) (param-values ?robot))
	=>
	(printout error "WP " ?wp " was expected to be at " ?mps " (" ?side") but could not be detected. Restarting wp-put!")
	(modify ?pa-check (state FORMULATED) (error-msg ""))
	(modify ?pa-put (state FORMULATED) (error-msg ""))

	(assert
	  (wm-fact (key monitoring retry-after-sensing args? r ?robot a wp-put id (sym-cat ?id-put) m ?mps g ?goal-id))
	)
	(retract ?wp-atf)
	(retract ?can-hold)
	(assert (domain-fact (name holding) (param-values ?robot ?wp)))
	(assert (wm-fact (key domain fact mps-side-free args? m ?mps side ?side) (type BOOL) (value TRUE)))
)

(defrule execution-monitoring-wp-check-there-add-fail-goal-flag-after-retry
" If a wp-check action with query THERE fails, assume that
  the WP was lost and clean up accordingly.
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))

	(plan-action (id ?id-check) (goal-id ?goal-id) (plan-id ?plan-id)
				 (action-name wp-check)
				 (param-values ?robot ?wp ?mps ?side THERE)
				 (state FAILED))
	(plan-action (id ?id-put&:(eq (- ?id-check 1) ?id-put)) (goal-id ?goal-id) (plan-id ?plan-id)
				 (action-name wp-put)
				 (param-values ?robot ?wp ?mps ?side ?complexity))

	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?side))
	(wm-fact (key monitoring retry-after-sensing args? r ?robot a wp-put id ?id-put-sym&:(eq ?id-put-sym (sym-cat ?id-put)) m ?mps g ?goal-id))
	=>
	(printout error "WP " ?wp " was expected to be at " ?mps " (" ?side") but could not be detected. Assume WP was lost.")
	(assert (wm-fact (key monitoring cleanup-wp args? wp ?wp)))
	(assert (wm-fact (key monitoring fail-goal args? g ?goal-id r WP-LOST)))
)

(defrule execution-monitoring-wp-check-absent-add-fail-goal-flag
" If a wp-check action with query ABSENT fails, assume that
  the WP wasn't gripped successfully. Restart the action.
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))

	(plan-action (id ?id) (goal-id ?goal-id) (plan-id ?plan-id)
				 (action-name wp-check)
				 (param-values ?robot ?mps ?side ABSENT)
				 (state FAILED))
	(plan-action (id ?) (goal-id ?goal-id) (plan-id ?plan-id)
				 (action-name wp-get)
				 (param-values ?robot ?wp ?mps ?side $?)
				 (state FAILED))

	(not (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?side)))
	?holding <- (wm-fact (key domain fact holding args? r ?robot wp ?wp))
	=>
	(printout error "No WP was expected to be at " ?mps " (" ?side") but there was one detected, fail the goal.")
	(assert (wm-fact (key monitoring fail-goal args? g ?goal-id r WP-NOT-PICKED)))
	;fix the WM
	(retract ?holding)
	(assert (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?side)))
)


; ----------------------- HANDLE BROKEN MPS -----------------------------------

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
	                    (state PENDING)
	                    (precondition ?grounding-id)
	)
	(grounded-pddl-predicate (grounding ?grounding-id) (is-satisfied FALSE))
	=>
	(fail-action ?pa (str-cat "Action uses broken MPS " ?mps))
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
	   (action-name ?an&~move&~go-wait&~wait))
	(not (wm-fact (key monitoring fail-goal args? g ?goal-id r ?)))
	=>
	(assert (wm-fact (key monitoring fail-goal args? g ?goal-id r BROKEN-MPS)))
)

(defrule execution-monitoring-broken-mps-running-skill-add-fail-goal-flag
" If an action is currently being executed which utilizes a broken machine,
  this can have various outcomes, in either way the goal needs to be failed
  for now.
"
	(declare (salience ?*MONITORING-SALIENCE*))
	(wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(plan-action (id ?id) (plan-id ?plan-id) (goal-id ?goal-id)
	   (state WAITING|RUNNING)
	   (param-values $? ?mps $?)
	   (action-name ?an&~move&~go-wait&~wait&~wp-check&~wp-get))
	(not (wm-fact (key monitoring fail-goal args? g ?goal-id r ?)))
	=>
	(assert (wm-fact (key monitoring fail-goal args? g ?goal-id r INTERACTED-WITH-BROKEN-MPS)))
)


(defrule execution-monitoring-broken-mps-fail-goal
" If the current dispatched goal is marked to be failed and no action is running,
  set the goal to finished and failed
"
	(declare (salience ?*MONITORING-SALIENCE*))
	?fg <-(wm-fact (key monitoring fail-goal args? g ?goal-id r ?reason))
	?g <- (goal (id ?goal-id) (outcome UNKNOWN))
	(not (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (state ~FORMULATED&~PENDING&~FINAL&~FAILED)))
	=>
	(retract ?fg)
	(bind ?msg "")
	(switch ?reason
		(case BROKEN-MPS then
			(bind ?msg "Needs to use a broken MPS")
		)
		(case WP-LOST then
			(bind ?msg "Workpiece lost")
		)
		(case INTERACTED-WITH-BROKEN-MPS then
			(bind ?msg "Interacted with MPS that was broken")
		)
	)
	(printout t "Fail goal " ?goal-id " (" ?msg ")" crlf)
	(modify ?g (mode FINISHED) (outcome FAILED) (error ?reason)
	          (message ?msg))
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
	                                       )
	                                   )
	  (printout t "Exec-Monitoring: Broken Machine " ?wf:key crlf " domain facts flushed!"  crlf)
	  (retract ?wf)
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
)

(defrule execution-monitoring-cleanup-wp-facts
"If a workpiece is lost, it is marked for cleanup.
 In this rule all corresponding fact of a marked workpiece are removed
"
	(declare (salience ?*MONITORING-SALIENCE*))
	?cleanup <- (wm-fact (key monitoring cleanup-wp args? wp ?wp))
	=>
	(do-for-all-facts ((?wf wm-fact)) (and (neq (member$ ?wp (wm-key-args ?wf:key)) FALSE)
	                                       (or
	                                         (wm-key-prefix ?wf:key (create$ domain fact wp-usable))
	                                         (wm-key-prefix ?wf:key (create$ monitoring cleanup-wp))
	                                         (wm-key-prefix ?wf:key (create$ domain fact holding))
	                                       )
	                                  )
	  (printout t "WP-fact " ?wf:key crlf " domain fact flushed!"  crlf)
	  (retract ?wf)
	)
	(do-for-all-facts ((?wf wm-fact)) (and (wm-key-prefix ?wf:key (create$ domain fact wp-at))
	                  (eq ?wp (wm-key-arg ?wf:key wp)))
	  (assert (wm-fact (key domain fact mps-side-free args? m (wm-key-arg ?wf:key m) side (wm-key-arg ?wf:key side)) (type BOOL) (value TRUE)))
	  (printout t "WP-fact " ?wf:key crlf " domain fact flushed!" crlf)
	  (retract ?wf)
	)
	(do-for-all-facts ((?wf wm-fact)) (and (wm-key-prefix ?wf:key (create$ domain fact wp-on-shelf))
	                                       (eq ?wp (wm-key-arg ?wf:key wp)))
	  (assert (wm-fact (key domain fact spot-free args? m (wm-key-arg ?wf:key m) spot (wm-key-arg ?wf:key spot)) (type BOOL) (value TRUE)))
	  (printout t "WP-fact " ?wf:key crlf " domain fact flushed!" crlf)
	  (retract ?wf)
	)
	(assert (wm-fact (key wp-unused args? wp ?wp)))
	(do-for-all-facts ((?g goal))
		(and (eq ?g:outcome UNKNOWN) (member$ ?wp ?g:params))
		(assert (wm-fact (key monitoring fail-goal args? g ?g:id r WP-LOST)))
	)
	(retract ?cleanup)
)

; ----------------------- HANDLE SAME SIDE MOVE -------------------------------

(defrule execution-monitoring-handle-same-side-move
" Allows move to current location by setting mps mps-side to be approachable.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (plan-action
	   (state PENDING)
	   (action-name move)
	   (param-values $? ?mps ?mps-side ?mps ?mps-side $?)
  )
  (not (wm-fact (key domain fact mps-side-approachable args? m ?mps side ?mps-side)))
  =>
  (printout t "Move with equal start and target location "
               ?mps " " ?mps-side " is detected. Set side approachable" crlf)
  (assert (wm-fact (key domain fact mps-side-approachable args? m ?mps side ?mps-side) (type BOOL) (value TRUE)))
)

(defrule execution-monitoring-wp-put-slide-cc
" Adapting params of wp-put-slide-cc if the number of rings of rs-before differs
  from the number of rings the rs is filled with "
  (declare (salience ?*MONITORING-SALIENCE*))
  ?pa <- (plan-action (state PENDING)
               (action-name wp-put-slide-cc)
               (id ?action-id)
               (param-values ?robot ?wp ?rs ?rs-before ?rs-after)
                      (precondition ?grounding-id)
  )
  (grounded-pddl-predicate (grounding ?grounding-id) (is-satisfied FALSE))
  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-num&:(not (eq ?rs-num ?rs-before))))
  (wm-fact (key domain fact rs-inc args? summand ?rs-num sum ?rs-num-after))
  =>
  (printout t "execuction monitoring wp-put-slide-cc: rs-before is unequal to rs-filled-with" crlf)
  (modify ?pa (param-values ?robot ?wp ?rs ?rs-num ?rs-num-after))
)

(defrule execution-monitoring-correct-inconsistent-slide-counter
" Whenever a wp-put-slide-cc and a mount-ring action are both RUNNING, the one
  that finishes first will cause the second action to apply the wrong effects.
  Either way, the result are two rs-filled-with facts, one with the initial
  count + 1, one with the initial count - payment-value.
  The correct solution however would be the initial count - payment-value + 1.
"
  (declare (salience ?*SALIENCE-HIGH*))
  ?wm1 <- (wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-num))
  ?wm2 <- (wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-num2&:(< (sym-to-int ?rs-num) (sym-to-int ?rs-num2))))
  =>
  (printout t "execuction monitoring: two rs-filled-with facts detected" crlf)
  (retract ?wm1 ?wm2)
  (assert (domain-fact (name rs-filled-with) (param-values ?rs (int-to-sym (+ (sym-to-int ?rs-num) 1)))))
)

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
                                   cs-retrieve-cap|
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

(defrule execution-monitoring-fix-rs-mount-counter
  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (id ?id)
                      (state PENDING) (executable FALSE)
                      (action-name rs-mount-ring1|
                                   rs-mount-ring2|
                                   rs-mount-ring3)
                      (param-values ?rs $?o-args ?rs-before ?rs-after ?rs-req))
	(wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-new&:(neq ?rs-new ?rs-before)))
	(wm-fact (key domain fact rs-sub args? minuend ?rs-new
                                         subtrahend ?rs-req
                                         difference ?bases-remain))
  =>
  (modify ?pa (param-values ?rs $?o-args ?rs-new ?bases-remain ?rs-req))
)

; ----------------------- RESTORE AND INSERT -------------------------------

; robot-lost: robot is not reachable via bb
; once a robot is lost, remove it's active goals and remove robot-waiting
; once a robot is in maintenance, additionally clear it's worldmodel
; a robot that was lost or in maintenance can only return if
; 1) it is not lost anymore
; 2) it is not in maintenance
; 3) it is not still in cleanup

(defrule execution-monitoring-enter-maintenance
	(wm-fact (key monitoring robot-in-maintenance args? r ?robot))
	=>
	(assert (wm-fact (key monitoring stop-goals-of-robot args? r ?robot)))
	(assert (wm-fact (key monitoring reset-wm-of-robot args? r ?robot)))
)

(defrule execution-monitoring-leave-maintenance
	?r-in <- (wm-fact (key monitoring robot-in-maintenance args? r ?robot))
	?r-out <- (wm-fact (key monitoring robot-out-of-maintenance args? r ?robot))
	(not (wm-fact (key monitoring stop-goals-of-robot args? r ?robot)))
	(not (wm-fact (key monitoring reset-wm-of-robot args? r ?robot)))
	=>
	(retract ?r-in)
	(retract ?r-out)
)

(defrule execution-monitoring-robot-lost
	(declare (salience ?*SALIENCE-HIGH*))
	(wm-fact (key central agent robot args? r ?robot))
	(HeartbeatInterface (id ?id&:(str-index ?robot ?id)) (alive FALSE))
	(not (wm-fact (key central agent robot-lost args? r ?robot)))
	?rw <- (wm-fact (key central agent robot-waiting args? r ?robot))
	=>
	(assert (wm-fact (key central agent robot-lost args? r ?robot)))
	(assert (wm-fact (key monitoring stop-goals-of-robot args? r ?robot)))
)

(defrule execution-monitoring-robot-reachable
	(declare (salience ?*SALIENCE-HIGH*))
	(wm-fact (key central agent robot args? r ?robot))
	(HeartbeatInterface (id ?id&:(str-index ?robot ?id)) (alive TRUE))
	(not (wm-fact (key monitoring stop-goals-of-robot args? r ?robot)))
	(not (wm-fact (key monitoring reset-wm-of-robot args? r ?robot)))
	?rl <- (wm-fact (key central agent robot-lost args? r ?robot))
	=>
	(retract ?rl)
)

(defrule execution-monitoring-remove-waiting-robot-lost
	(declare (salience ?*SALIENCE-HIGH*))
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key central agent robot-lost args? r ?robot))
	?rw <- (wm-fact (key central agent robot-waiting args? r ?robot))
	=>
	(retract ?rw)
)

(defrule execution-monitoring-remove-waiting-robot-maintenance
	(declare (salience ?*SALIENCE-HIGH*))
	(wm-fact (key monitoring robot-in-maintenance args? r ?robot))
	?rw <- (wm-fact (key central agent robot-waiting args? r ?robot))
	=>
	(retract ?rw)
)

(defrule execution-monitoring-add-waiting-robot
	(declare (salience ?*SALIENCE-HIGH*))
	(wm-fact (key central agent robot args? r ?robot))
	(not (wm-fact (key central agent robot-lost args? r ?robot)))
	(not (wm-fact (key monitoring robot-in-maintenance args? r ?robot)))
	(not (wm-fact (key monitoring stop-goals-of-robot args? r ?robot)))
	(not (wm-fact (key monitoring reset-wm-of-robot args? r ?robot)))
	(not (wm-fact (key central agent robot-waiting args? r ?robot)))
	=>
	(assert (wm-fact (key central agent robot-waiting args? r ?robot)))
)

(defrule execution-monitoring-stop-goals-for-robot
	(declare (salience ?*SALIENCE-HIGH*))
	(domain-facts-loaded)
	(wm-fact (key monitoring stop-goals-of-robot args? r ?robot))
	?g <- (goal (id ?g-id) (mode DISPATCHED) (type ACHIEVE))
	(goal-meta (goal-id ?g-id) (assigned-to ?robot))
	=>
	(if (not
	(do-for-all-facts
		((?plan-action plan-action))
		(and (eq ?plan-action:goal-id ?g-id)
		     (neq ?plan-action:state FORMULATED)
		     (neq ?plan-action:state FINAL)
		)
		(printout t "   Aborting action " ?plan-action:action-name " on interface" ?plan-action:skiller crlf)
		(bind ?m (blackboard-create-msg (str-cat "SkillerInterface::" ?plan-action:skiller) "StopExecMessage"))
		(blackboard-send-msg ?m)
		(do-for-fact ((?sae skill-action-execinfo))
			(and (eq ?sae:goal-id ?plan-action:goal-id)
			     (eq ?sae:plan-id ?plan-action:plan-id)
			     (eq ?sae:action-id ?plan-action:id)
			)
			(do-for-fact ((?skill skill))
				(and (eq ?skill:id ?sae:skill-id)
				     (eq ?skill:skiller ?sae:skiller)
				)
				(retract ?skill)
			)
			(retract ?sae)
		)
		(modify ?plan-action (state EXECUTION-FAILED))
	))
	 then
		(do-for-fact
			((?formulated-action plan-action) (?final-action plan-action))
			(and (eq ?formulated-action:goal-id ?g-id)
			  (eq ?final-action:goal-id ?g-id)
			  (eq ?formulated-action:state FORMULATED)
			  (eq ?final-action:state FINAL))
			(modify ?formulated-action (state FAILED))
		)
	)
	(modify ?g (mode FINISHED) (outcome FAILED))
)

(defrule execution-monitoring-stop-goals-for-robot-done
	(declare (salience ?*SALIENCE-HIGH*))
	(domain-facts-loaded)
	?sgor <- (wm-fact (key monitoring stop-goals-of-robot args? r ?robot))
	(not (and (goal (id ?g-id) (mode ~FORMULATED&~RETRACTED) (type ACHIEVE))
		    (goal-meta (goal-id ?g-id) (assigned-to ?robot))
		)
	)
	=>
	(retract ?sgor)
)

(defrule execution-monitoring-reset-wm-of-robot
	(declare (salience ?*SALIENCE-HIGH*))
	(domain-facts-loaded)
	?reset <- (wm-fact (key monitoring reset-wm-of-robot args? r ?robot))
	; reset wm only when no goal is running
	(not (and (goal (id ?g-id) (mode ~FORMULATED&~RETRACTED) (type ACHIEVE))
		    (goal-meta (goal-id ?g-id) (assigned-to ?robot))
		)
	)
	=>
	(do-for-all-facts ((?wm wm-fact))
		(and (eq (wm-key-arg ?wm:key r) ?robot)
		    (not (wm-key-prefix ?wm:key (create$ refbox robot task seq)))
		)
		(retract ?wm)
	)

	(do-for-all-facts ((?df domain-fact)) (str-index ?robot (implode$ ?df:param-values))
		(retract ?df)
	)

	(do-for-all-facts ((?wsmf wm-sync-map-fact)) (eq (wm-key-arg ?wsmf:wm-fact-key r) ?robot)
		(retract ?wsmf)
	)

	(assert (domain-fact (name at) (param-values ?robot START INPUT))
	        (domain-fact (name can-hold) (param-values ?robot))
	        (domain-object (name ?robot) (type robot))
	        (wm-fact (key central agent robot args? r ?robot))
	        (wm-fact (key central agent robot-lost args? r ?robot))
	)
	(retract ?reset)
)

; ----------------------- BS TRACKING -------------------------------
(defrule execution-monitoring-bs-side-in-use
"If a BS is part of a goal's operation, assert a fact to indicate this state."
	(declare (salience ?*SALIENCE-HIGH*))
  (domain-fact (name bs-side-in-use) (param-values ?bs ?bs-side))
  (domain-fact (name mps-type) (param-values ?bs BS))
	(not (wm-fact (key mps meta bs-side-in-use args? bs ?bs $?)))
	(goal (id ?goal-id) (mode EXPANDED|COMMITTED|DISPATCHED) (sub-type SIMPLE))
	(plan-action (action-name wp-get) (goal-id ?goal-id) (param-values $? ?bs ?bs-side $?)
               (state FORMULATED|PENDING|WAITING|RUNNING))
	=>
	(assert (wm-fact (key mps meta bs-side-in-use args? bs ?bs bs-side ?bs-side goal ?goal-id)))
)

(defrule execution-monitoring-bs-side-not-in-use
"Retract BS in use fact if it is no longer in use."
	(declare (salience ?*SALIENCE-HIGH*))
	?wm <- (wm-fact (key mps meta bs-side-in-use args? bs ?bs bs-side ?bs-side goal ?goal-id))
	(wm-fact (key domain fact mps-type args? $? ?bs $? BS $?))
	(not (and (goal (id ?goal-id) (mode EXPANDED|COMMITTED|DISPATCHED))
	          (plan-action (action-name wp-get) (goal-id ?goal-id) (param-values $? ?bs ?bs-side $?) (state FORMULATED|PENDING|WAITING|RUNNING))
	))
	=>
	(retract ?wm)
)

(deffunction modify-all-plan-action-param-bs-side (?goal-id ?bs ?bs-side ?free-side)
"modifies all plan actions for a goal to use a specific BS side"
	(delayed-do-for-all-facts ((?pa plan-action))
		(and (eq ?pa:goal-id ?goal-id)
		     (or (eq ?pa:state FORMULATED) (eq ?pa:state PENDING))
		     (member$ ?bs ?pa:param-values)
		     (member$ ?bs-side ?pa:param-values)
		)
		(bind ?param-values (replace-member$ ?pa:param-values (create$ ?bs ?free-side) (create$ ?bs ?bs-side)))
		(modify ?pa (param-values ?param-values))
        )
)

(defrule execution-monitoring-modify-transport-goal-plans-if-bs-side-in-use-CX
"if a bs side is in use, modify MOUNT-RING, MOUNT-CAP and INSTRUCT-BS goals to use the free side."
	(declare (salience ?*MONITORING-SALIENCE*))
	(wm-fact (key mps meta bs-side-in-use args? bs ?bs bs-side ?bs-side goal ?ogid))
	(not (wm-fact (key mps meta bs-side-in-use args? $? bs ?bs bs-side ?other-side&:(neq ?bs-side ?other-side) $?)))
	(wm-fact (key domain fact mps-type args? $? ?bs $? BS $?))
	?g <- (goal (id ?goal-id&:(neq ?ogid ?goal-id)) (class MOUNT-RING) (mode FORMULATED|SELECTED))
	?gm <- (goal-meta (goal-id ?goal-id) (order-id ?orderid) (ring-nr ONE))
	?ig <- (goal (id ?ig-id) (class INSTRUCT-BS-DISPENSE-BASE) (mode FORMULATED) (params wp ?wp target-mps ?bs target-side ?bs-side base-color ?base-color))
	?igm <- (goal-meta (goal-id ?ig-id) (order-id ?orderid))
        =>
	(switch ?bs-side
		(case INPUT then
			(bind ?free-side OUTPUT))
		(case OUTPUT then
			(bind ?free-side INPUT))
	)
	(modify ?ig (params wp ?wp target-mps ?bs target-side ?free-side base-color ?base-color))
)

(defrule execution-monitoring-modify-transport-goal-plans-if-bs-side-in-use-C0
"if a bs side is in use, modify MOUNT-RING, MOUNT-CAP and INSTRUCT-BS goals to use the free side."
	(declare (salience ?*MONITORING-SALIENCE*))
	(wm-fact (key mps meta bs-side-in-use args? bs ?bs bs-side ?bs-side goal ?ogid))
	(not (wm-fact (key mps meta bs-side-in-use args? $? bs ?bs bs-side ?other-side&:(neq ?bs-side ?other-side) $?)))
	(wm-fact (key domain fact mps-type args? $? ?bs $? BS $?))
  (domain-fact (name order-complexity) (param-values ?orderid C0))
	?g <- (goal (id ?goal-id&:(neq ?ogid ?goal-id)) (class MOUNT-CAP) (mode FORMULATED|SELECTED))
	?gm <- (goal-meta (goal-id ?goal-id) (order-id ?orderid))
	?ig <- (goal (id ?ig-id) (class INSTRUCT-BS-DISPENSE-BASE) (mode FORMULATED) (params wp ?wp target-mps ?bs target-side ?bs-side base-color ?base-color))
	?igm <- (goal-meta (goal-id ?ig-id) (order-id ?orderid))
        =>
	(switch ?bs-side
		(case INPUT then
			(bind ?free-side OUTPUT))
		(case OUTPUT then
			(bind ?free-side INPUT))
	)
	(modify ?ig (params wp ?wp target-mps ?bs target-side ?free-side base-color ?base-color))
)
(defrule execution-monitoring-modify-instruct-to-match-waiting-transport
"if a bs side needs to be instructed and the robot is waiting at the wrong side, just adjust the instruct goal"
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (id ?goal-id) (class MOUNT-RING|MOUNT-CAP|PAY-FOR-RINGS-WITH-BASE) (mode DISPATCHED))
  (domain-fact (name mps-type) (param-values ?bs BS))
  (plan-action (action-name wp-get) (goal-id ?goal-id) (param-values $? ?bs ?bs-side $?)
               (state FORMULATED|PENDING))
  ?gm <- (goal-meta (goal-id ?goal-id) (order-id ?orderid))
  ?ig <- (goal (id ?ig-id) (class INSTRUCT-BS-DISPENSE-BASE) (mode FORMULATED) (params wp ?wp target-mps ?bs target-side ?o-side&:(neq ?o-side ?bs-side) base-color ?base-color))
  ?igm <- (goal-meta (goal-id ?ig-id) (order-id ?orderid))
        =>
  (printout error "We suck at managing the BS" crlf)
  (modify ?ig (params wp ?wp target-mps ?bs target-side ?bs-side base-color ?base-color))
)

(defrule execution-monitoring-modify-pay-for-rings-with-base-goals-plans-if-bs-side-in-use
"if a bs side is in use, modify payment goals to use the free side."
	(declare (salience ?*MONITORING-SALIENCE*))
	(wm-fact (key mps meta bs-side-in-use args? bs ?bs bs-side ?bs-side goal ?ogid))
	(not (wm-fact (key mps meta bs-side-in-use args? $? bs ?bs bs-side ?other-side&:(neq ?bs-side ?other-side) $?)))
	(wm-fact (key domain fact mps-type args? $? ?bs $? BS $?))
	?g <- (goal (id ?g-id) (class PAY-FOR-RINGS-WITH-BASE) (mode FORMULATED|SELECTED)  (params wp ?wp $?))
	?ig <- (goal (id ?ig-id) (class INSTRUCT-BS-DISPENSE-BASE) (mode FORMULATED) (params wp ?wp target-mps ?bs target-side ?bs-side base-color ?base-color))
	=>
	(switch ?bs-side
		(case INPUT then
			(bind ?free-side OUTPUT))
		(case OUTPUT then
			(bind ?free-side INPUT))
	)
	(modify ?ig (params wp ?wp target-mps ?bs target-side ?free-side base-color ?base-color))
)

; ----------------------- HANDLE FAILING INSTRUCT -----------------------------------
(defrule execution-monitoring-break-instruct-fails
"When an INSTRUCT fails on an MPS (except BS|DS), break the machine."
	(declare (salience ?*MONITORING-SALIENCE*))
	?g <- (goal (class INSTRUCT-CS-BUFFER-CAP|INSTRUCT-CS-MOUNT-CAP|INSTRUCT-RS-MOUNT-RING) (mode EVALUATED) (outcome FAILED) (error ~WP-LOST) (params $? target-mps ?mps $?))
	?wm <- (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
	(not (goal (class RESET-MPS) (params mps ?mps) (mode ~RETRACTED)))
	=>
	(bind ?goal-id (sym-cat RESET-MPS - (gensym*)))
	(assert (goal (id ?goal-id) (class RESET-MPS) (params mps ?mps) (mode EXPANDED) (sub-type SIMPLE) (type ACHIEVE)))
	(assert (goal-meta (goal-id ?goal-id) (assigned-to central)))
	(assert
	    (plan (id (sym-cat ?goal-id -PLAN)) (goal-id ?goal-id))
	    (plan-action (id 1) (plan-id (sym-cat ?goal-id -PLAN)) (goal-id ?goal-id)
	        (action-name reset-mps)
	        (param-names m)
	        (param-values ?mps))
	)
)

(defrule execution-monitoring-reformulate-instruct-fails-bs-ds
"When an INSTRUCT fails on a BS|DS, reformulate the instruct goal."
	(declare (salience ?*MONITORING-SALIENCE*))
	?g <- (goal (class INSTRUCT-BS-DISPENSE-BASE|INSTRUCT-DS-DELIVER) (id ?id) (mode RETRACTED) (outcome FAILED))
	?p <- (plan (goal-id ?id))
	=>
	(do-for-all-facts ((?plan-action plan-action)) (eq ?plan-action:goal-id ?id)
		(retract ?plan-action)
	)
	(modify ?g (mode FORMULATED) (outcome UNKNOWN))
	(retract ?p)
)

(defrule execution-monitoring-detect-disconnected-robot
  (declare (salience ?*MONITORING-SALIENCE*))
	(wm-fact (key central agent robot args? r ?robot))
	?hbi <- (HeartbeatInterface (id ?id&:(str-index ?robot ?id)) (alive FALSE))
	(not (wm-fact (key central agent robot-lost args? r ?robot)))
	=>
	(printout error "Robot " ?robot  " lost, removing from worldmodel" crlf)

	(do-for-fact ((?si SkillerInterface)) (str-index ?robot ?si:id)
		(retract ?si)
	)
	(assert (reset-robot-in-wm ?robot))
)

; ----------------------- Monitor points for plan-actions -----------------------------------
(defrule execution-monitoring-add-point-change-detector-plan-action-running
	(wm-fact (key refbox points ?team) (value ?points))
	(wm-fact (key refbox team-color) (value ?team))

	(plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id) (state RUNNING) (action-name ?action-name))
	(not (wm-fact (key monitoring points-for-action args? goal-id ?goal-id plan-id ?plan-id action-id ?action-id-sym&:(eq (sym-cat ?action-id) ?action-id-sym) action-name ?action-name)))
	=>
	(assert (wm-fact (key monitoring points-for-action args? goal-id ?goal-id plan-id ?plan-id action-id (sym-cat ?action-id) action-name ?action-name) (type UINT) (value ?points)))
)

(defrule execution-monitoring-add-point-change-detector-plan-action-final-start-timer
	(wm-fact (key monitoring points-for-action args? goal-id ?goal-id plan-id ?plan-id action-id ?action-id-sym action-name ?action-name) (value ?recorded-points))
	(plan-action (id ?action-id&:(eq (sym-cat ?action-id) ?action-id-sym)) (goal-id ?goal-id) (plan-id ?plan-id) (action-name ?action-name) (state FINAL))
	(not (points-timer (goal-id ?goal-id) (action-id ?action-id) (plan-id ?plan-id) (action-name ?action-name)))
	(wm-fact (key refbox game-time) (values $?now))
	(wm-fact (key config rcll wait-for-points) (value TRUE))
	=>
	(assert (points-timer (goal-id ?goal-id) (action-id ?action-id) (plan-id ?plan-id) (action-name ?action-name) (start-time ?now) (timeout-duration ?*WAIT-FOR-POINTS-TIMEOUT*)))
)

(defrule execution-monitoring-add-point-change-detector-plan-action-final-end-timer
	(wm-fact (key refbox game-time) (values $?now))
	?pt <- (points-timer (goal-id ?goal-id) (action-id ?action-id) (plan-id ?plan-id) (action-name ?action-name)
            (start-time $?st)
            (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
	?wf <- (wm-fact (key monitoring points-for-action args? goal-id ?goal-id plan-id ?plan-id action-id ?action-id-sym&:(eq (sym-cat ?action-id) ?action-id-sym)  action-name ?action-name) (value ?recorded-points))
	(wm-fact (key refbox points ?team) (value ?points))
	=>
	(retract ?pt ?wf)
	(assert (wm-fact (key monitoring action-estimated-score args? goal-id ?goal-id plan-id ?plan-id action-id (sym-cat ?action-id)  action-name ?action-name) (value (- ?points ?recorded-points))))
)

; TODO: revisit
;(defrule execution-monitoring-correct-slide-counter
;	?monitoring-fact <- (wm-fact (key monitoring action-estimated-score args? goal-id ?goal-id plan-id ?plan-id action-id ?action-id action-name wp-put-slide-cc) (value 0))
;	(goal (id ?goal-id) (params $? target-mps ?rs $?))
;	?request <- (wm-fact (key request pay args? ord ?order m ?rs ring ?ring seq ?seq prio ?prio) (values status ? assigned-to $?assigned-goals&:(member$ ?goal-id ?assigned-goals)))
;	?rs-filled <- (domain-fact (name rs-filled-with) (param-values ?rs ?bases-filled))
;	(domain-fact (name rs-inc) (param-values ?bases-now ?bases-filled))
;	=>
;	(printout t "Detected no point increase after put slide, re-issuing request and adjusting counter")
;	(retract ?monitoring-fact)
;	(modify ?rs-filled (param-values ?rs ?bases-now))
;	(modify ?request (values status OPEN assgined-to))
;)

; ----------------------- EXPLORATION -----------------------------------
(defrule execution-monitoring-generate-navgraph-when-all-tags-found
  "Generate the navgraph when all the mps tags where found."
  (wm-fact (key refbox phase) (value PRODUCTION))
  (forall
    (wm-fact (key domain fact mps-team args? m ?mps col ?any-team-color))
    (wm-fact (key game found-tag name args? m ?mps ))
  )
  (not (navgraph-all-tags-triggered))
  (blackboard-interface (id "/navgraph-generator")
	                    (type "NavGraphGeneratorInterface"))
=>
  (printout t "Triggering NavGraph generation with Ground-truth" crlf)
  (navgraph-add-all-new-tags)
  (assert (navgraph-all-tags-triggered))
)

; ----------------------- GOAL RESTRICTIONS -----------------------------------
(defrule execution-monitoring-remove-restriction-if-robot-lost
  "Remove goal restrictions if the robot is lost."
  (wm-fact (key central agent robot-lost args? r ?robot))
  ?g <- (goal (id ?ogid) (mode FORMULATED))
  ?gm <- (goal-meta (goal-id ?ogid) (restricted-to ?robot))
  =>
  (modify ?gm (restricted-to nil))
)

(defrule execution-monitoring-remove-restriction-robot-occupied
  (domain-object (type robot) (name ?robot))
  (goal (id ?gid) (class ~ENTER-FIELD) (mode DISPATCHED))
  (goal-meta (goal-id ?gid) (assigned-to ?robot))
  ?g <- (goal (id ?ogid) (mode FORMULATED))
  ?gm <- (goal-meta (goal-id ?ogid) (restricted-to ?robot))
  =>
  (modify ?gm (restricted-to nil))
)

(defrule execution-monitoring-change-robot-pos-preemptively
" Update the robot position if it is actually not at the current position anymore.
"
  (declare (salience ?*MONITORING-SALIENCE*))
  (goal (id ?goal-id))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))

  ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
    (id ?id) (state RUNNING)
    (action-name move|go-wait) (param-values ?robot ?mps ?side $?to)
  )
  ?at <- (domain-fact (name at) (param-values ?robot ?mps ?side))
  (domain-fact (name zone-content) (param-values ?zone ?mps))

  (Position3DInterface (id ?if-id&:(eq ?if-id (remote-if-id ?robot "Pose"))) (translation ?x ?y ?z))
  (test (> (distance ?x ?y (nth$ 1 (zone-coords ?zone)) (nth$ 2 (zone-coords ?zone))) 1))
  =>
  (bind ?zone (zone-string-to-sym-dash (zone-str-from-coords ?x ?y)))
  (modify ?at (param-values ?robot ?zone WAIT))
  (assert (domain-fact (name mps-side-approachable) (param-values ?mps ?side)))
  (modify ?pa (param-values ?robot ?zone WAIT $?to))
)
