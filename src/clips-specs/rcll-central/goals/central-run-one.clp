;---------------------------------------------------------------------------
;  central-run-one.clp - CLIPS executive - goal to run sub-goals
;                                          for the central agent
;
;  Created: Wed 19 May 2021 00:39:00 CET
;  Copyright  2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: CENTRAL-RUN-ONE-OF-SUBGOALS
; Perform: one goal at a time, ordered by goal priority
; Succeed: if exactly one sub-goal succeeds
; Fail:    if exactly one sub-goal fails
;
; A CENTRAL-RUN-ONE parent goal will order the executable goals by priority and
; then start performing them in order.  If any goal succeeds or fails, the
; parent succeeds or fails respectively.
;
; Interactions:
; - User FORMULATES goal
; - AUTOMATIC: SELECT executable sub-goal with highest priority if SELECTED
; - User: handle sub-goal dispatching, evaluating
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * FAILED: mode FINISHED, outcome FAILED, message
;   * COMPLETED: mode FINISHED, outcome COMPLETED
; User: EVALUATE goal


(defrule central-run-one-goal-select-child
	"Select the exectuable child with the highest priority."
	?gf <- (goal (id ?id) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) 
			(mode SELECTED) (is-executable TRUE))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) 
			(mode FORMULATED) (is-executable TRUE) (priority ?priority1))

	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (priority ?priority2&:(> ?priority2 ?priority1)) (is-executable TRUE)))
	=>
	(modify ?sg (mode SELECTED))
)

(defrule central-run-one-goal-subgoal-finished
	"Set the goal to finished when one subgoal is finished."
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS)
			(mode ~FINISHED))
	(goal (parent ?id) (type ACHIEVE) (mode FINISHED) (outcome COMPLETED))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED))
)

(defrule central-run-one-goal-subgoal-failed
	"Fail the goal if any of the child goals fail to propagate error handling."
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS)
			(mode ~FINISHED))
	?sg <- (goal (parent ?id) (type ACHIEVE) (mode FINISHED) (outcome FAILED))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
					(error SUB-GOAL-FAILED ?sg)
					(message (str-cat "Sub-goal '" ?sg "' of CENTRAL-ONE-ALL goal '" ?id "' has failed")))
)
