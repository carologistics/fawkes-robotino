;---------------------------------------------------------------------------
;  central-run-parallel.clp - CLIPS executive - goal to run all sub-goals in 
;                                               parallel on the central agent
;
;  Created: Wed 19 May 2021 01:10:00 CET
;  Copyright  2021  Daniel Swoboda
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: CENTRAL-RUN-SUBGOALS-IN-PARALLEL
; Perform: all sub-goals in parallel
; Succeed: if all sub-goal succeeds
; Fail:    if at least one sub-goal fails
;
; A CENTRAL-RUN-PARALLEL parent goal will select all currently executable 
; sub-goals and therefore mark them for execution. It will assign subsequently
; selected leaf goals with no asssigned-to entry to the robot for which the 
; selection currently is executed. Executable simple goals that are assigned to
; the central instance are executed immediately after the selection process.
; If all goals have been completed successfully, the parent goal succeeds.
;
;


(defrule central-run-parallel-subgoals-select
	""
	(goal (id ?id) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
	      (mode SELECTED))
	(goal (parent ?id) (id ?sub-id) (mode FORMULATED)
	      (priority ?prio) (is-executable TRUE))
	(not (goal (parent ?id) (mode FORMULATED)
	           (priority ?o-prio&:(> ?o-prio ?prio)) (is-executable TRUE)))
	=>
	(do-for-all-facts ((?g goal)) (and (eq ?g:parent ?id)
	                                   (eq ?g:mode FORMULATED)
									   (eq ?g:is-executable TRUE)
	                                   (eq ?g:priority ?prio))
		(modify ?g (mode SELECTED))
	)
)

(defrule central-run-parallel-goal-finish-all-subgoals
	?gf <- (goal (id ?id) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL) (mode FORMULATED))
	(not (goal (parent ?id) (mode FORMULATED|SELECTED)))
	=>
	(if (not (do-for-fact ((?g goal))
	                      (and (eq ?g:parent ?id) (eq ?g:outcome FAILED))
		(modify ?gf (mode FINISHED) (outcome ?g:outcome)
		          (error SUB-GOAL-FAILED ?g:id)
		          (message (str-cat "Sub-goal '" ?g:id "' of CENTRAL-RUN-PARALLEL goal '"
		                            ?id "' has failed")))))
	 then
		(modify ?gf (mode FINISHED) (outcome COMPLETED))
	)
)
