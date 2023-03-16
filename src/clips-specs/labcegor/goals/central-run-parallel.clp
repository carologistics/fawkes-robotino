;---------------------------------------------------------------------------
;  central-run-parallel.clp - CLIPS executive - goal to run all sub-goals in
;                                               parallel on the central agent
;
;  Created: Wed Dec 16 2020
;  Copyright  2020  Tarik Viehmann
;			  2021  Daniel Swoboda
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: CENTRAL-RUN-SUBGOALS-IN-PARALLEL
; Perform: all sub-goals in parallel
; Params: (params continue-on FAILED|REJECTED) to not stop SELECTION of
;         other goals when a sub-goal FAILED/is REJECTED
;         other values will be ignored
; Succeed: if all sub-goal succeeds
; Fail:    if at least one sub-goal fails
; Reject:  if any sub-goal is rejected but no sub-goal failed
;
; A RUN-PARALLEL parent goal will run all sub-goals in parallel by
; continuously SELECTING all sub-goals with the highest priority.
; If any goal fails, the parent fails. If any sub-goal is rejected,
; the parent is rejected. If all goals have been completed successfully,
; the parent goal succeeds.
; The RUN-PARALLEL goal can be configured to continue execution of other goals
; upon encountering a FAILED or REJECTED sub-goal by including
; 'continue-on FAILED' or 'continue-on REJECTED' in the goal params
; (continue-on FAILED implies continue-on REJECTED).

;
; Interactions:
; - User FORMULATES goal
; - User SELECTS goal
; - Automatic: expand goal (since goal trees are static, simple swtich)
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: if all sub-goals rejected -> REJECT
; - User: handle sub-goal selection, expansion, committing, dispatching,
;         evaluating
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * REJECTED or FAILED: Reject formulated sub-goals
;                         (unless configured otherwise),
;                         wait for completion of started sub-goals,
;                         message
;   * COMPLETED: wait for completion of other sub-goals
;   -> Sub-goal is RETRACTED
; - Automatic: once all sub-goals are RETRACTED, finish the parent goal:
;   * if a sub-goal FAILED: outcome FAILED
;   * else if a sub-goal was REJECTED: outcome REJECTED
;   * else (all sub-goals were COMPLETED): outcome COMPLETED
; - User: EVALUATE goal
; - User: RETRACT goal

(defrule central-run-parallel-goal-expand-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
	             (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	            (error NO-SUB-GOALS)
	            (message (str-cat "No sub-goal for RUN-PARALLEL goal '" ?id "'")))
)

(defrule central-run-parallel-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
	             (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode COMMITTED))
)


; (deffunction goal-production-find-a-robot (?task)
;   "Find a free robot"
;   (bind ?assn-robot nil)
;   (if (eq ?assn-robot nil) then
;     (if (not (do-for-all-facts ((?f goal-meta))
; 			                (and (neq ?f:assigned-to robot1) (eq ?task PRIMARY_TASK))
; 			                (bind ?assn-robot robot1)
;              )
;         ) then 
;         (if (not (do-for-all-facts ((?f goal-meta))
; 			                (and (neq ?f:assigned-to robot2) (eq ?task SECONDARY_TASK))
; 			                (bind ?assn-robot robot2)
;                  )
;             ) then 
;             (if (not (do-for-all-facts ((?f goal-meta))
; 			                                  (and (neq ?f:assigned-to robot3)  (eq ?task SECONDARY_TASK))
; 			                                  (bind ?assn-robot robot3)
;                      )
;                 ) then 
;                   (bind ?assn-robot nil) (printout t "assn-robot has been assigned as nil")
;             )
;         )
;     )
;   )
;   (return ?assn-robot)
; )



(defrule central-run-parallel-goal-select
	?tree <- (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL) (mode ~FORMULATED&~RETRACTED) (priority ?p-priority) )

	?child <- (goal (id ?sub-goal) (parent ?id) (sub-type SIMPLE) (mode FORMULATED))
	(wm-fact (key central agent robot args? r ?curr-robot))
	?gm <- (goal-meta (goal-id ?sub-goal) (sub-task-type ?task) (assigned-to nil))
	(test (or (and (eq ?task SECONDARY_TASK) (or (eq ?curr-robot robot2) (eq ?curr-robot robot3)))
	          (and (eq ?task PRIMARY_TASK) (eq ?curr-robot robot1))))
	=>
	(modify ?gm (assigned-to ?curr-robot))
	(modify ?child (mode SELECTED))
	
)



; (defrule central-run-parallel-goal-select
; 	?tree <- (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL) (priority ?p-priority) )
	
; 	(not (goal (id ~?id) (mode ~RETRACTED) (priority ?p-priority2&:(> ?p-priority2 ?p-priority))))

; 	?child <- (goal (id ?sub-goal) (parent ?id) (sub-type SIMPLE) (mode FORMULATED))
; 	(wm-fact (key central agent robot args? r ?curr-robot))
; 	?gm <- (goal-meta (goal-id ?sub-goal) (sub-task-type ?task) (assigned-to nil))
; 	(test (or (and (eq ?task SECONDARY_TASK) (or (eq ?curr-robot robot2) (eq ?curr-robot robot3)))
; 	          (and (eq ?task PRIMARY_TASK) (eq ?curr-robot robot1))))
; 	=>
; 	(modify ?gm (assigned-to ?curr-robot))
; 	(modify ?child (mode SELECTED))
	
; )

(defrule central-run-parallel-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
	             (mode COMMITTED)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(modify ?gf (mode DISPATCHED))
)

(defrule central-run-parallel-subgoal-evaluated
	(goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
	      (mode DISPATCHED))
	?sg <- (goal (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

(defrule central-run-parallel-goal-finish-all-subgoals-retracted
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
	             (mode DISPATCHED) (meta $?meta&:(not (member$ do-not-finish ?meta))))
	(not (goal (parent ?id) (type ACHIEVE)
	           (acquired-resources $?acq&:(> (length ?acq) 0))))
	(not (goal (parent ?id) (type ACHIEVE) (mode ~RETRACTED)))
	=>
	(if (not (do-for-fact ((?g goal))
	                      (and (eq ?g:parent ?id) (eq ?g:outcome FAILED))
		(modify ?gf (mode FINISHED) (outcome ?g:outcome)
		          (error SUB-GOAL-FAILED ?g:id)
		          (message (str-cat "Sub-goal '" ?g:id "' of RUN-PARALLEL goal '"
		                            ?id "' has failed")))))
	 then
		(if (not (do-for-fact ((?g goal))
		                    (and (eq ?g:parent ?id) (eq ?g:outcome REJECTED))
			(modify ?gf (mode FINISHED) (outcome ?g:outcome)
			          (error SUB-GOAL-REJECTED ?g:id)
			          (message (str-cat "Sub-goal '" ?g:id "' of RUN-PARALLEL goal '"
			                            ?id "' was rejected")))))
		 then
			(modify ?gf (mode FINISHED) (outcome COMPLETED))
		)
	)
)

(defrule central-run-parallel-goal-finish-all-subgoals-finished-completed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
	             (mode DISPATCHED) (meta $?meta&:(not (member$ do-not-finish ?meta))))
	(not (goal (parent ?id) (type ACHIEVE) (mode RETRACTED|FINISHED) (outcome ~COMPLETED)))
	(not (goal (parent ?id) (type ACHIEVE) (mode ~FINISHED&~RETRACTED)))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED))
)
