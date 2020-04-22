;---------------------------------------------------------------------------
;  schedule.clp - CLIPS executive - goal to run all sub-goals to completion
;
;  Created: Mon Jun 04 15:00:20 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;             2020       Mostafa Gomaa  [mostafa.go@gmail.com]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: RUN-ALL-OF-SUBGOALS
; Perform: one goal at a time, ordered by goal priority
; Succeed: if all sub-goal succeeds
; Fail:    if exactly one sub-goal fails
; Reject:  if any sub-goals is rejected
;
; A RUN-ALL parent goal will order the goals by priority and then
; start performing them in order. If any goal fails, the parent
; fails. If any sub-goal is rejected, the parent is rejected. If all
; goals have been completed successfully, the parent goal succeeds.
;
; Interactions:
; - User: FORMULATES goal
; - User: SELECTS goal
;   On SELECTED goal
;    * User FORMULATES sub-goals with parent ID equal the RUN-ALL goal ID
;                                                     (populating the children)
;    * Automatic: FAIL goal: if no sub-goal formulated
;    * Automatic: REJECT goal: if all sub-goals rejected
;    * USER: SELECTS sub-goal(s)
; - Automatic: if all selected sub-goal EXPANDED   -> goal EXPANDED
;                                                         (bottom up expantion)
; - Automatic: if root with all sub-goals expanded -> goal COMMITTED
; - Automatic: if parent committed                 -> goal COMMITTED
;         (top down committment to one or more goals with the hightst priority)
; - Automatic: if a sub-goal DISPATCHED            -> goal  DISPATCHED
;                                                       (bottom up DISPATCHING)
; - User: handle sub-goal expansion, committing, dispatching, evaluating
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * REJECTED: mode FINISHED, outcome REJECTED, message
;   * FAILED: mode FINISHED, outcome FAILED, message
;   * COMPLETED: mode FINISHED, outcome COMPLETED
;   -> Sub-goal is RETRACTED.
; User: EVALUATE goal
; User: RETRACT goal



(defrule schedule-goal-expand-failed
     (declare (salience ?*SALIENCE-LOW*))
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS)
							 (mode SELECTED))
 	(not (goal (type ACHIEVE) (parent ?id) (mode FORMULATED|SELECTED|EXPANDED)))
  	(not (plan (goal-id ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
					(error NO-SUB-GOALS)
					(message (str-cat "No sub-goals or plans for SCHEDULE goal '" ?id "'" )))
)

(defrule schedule-goal-commit-to-all-subgoals
     (declare (salience ?*SALIENCE-HIGH*))
     (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
     ?gf <- (goal (id ?g-id) (parent ?pg) (committed-to $?committed)
                  (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
     ;Goal is root || parent is committed
     (or (goal (id ?pg) (mode COMMITTED|DISPATCHED) (committed-to $? ?g-id $?))
         (not (goal (id ?pg))))
     ;Sub-goal is expanded
     ?sgf <- (goal (id ?sub-goal) (parent ?g-id) (params ?sub-params)
                   (committed-to nil) (type ACHIEVE) (mode EXPANDED))
     (not (test (member$ ?sub-goal ?committed)))
     ;Scheduled but not yet committed
     ;(schedule (id ?sched-id) (mode COMMITTED) (scheduled ?goal-id))
     ;(test (member$ ?goal-id ?scheduled))
     ;(test (member$ ?sub-goal ?scheduled))
    =>
	(modify ?gf  (committed-to (create$ ?committed ?sub-goal)))
 )


(defrule schedule-goal-commit-to-scheduled-plans
     (declare (salience ?*SALIENCE-HIGH*))
     (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
	?gf <- (goal (id ?g-id) (parent ?pg) (committed-to $?committed)
                  (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
     ;Goal is root || parent is committed
     (or (goal (id ?pg) (mode COMMITTED|DISPATCHED) (committed-to $? ?g-id $?))
         (not (goal (id ?pg))))
     ;Plan is schedueled
     (plan (id ?plan-id) (goal-id ?g-id))
     (schedule-event (sched-id ?s-id) (entity ?plan-id) (at START)
                     (scheduled TRUE) (scheduled-start ?scheduled-start)
                     (duration ?scheduled-duration))
     (not (test (member$ ?plan-id ?committed)))
   =>
	(modify ?gf  (committed-to (create$ ?committed ?plan-id)))
 )


(defrule schedule-goal-commit
     (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
	?gf <- (goal (id ?g-id) (parent ?pg) (committed-to $?committed)
                  (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
     ;Goal is root || parent is committed
     (or (goal (id ?pg) (mode COMMITTED|DISPATCHED) (committed-to $? ?g-id $?))
         (not (goal (id ?pg))))
     (not (goal (id ?sub-goal&:(not (member$ ?sub-goal ?committed))) (parent ?g-id)))
     (not (and (plan (id ?plan-id&:(not (member$ ?plan-id ?committed))) (goal-id ?g-id))
               (schedule-event (sched-id ?schedule-id) (entity ?plan-id) (at START)
                               (scheduled TRUE))))
     =>
	(modify ?gf (mode COMMITTED))
)

(defrule schedule-plan-dispatch
     (time ?now-sec ?now-msec)
     (schedule (id ?schedule-id) (start-time ?start-sec $?))
	?gf <- (goal (id ?goal-id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode COMMITTED)
	             (committed-to ?schedule-id $? ?plan-id $?)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
     ?pf <- (plan (id ?plan-id) (goal-id ?goal-id))
	(schedule-event (sched-id ?schedule-id) (entity ?plan-id) (at START)
                     (scheduled TRUE) (scheduled-start ?scheduled-start))
     (test (> ?now-sec (+ ?start-sec ?scheduled-start)))
     =>
	(modify ?gf (mode DISPATCHED))
     (modify ?pf (start-time (now)))
)


(defrule schedule-goal-dispatch
	?gf <- (goal (id ?goal-id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode COMMITTED)
	             (committed-to $? ?sub-goal $?)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	(goal (id ?sub-goal) (parent ?goal-id) (type ACHIEVE) (mode DISPATCHED))
	=>
	(modify ?gf (mode DISPATCHED))
)



(defrule schedule-goal-subgoal-evaluated
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode DISPATCHED)
	             (committed-to $? ?sub-goal $?))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
     (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?id)
       (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
         (retract ?a))
       (retract ?p))

	(modify ?sg (mode RETRACTED))
)

(defrule schedule-goal-subgoal-rejected-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS)
                  (committed-to $? ?sub-goals $?))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode RETRACTED) (outcome REJECTED))
	=>
	(modify ?gf (mode FINISHED) (outcome REJECTED) (committed-to (create$ ))
	            (error SUB-GOAL-REJECTED)
	            (message (str-cat "Sub-goal '" ?sub-goal "' of SCHEDULE goal '" ?id
				      "' was rejected")))
)


(defrule schedule-goal-subgoal-failed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode DISPATCHED)
	             (committed-to $? ?sub-goal $?))
	?sg <- (goal (id ?sub-goal) (type ACHIEVE) (parent ?id) (acquired-resources)
	             (mode EVALUATED) (outcome FAILED))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED) (committed-to (create$ ))
					(error SUB-GOAL-FAILED ?sub-goal)
					(message (str-cat "Sub-goal '" ?sub-goal "' of SCHEDULE goal '" ?id "' has failed")))
)

(defrule schedule-goal-subgoal-completed-one-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode DISPATCHED)
                  (committed-to $?committed))
	(forall (goal (id ?sub-goal&:(member$ ?sub-goal ?committed)) (type ACHIEVE) (parent ?id))
             (goal (id ?sub-goal) (acquired-resources) (mode RETRACTED) (outcome COMPLETED))
             )
	(goal (parent ?id) (type ACHIEVE) (mode EXPANDED))
	=>
	(modify ?gf (mode EXPANDED) (committed-to (create$)))
)

(defrule schedule-goal-subgoal-completed-all-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type SCHEDULE-SUBGOALS) (mode DISPATCHED)
	             (committed-to $?committed))
	(forall (goal (id ?sub-goal&:(member$ ?sub-goal ?committed)) (type ACHIEVE) (parent ?id))
             (goal (id ?sub-goal) (acquired-resources) (mode RETRACTED) (outcome COMPLETED))
             )
	;?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	;             (type ACHIEVE) (mode RETRACTED) (outcome COMPLETED))
	(not (goal (parent ?id) (type ACHIEVE) (mode EXPANDED)))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED) (committed-to (create$ )))
)

