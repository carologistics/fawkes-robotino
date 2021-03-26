;---------------------------------------------------------------------------
;  run-on-idle.clp - CLIPS executive - goal to run all sub-goals in parallel
;
;  Created: Mon Mar 8 2021
;  Copyright  2021  Luis Laas
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Sub-type: RUN-SUBGOALS-ON-IDLE
; Perform: all sub-goals in parallel
; Params: (params continue-on FAILED|REJECTED) to not stop SELECTION of
;         other goals when a sub-goal FAILED/is REJECTED
;         other values will be ignored
; Succeed: if all sub-goal succeeds
; Fail:    if at least one sub-goal fails
; Reject:  if any sub-goal is rejected but no sub-goal failed
;
; A run-on-idle parent goal will run all sub-goals in parallel by
; continuously SELECTING all sub-goals with the highest priority.
; If any goal fails, the parent fails. If any sub-goal is rejected,
; the parent is rejected. If all goals have been completed successfully,
; the parent goal succeeds.
; The run-on-idle goal can be configured to continue execution of other goals
; upon encountering a FAILED or REJECTED sub-goal by including
; 'continue-on FAILED' or 'continue-on REJECTED' in the goal params
; (continue-on FAILED implies continue-on REJECTED).

;
; Interactions:
; - User FORMULATES goal
; - User SELECTS goal
; - User EXPANDS goal, consisting of:
;   * create goals with parent ID equal the run-on-idle goal ID
;   * set run-on-idle goal mode to EXPANDED
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: if all sub-goals rejected -> REJECT
; - Automatic: SELECT all formulated sub-goals with highest priority,
;              SELECT next batch of sub-goals once all
;              previously SELECTED sub-goals are DISPATCHED|FINISHED|RETRACTED.
; - User: handle sub-goal expansion, committing, dispatching, evaluating
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

(defglobal
  ?*MAX-RUNNING-TASKS* = 3
)

(deftemplate running-tasks
  (slot number(type NUMBER))
)

(defrule init-running-tasks
(not (running-tasks (number ?some-number)))
=>
 (printout t "running tasks initialized" crlf)
 (assert (running-tasks (number 0)))
)

(deftemplate idle-robot
  (slot robot(type SYMBOL))
)

(deffunction run-on-idle-stop-execution (?params ?sub-goal-outcome)
	(return (and (or (eq ?sub-goal-outcome FAILED)
	                 (eq ?sub-goal-outcome REJECTED))
	             (not (member$ (create$ continue-on FAILED) ?params))
	             (not (member$ (create$ continue-on ?sub-goal-outcome) ?params))))
)

(defrule run-on-idle-goal-expand-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-ON-IDLE)
	             (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	            (error NO-SUB-GOALS)
	            (message (str-cat "No sub-goal for run-on-idle goal '" ?id "'")))
)

(defrule run-on-idle-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-ON-IDLE)
	             (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(modify ?gf (mode COMMITTED))
)

(defrule run-on-idle-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-ON-IDLE)
	             (mode COMMITTED)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(modify ?gf (mode DISPATCHED))
)

(defrule run-on-idle-subgoals-select
	(idle-robot (robot ?robot))
	?r <- (running-tasks (number ?running-tasks))
	(test (< ?running-tasks ?*MAX-RUNNING-TASKS*))
	(goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-ON-IDLE)
	      (mode DISPATCHED) (params $?params))
	(wm-fact (key refbox game-time) (values $?game-time))

	?g <- (goal (parent ?id) (id ?sub-id) (type ACHIEVE) (mode FORMULATED)
	      (meta delivery-begin ?delivery-begin )(priority ?prio))
	(not (goal (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (meta delivery-begin ?o-delivery-begin )(priority ?o-prio&:(> (+ ?o-prio (- (nth$ 1 ?game-time)?o-delivery-begin)) (+ ?prio (- (nth$ 1 ?game-time) ?delivery-begin))))))
	(not (goal (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (meta delivery-begin ?o-delivery-begin )(priority ?prio&:(and (> (nth$ 1 ?game-time) 800) (> ?prio 200)))))
	(not (goal (parent ?id) (type ACHIEVE)
	           (outcome ?outcome&:(run-on-idle-stop-execution ?params ?outcome))))
	=>
	(modify ?g (mode SELECTED))
	(assert (running-tasks (number (+ ?running-tasks 1))))
 	(retract ?r)
	(do-for-all-facts ((?i idle-robot))
	(printout t "retract idle robot " crlf)
	(retract ?i)
	)
)

(defrule run-on-idle-subgoal-reject-other-subgoal-rejected
	(goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-ON-IDLE)
	      (mode DISPATCHED) (params $?params))
	(wm-fact (key refbox game-time) (values $?game-time))

	?sg <- (goal (parent ?id) (id ?sub-id) (type ACHIEVE) (mode FORMULATED)
	               (meta delivery-begin ?delivery-begin)(priority ?prio))
	(not (goal (parent ?id) (type ACHIEVE) (mode FORMULATED)
	            (meta delivery-begin ?o-delivery-begin)(priority ?o-prio&:(< (+ ?o-prio (- (nth$ 1 ?game-time) ?o-delivery-begin)) (+ ?prio (- (nth$ 1 ?game-time) ?delivery-begin))))))
	(not (goal (parent ?id) (type ACHIEVE) (mode SELECTED|EXPANDED|COMMITTED)))
	(goal (parent ?id) (type ACHIEVE)
	      (outcome ?outcome&:(run-on-idle-stop-execution ?params ?outcome)))
	=>
	(modify ?sg (mode FINISHED) (outcome REJECTED))
)

(defrule run-on-idle-subgoal-evaluated
	(goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-ON-IDLE)
	      (mode DISPATCHED))
	?sg <- (goal (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

;(defrule run-on-idle-goal-finish-all-subgoals-retracted
;	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type RUN-SUBGOALS-ON-IDLE)
;	             (mode DISPATCHED))
;	(not (goal (parent ?id) (type ACHIEVE)
;	           (acquired-resources $?acq&:(> (length ?acq) 0))))
;	(not (goal (parent ?id) (type ACHIEVE) (mode ~RETRACTED)))
;	=>
;	(if (not (do-for-fact ((?g goal))
;	                      (and (eq ?g:parent ?id) (eq ?g:outcome FAILED))
;		(modify ?gf (mode FINISHED) (outcome ?g:outcome)
;		          (error SUB-GOAL-FAILED ?g:id)
;		          (message (str-cat "Sub-goal '" ?g:id "' of run-on-idle goal '"
;		                            ?id "' has failed")))))
;	 then
;		(if (not (do-for-fact ((?g goal))
;		                    (and (eq ?g:parent ?id) (eq ?g:outcome REJECTED))
;			(modify ?gf (mode FINISHED) (outcome ?g:outcome)
;			          (error SUB-GOAL-REJECTED ?g:id)
;			          (message (str-cat "Sub-goal '" ?g:id "' of run-on-idle goal '"
;			                            ?id "' was rejected")))))
;		 then
;			(modify ?gf (mode FINISHED) (outcome COMPLETED))
;		)
;	)
;)
