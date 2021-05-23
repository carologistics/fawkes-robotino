;---------------------------------------------------------------------------
;  central-run-one.clp - CLIPS executive - goal to run sub-goals
;                                          for the central agent
;
;  Created: Wed 19 May 2021 00:39:00 CET
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;             2021  Daniel Swoboda
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
; - User SELECTS goal
; - User EXPANDS goal, consisting of:
;   * create goals with parent ID equal the RUN-ONE goal ID
;   * set RUN-ONE goal mode to EXPANDED
; - Automatic: if no sub-goal formulated -> FAIL
; - Automatic: if all sub-goals rejected -> REJECT
; - Automatic: take highest FORMULATED sub-goal and COMMIT to
; - Automatic: DISPATCH committed sub-goal by SELECTING it
; - User: handle sub-goal expansion, commiting, dispatching
; - Automatic: when sub-goal is EVALUATED, outcome determines parent goal:
;   * REJECTED: mode EXPANDED (re-try with other sub-goal)
;   * FAILED: mode FINISHED, outcome FAILED, message
;   * COMPLETED: mode FINISHED, outcome COMPLETED
;   -> Sub-goal is RETRACTED.
; User: EVALUATE goal
; User: RETRACT goal

(defrule central-run-one-goal-expand-failed
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) (mode EXPANDED))
	(not (goal (type ACHIEVE) (parent ?id)))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED)
	            (message (str-cat "No sub-goal for RUN-ONE goal '" ?id "'")))
)

(defrule central-run-one-goal-commit
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) (mode EXPANDED))
	(goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?priority))
	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (priority ?priority2&:(> ?priority2 ?priority))))
	=>
	(modify ?gf (mode COMMITTED) (committed-to ?sub-goal))
)

(defrule central-run-one-goal-dispatch
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS)
	             (mode COMMITTED)
	             (required-resources $?req)
	             (acquired-resources $?acq&:(subsetp ?req ?acq)))
	=>
	(modify ?gf (mode DISPATCHED))
)

(defrule central-run-one-goal-subgoals-select
	(goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS)
	      (mode DISPATCHED) (params $?params))
	?g <- (goal (parent ?id) (id ?sub-id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?prio) (is-executable TRUE))
	(not (goal (parent ?id) (type ACHIEVE) (mode FORMULATED)
	           (priority ?o-prio&:(> ?o-prio ?prio)) (is-executable TRUE)))
	(not (goal (parent ?id) (type ACHIEVE) (mode SELECTED|EXPANDED|COMMITTED)))
	=>
	(modify ?g (mode SELECTED))
)


(defrule central-run-one-goal-subgoal-evaluated
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))
	=>
	(modify ?sg (mode RETRACTED))
)

(defrule central-run-one-goal-subgoal-rejected-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome REJECTED))
	=>
	(modify ?gf (mode EXPANDED) (committed-to nil))
)

(defrule central-run-one-goal-subgoal-failed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome FAILED))
	=>
	(modify ?gf (mode FINISHED) (outcome FAILED) (committed-to nil)
	            (error SUB-GOAL-FAILED)
	            (message (str-cat "Sub-goal '" ?sub-goal "' of RUN-ONE goal '" ?id "' has failed")))
)

(defrule central-run-one-goal-subgoal-completed-resources-clear
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) (mode DISPATCHED)
	             (committed-to ?sub-goal))
	?sg <- (goal (id ?sub-goal) (parent ?id) (acquired-resources)
	             (type ACHIEVE) (mode RETRACTED) (outcome COMPLETED))
	=>
	(modify ?gf (mode FINISHED) (outcome COMPLETED) (committed-to nil))
)

(defrule central-run-one-goal-subgoal-completed-clear-uncompleted
	(goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) 
	      (mode FINISHED) (outcome COMPLETED))
	?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED))
	=>
	(retract ?sg)
)

(defrule central-run-one-set-to-expanded
	"when a central-run-parall goal is selected it is automatically expanded"
	?gf <- (goal (id ?id) (type ACHIEVE) 
	             (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS) (mode SELECTED))
	=>
	(modify ?gf (mode EXPANDED))
)
