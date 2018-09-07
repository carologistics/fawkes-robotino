;---------------------------------------------------------------------------
;  action-selection-done-r2-r3.clp - Planning for RCLL production with the plugin clips-smt
;  - rules to finish goal for R-2 and R-3
;
;  Created: Tue Jul 20 14:38
;  Copyright  2018 Igor Bongartz <bongartz@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule action-selection-done-r-2
	"If no plan-action is not FINAL finish goal for R-2"
	(wm-fact (key config rcll robot-name) (value "R-2"))
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (plan-id ?plan-id) (status ~FINAL)))
	=>
	(printout t "Robot " (cx-identity) " has completed his plan-actions of " ?goal-id " " ?plan-id crlf)
	(assert (wm-fact (key plan-action ?goal-id ?plan-id done-r2) ) )
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-done-r-3
	"If no plan-action is not FINAL finish goal for R-3"
	(wm-fact (key config rcll robot-name) (value "R-3"))
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (plan-id ?plan-id) (status ~FINAL)))
	=>
	(printout t "Robot " (cx-identity) " has completed his plan-actions of " ?goal-id " " ?plan-id crlf)
	(assert (wm-fact (key plan-action ?goal-id ?plan-id r-3-done) ) )
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)
