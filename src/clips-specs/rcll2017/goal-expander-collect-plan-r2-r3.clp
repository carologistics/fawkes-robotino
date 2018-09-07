;---------------------------------------------------------------------------
;  goal-expander-collect-plan-r2-r3.clp - Planning for RCLL production with the plugin clips-smt
;  - collect plan actions for R-2, R-3
;  - expand for R-2, R-3

;  Created: Tue Feb 20 14:38
;  Copyright  2018 Igor Bongartz <bongartz@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;---------------------------------------------------------------------------
; If the amount of actions collected corresponds to the amount of overall plan-actions
; robots R-2 and R-3 are ready for expansion
;---------------------------------------------------------------------------

(defrule production-add-plan-actions-collected-r-2
	"Robot R-2 did not collect all plan-actions yet"
	(wm-fact (key config rcll robot-name) (value "R-2"))
	(not (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-2)) )

	(wm-fact (key plan-action ?goal-id ?plan-id  amount-plan-actions) (value ?amount-plan-actions))
	?apac <- (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-2) (value ?amount-plan-actions-collected))

	; Some action exists which has not yet been added
	(wm-fact (key plan-action ?goal-id ?plan-id ?id action))
	(not (wm-fact (key plan-action ?goal-id ?plan-id ?id action-added-r-2)) )
=>
	(assert (wm-fact (key plan-action ?goal-id ?plan-id ?id action-added-r-2)) )
	(modify ?apac (value (+ ?amount-plan-actions-collected 1)) )
)

(defrule production-add-plan-actions-collected-r-3
	"Robot R-3 did not collect all plan-actions yet"
	(wm-fact (key config rcll robot-name) (value "R-3"))
	(not (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-3)) )

	(wm-fact (key plan-action ?goal-id ?plan-id  amount-plan-actions) (value ?amount-plan-actions))
	?apac <- (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-3) (value ?amount-plan-actions-collected))

	; Some action exists which has not yet been added
	(wm-fact (key plan-action ?goal-id ?plan-id ?id action))
	(not (wm-fact (key plan-action ?goal-id ?plan-id ?id action-added-r-3)) )
=>
	(assert (wm-fact (key plan-action ?goal-id ?plan-id ?id action-added-r-3)) )
	(modify ?apac (value (+ ?amount-plan-actions-collected 1)) )
)

(defrule production-no-call-clips-smt-r-2
	"Robot R-2 did collect all plan-actions"
	?g <- (goal (id ?goal-id&PRODUCE-C3|PRODUCE-C0) (mode SELECTED))
	(wm-fact (key config rcll robot-name) (value "R-2"))
	(not (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-2)) )

	; Test that all actions have been added
	(wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions) (value ?amount-plan-actions))
	?apac <- (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-2) (value ?amount-plan-actions-collected))
	(test (<= ?amount-plan-actions ?amount-plan-actions-collected))
=>
	(retract ?apac)
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id))
		(wm-fact (key plan-action ?goal-id ?plan-id expanded-r-2))
	)
)

(defrule production-no-call-clips-smt-r-3
	"Robot R-3 did collect all plan-actions"
	?g <- (goal (id ?goal-id&PRODUCE-C3|PRODUCE-C0) (mode SELECTED))
	(wm-fact (key config rcll robot-name) (value "R-3"))
	(not (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-3)) )

	; Test that all actions have been added
	(wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions) (value ?amount-plan-actions))
	?apac <- (wm-fact (key plan-action ?goal-id ?plan-id amount-plan-actions-collected-r-3) (value ?amount-plan-actions-collected))
	(test (<= ?amount-plan-actions ?amount-plan-actions-collected))
=>
	(printout t "Expand for R-3 with " ?amount-plan-actions " and collected " ?amount-plan-actions-collected crlf)
	(retract ?apac)
	(assert
		(plan (id ?plan-id) (goal-id ?goal-id))
		(wm-fact (key plan-action ?goal-id ?plan-id expanded-r-3))
	)
)

;---------------------------------------------------------------------------
; Rules to determine a goal as EXPANDED
;---------------------------------------------------------------------------

(defrule production-expand-r-2
	"Expand goal if it is marked ready for expansion for robot R-2"
	?g <- (goal (id ?goal-id&PRODUCE-C3|PRODUCE-C0) (mode SELECTED))
	(wm-fact (key config rcll robot-name) (value "R-2"))
	?ex <- (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-2))
	(plan-action)
=>
	(printout t "Expand plan " ?plan-id " for R-2" crlf)
	(retract ?ex)
	(modify ?g (mode EXPANDED))
)

(defrule production-expand-r-3
	"Expand goal if it is marked ready for expansion for robot R-3"
	?g <- (goal (id ?goal-id&PRODUCE-C3|PRODUCE-C0) (mode SELECTED))
	(wm-fact (key config rcll robot-name) (value "R-3"))
	?ex <- (wm-fact (key plan-action ?goal-id ?plan-id expanded-r-3))
	(plan-action)
=>
	(printout t "Expand plan " ?plan-id " for R-3" crlf)
	(retract ?ex)
	(modify ?g (mode EXPANDED))
)
