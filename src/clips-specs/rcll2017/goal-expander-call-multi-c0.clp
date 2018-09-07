;---------------------------------------------------------------------------
;  goal-expander-multi-c0.clp - Planning for RCLL production with the plugin clips-smt
;  - call produce-c0 for multi robots with C0 order is O1 with no delivery window
;	 - this call happens after that R-1 and R-2 finished with their plan for a C3 order
;
;  Created: Tue Feb 20 14:38
;  Copyright  2018 Igor Bongartz <bongartz@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


(defrule production-call-clips-smt-c0
	"Call plugin clips-smt"
	(goal (id ?goal-id&PRODUCE-C0) (mode SELECTED))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key refbox team-color) (value ?team-color&CYAN|MAGENTA))

	; Does an order of wanted complexity exists?
	(wm-fact (key domain fact order-complexity args? ord ?order-id&O1 com ?complexity&C0) (value TRUE))

	; Was the order already fulfilled?
	(not (wm-fact (key domain fact order-fulfilled args? ord ?order-id) (value TRUE)))

	; There was no plan for this goal requested yet?
	(not (plan-requested ?goal-id ?order-id))

	; Only R-1 should run the planner
	(wm-fact (key config rcll robot-name) (value "R-1"))

	; Extract time
	(wm-fact (key refbox game-time) (values ?sec ?sec-2))

=>
	(printout t "SMT plan call for " ?order-id crlf)
	; Only use two robots because the third one is waiting for delivery
	(bind ?robots (create$ R-1 R-2))
	(bind ?p
	  (smt-create-data
			(smt-create-robots ?team-color ?robots)
			(smt-create-machines ?team-color)
			(smt-create-orders ?order-id ?complexity)
			(smt-create-rings)
			?sec
	  )
	)
	(assert (plan-requested ?goal-id ?order-id))
	(smt-request "smt-plan" ?p)
)
