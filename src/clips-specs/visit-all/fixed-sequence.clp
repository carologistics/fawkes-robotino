
(defrule goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id TESTGOAL))
	=>
	(assert
	  (plan (id TESTGOAL-PLAN) (goal-id TESTGOAL)
	        (type SEQUENTIAL))
	(plan-action (id 1) (plan-id TESTGOAL-PLAN) (goal-id TESTGOAL)
	             (action-name visit) (skiller "/robot1/Skiller")
	             (param-values C-BS OUTPUT CYAN))
	 )
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-maintain-beacon
	?p <- (goal (mode SELECTED) (id ?parent-id))
	?g <- (goal (id ?goal-id) (class SEND-BEACON) (mode FORMULATED)
	            (parent ?parent-id))
=>
	(modify ?p (mode COMMITTED) (committed-to ?goal-id))
)

(defrule goal-expander-send-beacon-signal
	?p <- (goal (mode DISPATCHED) (id ?parent-id))
	?g <- (goal (id ?goal-id) (class SEND-BEACON) (mode SELECTED)
	            (parent ?parent-id))
=>
	(assert
	 (plan (id BEACON-PLAN) (goal-id ?goal-id)
	        (type SEQUENTIAL))
	        (plan-action (id 1) (plan-id BEACON-PLAN) (goal-id ?goal-id)
	                     (action-name send-beacon))
	)
	(modify ?g (mode EXPANDED))
)
