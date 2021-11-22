
(defrule goal-expander-create-visit
	?g <- (goal (mode SELECTED) (id ?id) (class VISIT) (params machine ?m))
	=>
	(assert
		(plan (id VISIT-WITH1) (goal-id ?id)
	        (type SEQUENTIAL))
		(plan-action (id 1) (plan-id VISIT-WITH1) (goal-id ?id)
	             (action-name visit) (skiller "/robot1/Skiller")
	             (param-values ?m OUTPUT CYAN))

		(plan (id VISIT-WITH2) (goal-id ?id)
	        (type SEQUENTIAL))
		(plan-action (id 1) (plan-id VISIT-WITH2) (goal-id ?id)
	             (action-name visit) (skiller "/robot2/Skiller")
	             (param-values ?m OUTPUT CYAN))

		(plan (id VISIT-WITH3) (goal-id ?id)
	        (type SEQUENTIAL))
		(plan-action (id 1) (plan-id VISIT-WITH3) (goal-id ?id)
	             (action-name visit) (skiller "/robot3/Skiller")
	             (param-values ?m OUTPUT CYAN))
		
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
