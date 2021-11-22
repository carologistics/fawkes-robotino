
(defrule robot-1-goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id ?gid) (params target-mps ?m exec-robot ROB1))
	=>
	(assert
	  (plan (id ROBOT1-PLAN) (goal-id ?gid)
                 (type SEQUENTIAL))
	(plan-action (id 1) (plan-id ROBOT1-PLAN) (goal-id ?gid)
	             (action-name visit) (skiller "/robot1/Skiller")
	             (param-values ?m OUTPUT CYAN))
	 )
	(modify ?g (mode EXPANDED))
)

(defrule robot-2-goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id ?gid) (params target-mps ?m exec-robot ROB2))
	=>
	(assert
	  (plan (id ROBOT2-PLAN) (goal-id ?gid)
                 (type SEQUENTIAL))
	(plan-action (id 1) (plan-id ROBOT2-PLAN) (goal-id ?gid)
	             (action-name visit) (skiller "/robot2/Skiller")
	             (param-values ?m OUTPUT CYAN))
	 )
	(modify ?g (mode EXPANDED))
)

(defrule robot-3-goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id ?gid) (params target-mps ?m exec-robot ROB3))
	=>
	(assert
	  (plan (id ROBOT3-PLAN) (goal-id ?gid)
                 (type SEQUENTIAL))
	(plan-action (id 1) (plan-id ROBOT3-PLAN) (goal-id ?gid)
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
