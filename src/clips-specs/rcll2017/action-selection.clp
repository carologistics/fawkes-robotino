
(defrule action-selection-select
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
											(action-name ?action-name))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (status FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
	(modify ?pa (status PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (plan-id ?plan-id) (status ~FINAL)))
	=>
	(modify ?g (mode COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan-action (status FAILED))
	=>
	(modify ?g (mode FAILED))
)
