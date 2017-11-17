(defrule action-selection-select
  (Position3DInterface (id "Pose") (translation $?robot-trans))
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
											(action-name goto) (param-values $?params))
  (navgraph-node (name ?goal-node&:(eq ?goal-node (nth$ 1 ?params))) (pos $?goal-pos))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (status PENDING|WAITING|RUNNING|FAILED)))
	(not (and
    (plan-action (plan-id ?plan-id) (id ?other-id&~?id) (status FORMULATED)
      (action-name goto) (param-values $?otherp))
    (navgraph-node (name ?othern&:(eq ?othern (nth$ 1 ?otherp)))
      (pos $?other-pos&:
        (navgraph-closer (subseq$ ?robot-trans 1 2) ?other-pos ?goal-pos)
    ))
  ))
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
