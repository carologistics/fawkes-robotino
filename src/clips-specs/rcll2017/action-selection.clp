

(defrule action-selection-exploration-select
	?pa <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION)
			(id ?id) (status FORMULATED)
			(action-name ?action-name)
			(param-values $?param-values))
	(goal (id EXPLORATION) (mode DISPATCHED))
	(not (plan-action (goal-id EXPLORATION) (status PENDING|WAITING|RUNNING)))
	(not (plan (id EXPLORE-ZONE) (goal-id EXPLORATION)))
	(not (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (status FORMULATED) (id ?oid&: (< ?oid ?id))))
	=>
	(printout t "Selected next Exploration move action" ?action-name ?param-values crlf)
	(modify ?pa (status PENDING))
)

(defrule action-selection-exploration-done
	(not (plan-action (goal-id EXPLORATION) (plan-id EXPLORATION-PLAN) (status ?status&~FINAL&~FAILED)))
	?g <- (goal (id EXPLROATION) (mode DISPATCHED))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-seleection-explorezone-release-locks
	?p <- (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
	(not (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (status ?s&~FINAL&~FORMULATED&~FAILED))) 
	?rl <- (plan-action (plan-id EXPLORE-ZONE) (action-name release-locks) (status FORMULATED))
	=>
	(modify ?rl (status PENDING))
)


(defrule action-selection-explorezone-done
	?p <- (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
	(not (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (status ?s&~FINAL&~FORMULATED&~FAILED))) 
	(plan-action (plan-id EXPLORE-ZONE) (action-name release-locks) (status FINAL))
	=>
	(do-for-all-facts ((?pa plan-action)) (eq ?pa:plan-id EXPLOREZONE)
		(retract ?pa)
	)
	(retract ?p)
)
	
(defrule action-selection-exploration-failed
	?p <- (plan (id EXPLORATION-PLAN) (goal-id EXPLORATION))
	?pa <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (id ?id)
			(action-name move-node) (param-values ?r ?node) (status FAILED))
	(Position3DInterface (id "Pose") (translation $?r-pose))
	(navgraph-node (name ?node) (pos $?node-pos))
	(not (plan (id EXPLORE-ZONE) (goal-id EXPLORATION)))
	?panext <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (id ?oid&: (> ?oid ?id)) (status FORMULATED))
	(not (plan-action (plan-id EXPLORATION-PLAN) (id ?hid&: (and (< ?hid ?oid) (> ?hid ?id))) (status FORMULATED)))
	=>
	(if (< (distance-mf ?node-pos ?r-pose) 1.5) then
		(modify ?panext (status PENDING))
		else
		(modify ?pa (status FORMULATED))
	)
)

(defrule action-selection-select
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
                      (id ?id) (status FORMULATED)
                      (action-name ?action-name)
                      (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
  (printout t "Selected next action " ?action-name ?param-values crlf)
	(modify ?pa (status PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)
