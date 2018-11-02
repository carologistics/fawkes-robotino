(defrule action-selection-exploration-select
	(goal (id ?parent-id) (class EXPLORATION) (mode DISPATCHED))
	(goal (id ?goal-id) (parent ?parent-id) (mode DISPATCHED))
	?pa <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id ?goal-id)
			(id ?id) (state FORMULATED)
			(action-name ?action-name)
			(param-values $?param-values))
	(not (plan-action (goal-id ?goal-id) (state FAILED|PENDING|WAITING|RUNNING)))
	(not (plan (id EXPLORE-ZONE) (goal-id ?goal-id)))
	(not (plan-action (plan-id EXPLORATION-PLAN) (goal-id ?goal-id) (state FORMULATED) (id ?oid&: (< ?oid ?id))))
	=>
	(printout t "Selected next Exploration move action" ?action-name ?param-values crlf)
	(modify ?pa (state PENDING))
)

(defrule action-selection-exploration-done
	?g <- (goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED))
	(not (plan-action (goal-id ?goal-id) (plan-id EXPLORATION-PLAN)
                    (state ?status&~FINAL&~FAILED)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-seleection-explorezone-release-locks
	(goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED))
	?p <- (plan (id EXPLORE-ZONE) (goal-id ?goal-id))
	(not (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id ?goal-id)
                    (state ?s&~FINAL&~FORMULATED&~FAILED)))
	?rl <- (plan-action (plan-id EXPLORE-ZONE) (action-name release-locks)
                      (state FORMULATED))
	=>
	(modify ?rl (state PENDING))
)


(defrule action-selection-explorezone-done
	(goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED))
	?p <- (plan (id EXPLORE-ZONE) (goal-id ?goal-id))
	(or (not (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id ?goal-id)
                        (state ?s&~FINAL)))
      (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id ?goal-id)
                   (state ?s&FAILED))
	)
	(or (plan-action (goal-id ?goal-id) (plan-id EXPLORE-ZONE)
                   (action-name release-locks) (state FINAL))
		  (not (plan-action (goal-id ?goal-id) (plan-id EXPLORE-ZONE)
                        (action-name release-locks))))
	=>
	(do-for-all-facts ((?pa plan-action))
                    (and (eq ?pa:goal-id ?goal-id)
                         (eq ?pa:plan-id EXPLORE-ZONE))
                    (retract ?pa)
	)
	(retract ?p)
)

(defrule action-selection-exploration-failed
	(goal (id ?goal-id) (class EXPLORATION) (mode DISPATCHED))
	?p <- (plan (id EXPLORATION-PLAN) (goal-id ?goal-id))
	?pa <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id ?goal-id) (id ?id)
			(action-name move-node) (param-values ?r ?node) (state FAILED))
        (not (plan-action (plan-id EXPLORATION-PLAN) (goal-id ?goal-id) (id ?id2&:(> ?id2 ?id)) 
			(action-name move-node) (param-values ?r ?node2) (state FAILED)))
	(Position3DInterface (id "Pose") (translation $?r-pose))
	(navgraph-node (name ?node) (pos $?node-pos))
	(not (plan (id EXPLORE-ZONE) (goal-id ?goal-id)))
	?panext <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id ?goal-id)
                          (id ?oid&: (> ?oid ?id)) (state FORMULATED))
	(not (plan-action (goal-id ?goal-id) (plan-id EXPLORATION-PLAN)
                    (id ?hid&: (and (< ?hid ?oid) (> ?hid ?id)))
                    (state FORMULATED)))
	=>
	(if (< (distance-mf ?node-pos ?r-pose) 1.5) then
		(printout t "EXP Go to next node" crlf)
		(modify ?panext (state PENDING))
		(modify ?pa (state FINAL))
		else
		(printout t "EXP Retry node" crlf)
		(modify ?pa (state FORMULATED))
	)
)

(defrule action-selection-select
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
                      (id ?id) (state FORMULATED)
                      (action-name ?action-name)
                      (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (class ?class) (mode DISPATCHED))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
  (if (neq ?class BEACONACHIEVE) then
    (printout t "Selected next action " ?action-name ?param-values crlf)
  )
	(modify ?pa (state PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id&~EXPLORATION) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (class ?class& : (neq ?class EXPLORATION)) (mode DISPATCHED))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)
