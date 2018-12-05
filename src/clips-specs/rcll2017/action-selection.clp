(defrule action-selection-exploration-select
	(goal (id EXPLORATION) (mode DISPATCHED))
	?pa <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION)
			(id ?id) (status FORMULATED)
			(action-name ?action-name)
			(param-values $?param-values))
	(goal (id EXPLORATION) (mode DISPATCHED))
	(not (plan-action (goal-id EXPLORATION) (status FAILED|PENDING|WAITING|RUNNING)))
	(not (plan (id EXPLORE-ZONE) (goal-id EXPLORATION)))
	(not (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (status FORMULATED) (id ?oid&: (< ?oid ?id))))
	=>
	(printout t "Selected next Exploration move action" ?action-name ?param-values crlf)
	(modify ?pa (status PENDING))
)

(defrule action-selection-exploration-done
	(not (plan-action (goal-id EXPLORATION) (plan-id EXPLORATION-PLAN) (status ?status&~FINAL&~FAILED)))
	?g <- (goal (id EXPLORATION) (mode DISPATCHED))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-seleection-explorezone-release-locks
	(goal (id EXPLORATION) (mode DISPATCHED))
	?p <- (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
	(not (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (status ?s&~FINAL&~FORMULATED&~FAILED)))
	?rl <- (plan-action (plan-id EXPLORE-ZONE) (action-name release-locks) (status FORMULATED))
	=>
	(modify ?rl (status PENDING))
)


(defrule action-selection-explorezone-done
	?p <- (plan (id EXPLORE-ZONE) (goal-id EXPLORATION))
	(goal (id EXPLORATION) (mode DISPATCHED))
	(or (not (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (status ?s&~FINAL)))
					 (plan-action (id ?id) (plan-id EXPLORE-ZONE) (goal-id EXPLORATION) (status ?s&FAILED))
	)
	(or (plan-action (plan-id EXPLORE-ZONE) (action-name release-locks) (status FINAL))
		  (not (plan-action (plan-id EXPLORE-ZONE) (action-name release-locks))))
	=>
	(do-for-all-facts ((?pa plan-action)) (eq ?pa:plan-id EXPLORE-ZONE)
		(retract ?pa)
	)
	(retract ?p)
)

(defrule action-selection-exploration-failed
	?p <- (plan (id EXPLORATION-PLAN) (goal-id EXPLORATION))
	(goal (id EXPLORATION) (mode DISPATCHED))
	?pa <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (id ?id)
			(action-name move-node) (param-values ?r ?node) (status FAILED))
	(Position3DInterface (id "Pose") (translation $?r-pose))
	(navgraph-node (name ?node) (pos $?node-pos))
	(not (plan (id EXPLORE-ZONE) (goal-id EXPLORATION)))
	?panext <- (plan-action (plan-id EXPLORATION-PLAN) (goal-id EXPLORATION) (id ?oid&: (> ?oid ?id)) (status FORMULATED))
	(not (plan-action (plan-id EXPLORATION-PLAN) (id ?hid&: (and (< ?hid ?oid) (> ?hid ?id))) (status FORMULATED)))
	=>
	(if (< (distance-mf ?node-pos ?r-pose) 1.5) then
		(printout t "EXP Go to next node" crlf)
		(modify ?panext (status PENDING))
		(modify ?pa (status FINAL))
		else
		(printout t "EXP Retry node" crlf)
		(modify ?pa (status FORMULATED))
	)
)

(deffunction parent-actions-finished (?goal-id ?plan-id $?actions-ids)
	"check if all actions in the list of ids are finished successfully"
	(foreach ?a ?actions-ids
		(printout t "parent-actions-finished check for planfinal " ?a crlf)
		(bind ?result
			(do-for-fact ((?wm-fact wm-fact))
				(and
					(wm-key-prefix ?wm-fact:key (create$ plan-action ?goal-id ?plan-id ?a status))
					(eq ?wm-fact:value FINAL)
				)
			)
		)
		(if (not ?result) then
			(printout t "parent-actions-finished FALSE" crlf)
			(return FALSE)
		)
	)

	(printout t "parent-actions-finished TRUE" crlf)
	(return TRUE)
)

(defrule action-update-status
	"update information abount plan-actions status to wm-fact base"
	(plan-action (id ?id) (plan-id ?plan-id) (status ?status-new))
	?wmf <- (wm-fact (key plan-action ?goal-id ?plan-id ?sym-id&:(eq ?sym-id ?id) status) (value ?status-old&:(not (eq ?status-old ?status-new))))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	=>
	(modify ?wmf (value ?status-new))
)


(defrule action-selection-select-parallel
	"select earliest action if no other is chosen and if all actions indicated by parents-ids are finished"
		?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
                      (id ?id) (status FORMULATED)
                      (action-name ?action-name)
                      (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(wm-fact (key plan-action ?goal-id ?plan-id ?sym-id&:(eq ?sym-id ?id) dep)); (values ?parents-ids))
	(not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))
	; (test (parent-actions-finished ?goal-id ?plan-id ?parents-ids))
	=>
	(printout t "Selected next action " ?action-name ?param-values " with sym-id " ?sym-id " and id " ?id crlf)
	(modify ?pa (status PENDING))
)

; (defrule action-selection-select
;     "select earliest action if no other is chosen"
;     ?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
;                       (action-name ?action-name)
;                       (param-values $?param-values))
;     (plan (id ?plan-id) (goal-id ?goal-id))
;     (goal (id ?goal-id) (mode DISPATCHED))
;     (not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
;     (not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))
;     =>
;   (printout t "Selected next action " ?action-name ?param-values crlf)
;     (modify ?pa (status PENDING))
; )

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id&~EXPLORATION) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id& :(neq ?goal-id EXPLORATION)) (mode DISPATCHED))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (status FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)
