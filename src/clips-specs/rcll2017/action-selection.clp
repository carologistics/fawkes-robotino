(deffunction parent-actions-finished (?goal-id ?plan-id $?actions-ids)
	"check if all actions in the list of ids are finished successfully"
	(foreach ?a ?actions-ids
		(printout t "parent-actions-finished check for planfinal " ?a crlf)
		(bind ?result
			(do-for-fact ((?wm-fact wm-fact))
				(and
					(wm-key-prefix ?wm-fact:key (create$ planfinal ?plan-id ?goal-id))
					(eq ?wm-fact:value ?a)
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

; (defrule action-selection-select-parallel
;     "select earliest action if no other is chosen and if all actions indicated by parents-ids are finished"
;     ?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
;                       (action-name ?action-name)
;                       (param-values $?param-values))
;     (plan (id ?plan-id) (goal-id ?goal-id))
;     (goal (id ?goal-id) (mode DISPATCHED))
;     (wm-fact (key plandep ?goal-id ?plan-id ?sym-id&:(eq ?sym-id ?id)) (values ?parents-ids))
;     (not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
;     (not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))
;     (test (parent-actions-finished ?goal-id ?plan-id ?parents-ids))
;     =>
;     (printout t "Selected next action " ?action-name ?param-values crlf)
;     (modify ?pa (status PENDING))
; )

(defrule action-selection-select
	"select earliest action if no other is chosen"
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
					  (action-name ?action-name)
					  (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
  (printout t "Selected next action " ?action-name ?param-values crlf)
	(modify ?pa (status PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (plan-id ?plan-id) (status ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan-action (status FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)
