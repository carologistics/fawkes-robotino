(deffunction parent-actions-finished (?goal-id ?plan-id $?actions-ids)
	"check if all actions in the list of ids are finished successfully"
	(foreach ?a ?actions-ids
		(bind ?result
			(do-for-fact ((?wm-fact wm-fact))
				(and
					(wm-key-prefix ?wm-fact:key (create$ plan-action ?goal-id ?plan-id ?a status))
					(eq ?wm-fact:value FINAL)
				)
				(printout t "Dependency of action with id " ?a " was detected" crlf) ; TODO Why is this printout crucial for the correct execution?
			)
		)
		(if (not ?result) then
			(return FALSE)
		)
	)
	(return TRUE)
)

(defrule action-update-status
	"update information abount plan-actions status to wm-fact base"
	(plan-action (id ?id) (plan-id ?plan-id) (status ?status-new))
	?wmf <- (wm-fact (key plan-action ?goal-id ?plan-id ?sym-id&:(eq ?sym-id ?id) status) (value ?status-old&:(not (eq ?status-old ?status-new))))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	=>
	(printout t "Update status for action with id " ?sym-id " [" ?status-old " -> " ?status-new "]"  crlf)
	(modify ?wmf (value ?status-new))
)

(defrule action-selection-select-parallel
	"select earliest action if no other is chosen and if all actions indicated by parents-ids are finished"
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
					  (action-name ?action-name)
					  (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	(wm-fact (key plan-action ?goal-id ?plan-id ?sym-id&:(eq ?sym-id ?id) dep) (values $?parents-ids))
	(wm-fact (key plan-action ?goal-id ?plan-id ?sym-id-o status))
	(not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))
	(test (parent-actions-finished ?goal-id ?plan-id ?parents-ids))
	=>
	(printout t "Select next action " ?action-name ?param-values " with id " ?sym-id " and parents-ids " ?parents-ids  crlf)
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

(defrule action-selection-continue
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (plan-id ?plan-id) (status ~FINAL)))
	(not (plan-action (action-name fulfill-order-c0|fulfill-order-c1|fulfill-order-c2|fulfill-order-c3)))
	=>
	(do-for-all-facts ((?wm-fact wm-fact)) (wm-key-prefix ?wm-fact:key (create$ plan-action ?goal-id ?plan-id))
		(retract ?wm-fact)
	)
	(do-for-all-facts ((?pa plan-action)) (eq ?pa:plan-id ?plan-id)
		(retract ?pa)
	)
	(modify ?g (mode SELECTED))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (plan-id ?plan-id) (status ~FINAL)))
	(plan-action (action-name fulfill-order-c0|fulfill-order-c1|fulfill-order-c2|fulfill-order-c3))
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
