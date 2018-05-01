(deffunction parent-actions-finished ($?actions-ids)
  "check if all actions in the list of ids are finished successfully"
  (foreach ?a ?actions-ids
    (bind ?result
		(do-for-fact ((?wm-fact wm-fact))
			(and
				(wm-key-prefix ?wm-fact:key (create$ planfinal))
				(eq ?wm-fact:id ?a)
			)
		)
	)
    (if (not ?result) then
      (return FALSE)
      )
  )
  (return TRUE)
)

(defrule action-completed
	"add information abount final plan-actions to wm-fact base"
	(plan-action (id ?id) (plan-id ?plan-id) (status FINAL))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (mode DISPATCHED))
	=>
	(assert
		(wm-fact
			(key planfinal ?plan-id ?goal-id args? id ?id)
		)
	)
)
; (defrule action-selection-select-parallel
;     "select earliest action if no other is chosen and if all actions indicated by parents-ids are finished"
;     ?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
;                       (action-name ?action-name)
;                       (param-names $?param-names)
;                       (param-values $?param-values))
;     (plan (id ?plan-id) (goal-id ?goal-id))
;     (goal (id ?goal-id) (mode DISPATCHED))
;     (bind ?sym-id ?id)
;     (wm-fact (key plandep ?plan-id ?goal-id ?sym-id) (values ?parents-ids))
;     (not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
;     (not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id))))
;     (test (parent-actions-finished ?parents-ids))
;     =>
;   (printout t "Selected next action " ?action-name ?param-values crlf)
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
