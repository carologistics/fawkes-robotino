(deffunction parent-actions-finished ($?actions-ids)
  "check if all actions in the list of ids are finished successfully"
  (foreach ?a ?actions-ids
    (bind ?result
		(do-for-fact ((?plan-action plan-action)) (and (eq ?plan-action:status FINAL) (eq ?plan-action:id ?a))))
    (if (not ?result) then
      (return FALSE)
      )
  )
  (return TRUE)
)

; (defrule action-selection-select-parallel
; 	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status FORMULATED)
; 					  (parents-ids $?parents-ids)
;                       (action-name ?action-name)
;                       (param-names $?param-names)
;                       (param-values $?param-values))
; 	(plan (id ?plan-id) (goal-id ?goal-id))
; 	(goal (id ?goal-id) (mode DISPATCHED))
; 	(bind ?actor (plan-action-arg r ?param-names ?param-values ""))
; 	(bind ?team-color (plan-action-arg team-color ?param-names ?param-values ""))
; 	(not (plan-action (plan-id ?plan-id) (status PENDING|WAITING|RUNNING|FAILED)))
; 	(not (plan-action (plan-id ?plan-id) (status FORMULATED) (id ?oid&:(< ?oid ?id)) (param-values ?actor ?team-color)))
; 	(test (parent-actions-finished ?parents-ids))
; 	=>
;   (printout t "Selected next action " ?action-name ?param-values crlf)
; 	(modify ?pa (status PENDING))
; )

(defrule action-selection-select
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
	(modify ?g (mode COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (mode DISPATCHED))
	(plan-action (status FAILED))
	=>
	(modify ?g (mode FAILED))
)
