;React to broken mps
(defrule broken-mps-reject-goals
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?g <- (goal (id ?goal-id) (mode FORMULATED|SELECTED|EXPANDED) (params $? ?mps $?))
  (plan (id ?plan-id) (goal-id ?goal-id))
  =>
  (modify ?g (mode REJECTED))
)

(defrule broken-mps-exit-goal
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?pa <- (plan-action (id ?id) (action-name ?action-name)
	(plan-id ?plan-id) (goal-id ?goal-id)
	(status ?status&~FINAL)
	(param-values $? ?mps $?))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (mode DISPATCHED))
  =>
  (modify ?pa (status FAILED))
)



;Clean up facts
(defrule broken-mps-remove-wp-facts
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?wa <- (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?side))
  ?wm <- (wm-fact (key domain fact ?fact& : (neq ?fact wp-at) args? $? wp ?wp $?))
   =>
  (retract ?wm)
)

(defrule broken-mps-remove-wp-at
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?wa <- (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?side))
  (not (wm-fact (key domain fact ?fact& : (neq ?fact wp-at) args $? wp ?wp $?)))
  =>
  (retract ?wa)
)

(defrule broken-mps-remove-bs-prepared-color
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?msp s BROKEN))
  ?wa <- (wm-fact (key domain fact bs-prepared-color args? m ?mps col ?col))
  =>
  (retract ?wa)
)

(defrule broken-mps-remove-bs-prepared-side
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?wa <- (wm-fact (key domain fact bs-prepared-side args? m ?mps side ?side))
  =>
  (retract ?wa)
)

(defrule broken-mps-reset-cs-can-perform
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?wa <- (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ?cb <- (wm-fact (key domain fact cs-buffered args? m ?mps col ?col))
   =>
  (modify ?wa (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  (retract ?cb)
)

(defrule broken-mps-remove-cs-prepared-for
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?wa <- (wm-fact (key domain fact cs-prepare-for args? m ?mps op ?op))
  =>
  (retract ?wa)
)

(defrule broken-mps-remove-rs-prepared-color
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?wa <- (wm-fact (key domain fact rs-prepared-color args? m ?mps col ?col))
  =>
  (retract ?wa)
)

(defrule broken-mps-reset-rs-filled-with
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?wa <- (wm-fact (key domain fact rs-filled-with args? m ?mps n ?ring-num))
  =>
  (modify ?wa (key domaini fact rs-filled-with args? m ?mps n ZERO))
)

(defrule broken-mps-remove-ds-prepared-gate
  (declare (salience 1))
  (wm-fact (key domain fact mps-state args? m ?mps s BROKEN))
  ?wa <- (wm-fact (key domain fact ds-prepared-gate args? m ?mps g ?gate))
  =>
  (retract ?wa)
)

