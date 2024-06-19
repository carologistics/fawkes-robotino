(defrule transport-blocked-positive
" If a running transport goal targets a machine, no other transport goal
  should target the same machine.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (goal (class BUFFER-CAP|MOUNT-CAP|MOUNT-RING)
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? target-mps ?mps $?)
  )
  (not (domain-fact (name transport-blocked) (param-values ?mps)))
 =>
  (assert (domain-fact (name transport-blocked) (param-values ?mps)))
)

(defrule transport-blocked-negative
" If no transport goal is running, release the block.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name transport-blocked) (param-values ?mps))
  (not (goal (class BUFFER-CAP|MOUNT-CAP|MOUNT-RING)
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? target-mps ?mps $?)
  ))
 =>
  (retract ?f)
)
