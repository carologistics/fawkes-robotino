(defrule wp-blocked-positive
" If a running goal uses a wp, no other goal should target the same wp.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
   (goal (class MOUNT-CAP|MOUNT-RING|DELIVER|DISCARD|PAY-FOR-RINGS-WITH-BASE)
     (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
     (params $? wp ?wp $?)
   )
  (not (domain-fact (name wp-blocked) (param-values ?wp)))
 =>
  (assert (domain-fact (name wp-blocked) (param-values ?wp)))
)

(defrule wp-blocked-negative
" If no wp goal is running, release the block.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name wp-blocked) (param-values ?wp))
  (not (goal (class MOUNT-CAP|MOUNT-RING|DELIVER|DISCARD|PAY-FOR-RINGS-WITH-BASE)
    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
    (params $? wp ?wp $?)
  ))
 =>
  (retract ?f)
)
