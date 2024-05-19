(defrule rs-input-blocked-positive
" Block the input of a RS for a workpiece that reseves it.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name rs-input-reserved-for-wp) (param-values ?mps ?wp))
  (not (domain-fact (name rs-input-blocked) (param-values ?mps ?wp)))
 =>
  (assert (domain-fact (name rs-input-blocked) (param-values ?mps)))
)

(defrule rs-input-blocked-negative
" Unblock the input of a RS if the reservation is gone.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name rs-input-blocked) (param-values ?mps))
  (not (domain-fact (name rs-input-reserved-for-wp) (param-values ?mps ?)))
 =>
  (retract ?f)
)
