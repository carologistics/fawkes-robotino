(defrule rs-input-reserved-for-wp-positive
" Reserve the input of an RS for a workpiece that is at this machines output
  and needs the same machine again, in the next step.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name mps-type) (param-values ?mps RS))
  (domain-fact (name wp-at) (param-values ?wp ?mps OUTPUT))
  (not (domain-fact (name rs-input-reserved-for-wp) (param-values ?mps ?wp)))
  (domain-fact (name next-machine) (param-values ?wp ?mps))
 =>
  (assert (domain-fact (name rs-input-reserved-for-wp) (param-values ?mps ?wp)))
)

(defrule rs-input-reserved-for-wp-negative
" Stop the reservation if the wp does not need the machine anymore,
  either because of a next step or because it is gone.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name rs-input-reserved-for-wp) (param-values ?mps ?wp))
  (not (domain-fact (name next-machine) (param-values ?wp ?mps)))
 =>
  (retract ?f)
)
