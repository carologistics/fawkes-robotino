(defrule mps-found-positive
" A MPS is found when it's zone is known.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name zone-content) (param-values ? ?mps))
  (domain-fact (name mps-type) (param-values ?mps ?))
  (not (domain-fact (name mps-found) (param-values ?mps)))
 =>
  (assert (domain-fact (name mps-found) (param-values ?mps)))
)

(defrule mps-found-negative
" If no zone is known anymore, the MPS is not found either.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name mps-found) (param-values ?mps))
  (not (domain-fact (name zone-content) (param-values ? ?mps)))
 =>
  (retract ?f)
)
