(defrule next-machine-positive
" Mirror result of wp meta fact."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (wm-fact (key wp meta next-machine args? wp ?wp) (value ?step))
  (not (domain-fact (name next-machine) (param-values ?wp $?)))
 =>
  (assert (domain-fact (name next-machine) (param-values ?wp ?step)))
)

(defrule next-machine-negative
" Mirror result of wp meta fact."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <-(domain-fact (name next-machine) (param-values ?wp ?machine))
  (not (wm-fact (key wp meta next-machine args? wp ?wp) (value ?machine)))
 =>
  (retract ?f)
)
