(defrule next-step-positive
" Mirror result of wp meta fact."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (wm-fact (key wp meta next-step args? wp ?wp) (value ?step))
  (not (domain-fact (name next-step) (param-values ?wp $?)))
 =>
  (assert (domain-fact (name next-step) (param-values ?wp ?step)))
)

(defrule next-step-negative
" Mirror result of wp meta fact."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <-(domain-fact (name next-step) (param-values ?wp ?step))
  (not (wm-fact (key wp meta next-step args? wp ?wp) (value ?step)))
 =>
  (retract ?f)
)
