(defrule wait-for-wp-positive
" True if the corresponding action is running."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (plan-action (action-name wait-for-wp) (param-values ?robot ?mps ?side ?wp)
    (state PENDING|RUNNING))
  (not (domain-fact (name wait-for-wp) (param-values ?wp ?mps ?side)))
 =>
  (assert (domain-fact (name wait-for-wp) (param-values ?wp ?mps ?side)))
)

(defrule wait-for-wp-negative
" False if the corresponding action is not running."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name wait-for-wp) (param-values ?wp ?mps ?side))
  (not (plan-action (action-name wait-for-wp) (param-values ? ?mps ?side ?wp)
    (state PENDING|RUNNING)
  ))
 =>
  (retract ?f)
)
