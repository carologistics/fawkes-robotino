(defrule slide-in-use-positive
" If a slide put action is running, the slide is in use.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name mps-type) (param-values ?mps RS))
  (plan-action (action-name wp-put-slide-cc) (param-values $? ?mps $?)
    (state ?state&:(not (member$ ?state (create$ FORMULATED PENDING FINAL FAILED)))
  ))
  (not (domain-fact (name slide-in-use) (param-values ?mps)))
 =>
  (assert (domain-fact (name slide-in-use) (param-values ?mps)))
)

(defrule slide-in-use-negative
" If no slide put action is running, the slide is not used anymore
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name slide-in-use) (param-values ?mps))
   (not (plan-action (action-name wp-put-slide-cc) (param-values $? ?mps $?)
     (state ?state&:(not (member$ ?state (create$ FORMULATED PENDING FINAL FAILED))))
   ))
 =>
  (retract ?f)
)
