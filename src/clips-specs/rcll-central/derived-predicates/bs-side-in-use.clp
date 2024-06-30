(defrule bs-side-in-use
"If a BS is part of a goal's operation, assert a fact to indicate this state."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
	(not (domain-fact (name bs-side-in-use) (param-values bs $?)))
	(wm-fact (key domain fact mps-type args? $? ?bs $? BS $?))
	(goal (id ?goal-id) (mode EXPANDED|COMMITTED|DISPATCHED) (sub-type SIMPLE))
	(plan-action (action-name wp-get) (goal-id ?goal-id) (param-values $? ?bs ?bs-side $?)
               (state FORMULATED|PENDING|WAITING|RUNNING))
	=>
	(assert (domain-fact (name bs-side-in-use) (param-values ?bs ?bs-side)))
)

(defrule bs-side-in-use-negative
"Retract BS in use fact if it is no longer in use."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
	?wm <- (domain-fact (name bs-side-in-use) (param-values ?bs ?bs-side))
	(wm-fact (key domain fact mps-type args? $? ?bs $? BS $?))
	(not (and (goal (id ?goal-id) (mode EXPANDED|COMMITTED|DISPATCHED))
	          (plan-action (action-name wp-get) (goal-id ?goal-id) (param-values $? ?bs ?bs-side $?) (state FORMULATED|PENDING|WAITING|RUNNING))
	))
	=>
	(retract ?wm)
)
