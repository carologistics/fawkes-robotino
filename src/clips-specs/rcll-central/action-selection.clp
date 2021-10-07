(defrule action-selection-select
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
                      (id ?id) (state FORMULATED)
                      (action-name ?action-name&:(neq ?action-name move))
                      (param-values $?param-values))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (class ?class) (mode DISPATCHED) (verbosity ?verbosity))
	(or (wm-fact (key game state) (value RUNNING))
	    (test (eq ?action-name send-beacon))
	)

  (not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	=>
  (if (neq ?verbosity QUIET) then
    (printout t "Selected next action " ?action-name ?param-values crlf)
  )
	(modify ?pa (state PENDING))
)

(defrule action-selection-done
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (class ~EXPLORATION) (mode DISPATCHED) (type ACHIEVE))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state ~FINAL)))
	=>
	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule action-selection-failed
	(plan (id ?plan-id) (goal-id ?goal-id))
	?g <- (goal (id ?goal-id) (class ?class& : (neq ?class EXPLORATION)) (mode DISPATCHED))
	(plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED))
	=>
	(modify ?g (mode FINISHED) (outcome FAILED))
)

(defrule action-selection-select-move
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id)
	                    (id ?id) (state FORMULATED)
	                    (action-name ?action-name&move)
	                    (param-values ?r ?from ?from-side ?to ?to-side))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(goal (id ?goal-id) (class ?class) (mode DISPATCHED) (verbosity ?verbosity))

	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state PENDING|WAITING|RUNNING|FAILED)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (state FORMULATED) (id ?oid&:(< ?oid ?id))))
	(not (plan-action (state PENDING|WAITING|RUNNING)
	                  (param-values ? ? ? ?to ?to-side)))
	=>
	(modify ?pa (state PENDING))
)
