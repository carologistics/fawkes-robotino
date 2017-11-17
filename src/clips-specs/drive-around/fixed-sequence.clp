
(defrule goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id VISITALL))
	=>
	(assert
	 (plan (id VISITALL-PLAN) (goal-id VISITALL))
	 (plan-action (id 1) (plan-id VISITALL-PLAN) (duration 4.0)
								(action-name goto)
								(param-names to) (param-values "center-field"))
	 (plan-action (id 2) (plan-id VISITALL-PLAN) (duration 4.0)
								(action-name goto)
								(param-names to) (param-values "source-red-output"))
	 (plan-action (id 3) (plan-id VISITALL-PLAN) (duration 4.0)
								(action-name goto)
								(param-names to) (param-values "source-black-output"))
	 (plan-action (id 4) (plan-id VISITALL-PLAN) (duration 4.0)
								(action-name goto)
								(param-names to) (param-values "target-magenta-input"))
	 (plan-action (id 5) (plan-id VISITALL-PLAN) (duration 4.0)
								(action-name goto)
								(param-names to) (param-values "target-cyan-input"))
	 )
	(modify ?g (mode EXPANDED))
)
