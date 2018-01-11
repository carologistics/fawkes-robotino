
(defrule goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id TESTGOAL))
	=>
	(assert
	 (plan (id TESTGOAL-PLAN) (goal-id TESTGOAL))
	 (plan-action (id 1) (plan-id TESTGOAL-PLAN) (duration 4.0)
								(action-name visit)
								(param-names to) (param-values "C-BS-O"))
	 )
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-send-beacon-signal
  ?g <- (goal (mode SELECTED) (id BEACONACHIEVE))
=>
  (assert
    (plan (id BEACONPLAN) (goal-id BEACONACHIEVE))
    (plan-action (id 1) (plan-id BEACONPLAN) (duration 0.0)
      (action-name send-beacon)))
  (modify ?g (mode EXPANDED))
)
