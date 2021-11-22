(defrule goal-expander-create-sequence
	?g1 <- (goal (mode SELECTED) (id TESTGOAL1))
	?g2 <- (goal (mode SELECTED) (id TESTGOAL2))
	?g3 <- (goal (mode SELECTED) (id TESTGOAL3))
	=>
	(assert
	  (plan (id TESTGOAL-PLAN1) (goal-id TESTGOAL1)
	        (type SEQUENTIAL))
	  (plan (id TESTGOAL-PLAN2) (goal-id TESTGOAL2)
	        (type SEQUENTIAL))
	  (plan (id TESTGOAL-PLAN3) (goal-id TESTGOAL3)
	)

	(bind ?i 1)
	(bind ?rnum 0)
	;(bind ?r "/robot1/Skiller")
	(do-for-all-facts ((?m domain-fact)) (eq ?m:name mps-type) 
		(bind ?rnum (+ (mod ?rnum 3) 1))
		(bind ?r (str-cat (str-cat "/robot" ?rnum) "/Skiller"))
		(printout t ?r crlf)
		(assert (plan-action (id ?i) (plan-id (sym-cat TESTGOAL-PLAN ?rnum)) (goal-id (sym-cat TESTGOAL ?rnum))

							(action-name visit) (skiller ?r)
						(param-values (nth$ 1 ?m:param-values) OUTPUT CYAN)))
		(bind ?i (+ ?i 1))
	)
	(modify ?g1 (mode EXPANDED))
	(modify ?g2 (mode EXPANDED))
	(modify ?g3 (mode EXPANDED))
)

(defrule goal-expander-maintain-beacon
	?p <- (goal (mode SELECTED) (id ?parent-id))
	?g <- (goal (id ?goal-id) (class SEND-BEACON) (mode FORMULATED)
	            (parent ?parent-id))
=>
	(modify ?p (mode COMMITTED) (committed-to ?goal-id))
)

(defrule goal-expander-send-beacon-signal
	?p <- (goal (mode DISPATCHED) (id ?parent-id))
	?g <- (goal (id ?goal-id) (class SEND-BEACON) (mode SELECTED)
	            (parent ?parent-id))
=>
	(assert
	 (plan (id BEACON-PLAN) (goal-id ?goal-id)
	        (type SEQUENTIAL))
	        (plan-action (id 1) (plan-id BEACON-PLAN) (goal-id ?goal-id)
	                     (action-name send-beacon))
	)
	(modify ?g (mode EXPANDED))
)
