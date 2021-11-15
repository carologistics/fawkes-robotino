(defrule goal-expander-create-sequence
	?g <- (goal (mode SELECTED) (id TESTGOAL)) 
	=>
	(assert
	  (plan (id TESTGOAL-PLAN) (goal-id TESTGOAL)
	        (type SEQUENTIAL))
	)

	(bind ?i 1)
	(bind ?rnum 0)
	;(bind ?r "/robot1/Skiller")
	(do-for-all-facts ((?m domain-fact)) (eq ?m:name mps-type) 
		(bind ?rnum (+ (mod ?rnum 3) 1))
		(bind ?r (str-cat (str-cat "/robot" ?rnum) "/Skiller"))
		(printout t ?r crlf)
		(assert (plan-action (id ?i) (plan-id TESTGOAL-PLAN) (goal-id TESTGOAL) 
							(action-name visit) (skiller ?r)
						(param-values (nth$ 1 ?m:param-values) OUTPUT CYAN)))
		(bind ?i (+ ?i 1))
	)
	(modify ?g (mode EXPANDED))
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
