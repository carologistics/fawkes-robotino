
(defrule goal-expander-create-sequence1
	?g <- (goal (mode SELECTED) (id ?machine))
	(domain-fact (name robot-waiting) (param-values ?r1))
	(domain-fact (name robot-waiting) (param-values ?r2))
	(test (neq ?r1 ?r2))
	;(navgraph-node (name ?machine) (pos ?x ?y) $?) distance function verwenden!!
	 ; If a goal still exists, then this machine has not been visited yet
 	;[EXISTS robot ?r1: ?machine is the nearest goal for ?r1]
	;[FORALL robot ?r2: ?r1 is nearer(or equal) to ?machine than ?r2]
 	=> ; assign ?r1 to goal ?machine
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-create-sequence2
	?g <- (goal (mode SELECTED) (id ?machine))
	(domain-fact (name robot-waiting) (param-values ?r1))
	not ((domain-fact (name robot-waiting) (param-values ?r2:(neq ?r1 ?r2))))


	 ; If a goal still exists, then this machine has not been visited yet
	; [EXISTS robot ?r1: ?machine is the nearest goal for ?r1]
	; [FORALL robot ?r2 != ?r1: ?r2 is busy]
	=> ; assign ?r1 to goal ?machine
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-create-sequence-robot1
	?g <- (goal (mode SELECTED) (id GOALR1))
	=>
        (assert (plan (id GOALR1-PLAN) (goal-id GOALR1) (type SEQUENTIAL)))
		(assert (plan-action (id 1) (plan-id GOALR1-PLAN) (goal-id GOALR1)          
               			(action-name visit) (skiller "/robot1/Skiller")                  
                        	(param-values C-BS OUTPUT CYAN)))

	; (bind ?id 1)
	; (delayed-do-for-all-facts ((?obj domain-object)) (eq ?obj:type mps) 
	; 	(assert (plan-action (id ?id) (plan-id GOALR1-PLAN) (goal-id GOALR1)          
    ;            			(action-name visit) (skiller "/robot1/Skiller")                  
    ;                     	(param-values ?obj:name OUTPUT CYAN)))
	; 	(bind ?id (+ ?id 1))
	; )
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-create-sequence-robot2
	?g <- (goal (mode SELECTED) (id GOALR2))
	=>
        (assert (plan (id GOALR2-PLAN) (goal-id GOALR2) (type SEQUENTIAL)))

		(assert (plan-action (id 1) (plan-id GOALR2-PLAN) (goal-id GOALR2)          
               			(action-name visit) (skiller "/robot2/Skiller")                  
                        	(param-values C-CS1 OUTPUT CYAN)))

	;(bind ?id 1)
	;(delayed-do-for-all-facts ((?obj domain-object)) (eq ?obj:type mps) 
	;	(assert (plan-action (id ?id) (plan-id GOALR2-PLAN) (goal-id GOALR2)          
    ;           			(action-name visit) (skiller "/robot2/Skiller")                  
    ;                    	(param-values ?obj:name OUTPUT CYAN)))
	;	(bind ?id (+ ?id 1))
	;)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-create-sequence-robot3
	?g <- (goal (mode SELECTED) (id GOALR3))
	=>
        (assert (plan (id GOALR3-PLAN) (goal-id GOALR3) (type SEQUENTIAL)))
		(assert (plan-action (id 1) (plan-id GOALR3-PLAN) (goal-id GOALR3)          
               			(action-name visit) (skiller "/robot3/Skiller")                  
                        	(param-values C-CS2 OUTPUT CYAN)))
	; (bind ?id 1)
	; (delayed-do-for-all-facts ((?obj domain-object)) (eq ?obj:type mps) 
	; 	(assert (plan-action (id ?id) (plan-id GOALR3-PLAN) (goal-id GOALR3)          
    ;            			(action-name visit) (skiller "/robot3/Skiller")                  
    ;                     	(param-values ?obj:name OUTPUT CYAN)))
	; 	(bind ?id (+ ?id 1))
	; )
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
