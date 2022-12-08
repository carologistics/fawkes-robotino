
(deffunction distance (?x ?y ?x2 ?y2)
  "Returns the distance of two points in the x,y-plane.
  @param ?x ?y coordinates of one point
  @param ?x2 ?y2 coordinates of the other point

  @return euclidean distance of the two points
  "
  (return (float (sqrt (float(+ (* (- ?x ?x2) (- ?x ?x2)) (* (- ?y ?y2) (- ?y ?y2)))))))
)

(defrule goal-expander-create-sequence1
	?g <- (goal (mode SELECTED) (id ?machine))
	(domain-fact (name robot-waiting) (param-values ?r1))
	(domain-fact (name robot-waiting) (param-values ?r2))
	(test (neq ?r1 ?r2))
	(wm-fact (id ?) (key domain fact at args? r ?r1 m ?loc1 $?) $?)
	(wm-fact (id ?) (key domain fact at args? r ?r2 m ?loc2 $?) $?)
	(navgraph-node (name (str-cat ?loc1)) (pos ?x1 ?y1) $?)
	(navgraph-node (name (str-cat ?loc2)) (pos ?x2 ?y2) $?)
	(navgraph-node (name (str-cat ?machine)) (pos ?x3 ?y3) $?)
	(> (distance ?x2 ?y2 ?x3 ?y3) (distance ?x1 ?y1 ?x3 ?y3))
 	=>
	(assert (plan (id (str-cat ?machine "-PLAN")) (goal-id ?machine) (type SEQUENTIAL)))
	(assert (plan-action (id 1) (plan-id (str-cat ?machine "-PLAN")) (goal-id ?machine)          
               			(action-name visit) (skiller (str-cat "/robot" ?r1 "/Skiller")                  
                        	(param-values ?machine OUTPUT CYAN))))

	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-create-sequence2
	?g <- (goal (mode SELECTED) (id ?machine))
	(domain-fact (name robot-waiting) (param-values ?r1))
	not ((domain-fact (name robot-waiting) (param-values ?r2:(neq ?r1 ?r2))))

	=>
	(assert (plan (id (str-cat ?machine "-PLAN")) (goal-id ?machine) (type SEQUENTIAL)))
	(assert (plan-action (id 1) (plan-id (str-cat ?machine "-PLAN")) (goal-id ?machine)          
               			(action-name visit) (skiller (str-cat "/robot" ?r1 "/Skiller")                  
                        	(param-values ?machine OUTPUT CYAN))))
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
