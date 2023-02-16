;---------------------------------------------------------------------------

; Sub-type: CENTRAL-RUN-LINEAR
; Perform: one goal at a time, ordered by goal priority
; Succeed: if all sub-goals succeed
; Fail:    if any sub-goal fails
;
; A CENTRAL-RUN-LINEAR parent goal will order the executable goals by priority and
; then start performing them in order.

(defrule central-run-linear-expand ; 
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode SELECTED))
	=>
	(modify ?gf (mode EXPANDED))
)

(defrule central-run-linear-commit ; 
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode EXPANDED))
	=>
	(modify ?gf (mode COMMITTED))
)

(defrule central-run-linear-dispatch ; 
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode COMMITED))
	=>
	(modify ?gf (mode DISPATCHED))
)

(defrule central-run-linear-subgoals-select ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?priority) (is-executable TRUE))
	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (mode ?mode&~EVALUATED)
	           (priority ?priority2&:(< ?priority2 ?priority)) (is-executable TRUE))) ; The goal with priority ?priority is the lowest-priority non-evaluated goal that is executable
    
    =>

    (modify ?sg (mode SELECTED))
)

(defrule central-run-linear-subgoals-evaluated ; 
    ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode DISPATCHED))
    (not (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode ?mode&~EVALUATED)))

    =>

    (modify ?gf (mode EVALUATED) (outcome COMPLETED))
)

(defrule central-run-linear-subgoals-retract ; Retract subgoals after the parent goal is evaluated
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode EVALUATED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))

    =>

    (modify ?sg (mode RETRACTED))
)