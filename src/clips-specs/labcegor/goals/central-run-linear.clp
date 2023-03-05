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
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode COMMITTED))
	=>
	(modify ?gf (mode DISPATCHED))
)

;Here also machines will be blocked
(defrule central-run-linear-subgoals-select ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (class ?subgoal-class) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?priority) (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order-id ?root-id))
    (goal (goal-id ?root-id) (class ?root-class))
	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (outcome ?outcome&~COMPLETED)
	           (priority ?priority2&:(< ?priority2 ?priority)) (is-executable TRUE))) ; The goal with priority ?priority is the lowest-priority non-evaluated goal that is executable
    
    ;Facts for delivery station (if needed to block):
    (domain-object (name ?team-color) (type team-color))
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-type) (param-values ?ds DS))

    =>
    (if (and (eq ?subgoal-class GOAL-DELIVER-C0) (eq ?root-class GOAL-ORDER-C0))
        then
        (assert (machine-used (mps ?ds) (order-id ?ord)))
    )
    (modify ?sg (mode SELECTED))
)

(defrule central-run-linear-subgoals-evaluated ; 
    ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode DISPATCHED))
    (not (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (outcome ?outcome&~COMPLETED)))

    =>

    (modify ?gf (mode EVALUATED) (outcome COMPLETED))
)

(defrule central-run-linear-subgoals-retract ; Retract subgoals after the parent goal is evaluated
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode EVALUATED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))

    =>

    (modify ?sg (mode RETRACTED))
)