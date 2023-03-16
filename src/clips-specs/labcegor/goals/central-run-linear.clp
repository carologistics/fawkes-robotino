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

; Selection rule when no machine blockings are considered
(defrule central-run-linear-subgoals-select ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (class ?subgoal-class) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?priority) (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class ?root-class))
	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (outcome ?outcome&~COMPLETED)
	           (priority ?priority2&:(< ?priority2 ?priority)) (is-executable TRUE))) ; The goal with priority ?priority is the lowest-priority non-evaluated goal that is executable

    (test(not (and (eq ?subgoal-class GOAL-DELIVER-C0) (eq ?root-class GOAL-ORDER-C0))))
    (test(not (and (eq ?subgoal-class GOAL-DELIVER-C1) (eq ?root-class GOAL-ORDER-C1))))
    (test(not (and (eq ?subgoal-class GOAL-DELIVER-C2) (eq ?root-class GOAL-ORDER-C2))))
    =>
    (modify ?sg (mode SELECTED))
)
; Selection rule for selection of deliver-c0/c1/c2 for c0/c1/c2 orders.
(defrule central-run-linear-subgoals-select-deliver-c0
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-LINEAR) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (class GOAL-DELIVER-C0|GOAL-DELIVER-C1|GOAL-DELIVER-C2) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	      (priority ?priority) (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2))
	(not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (outcome ?outcome&~COMPLETED)
	           (priority ?priority2&:(< ?priority2 ?priority)) (is-executable TRUE))) ; The goal with priority ?priority is the lowest-priority non-evaluated goal that is executable
    
    ;Facts for delivery station:
    (domain-object (name ?team-color) (type team-color))
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-type) (param-values ?ds DS))
    ; Block Check for delivery station
    (not (machine-used (mps ?ds) (order-id ?some-order-id)))
    =>
    (assert (machine-used (mps ?ds) (order-id ?ord)))
    (printout t ?ds " is now in use for order " ?ord crlf)
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