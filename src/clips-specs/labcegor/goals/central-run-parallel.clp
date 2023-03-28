;---------------------------------------------------------------------------

; Sub-type: CENTRAL-RUN-PARALLEL
; Perform: all sub-goals in parallel
; Succeed: if all sub-goals succeed
; Fail:    if at least one sub-goal fails
; Reject:  if any sub-goal is rejected but no sub-goal failed
;
; A RUN-PARALLEL parent goal will run all sub-goals in parallel by
; continuously SELECTING all sub-goals.
; If any goal fails, the parent fails. If any sub-goal is rejected,
; the parent is rejected. If all goals have been completed successfully,
; the parent goal succeeds.

(deftemplate machine-used (slot mps) (slot order-id))

(defrule central-run-parallel-expand ; 
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode SELECTED))
	=>
	(modify ?gf (mode EXPANDED))
)

(defrule central-run-parallel-commit ; 
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode EXPANDED))
	=>
	(modify ?gf (mode COMMITTED))
)

(defrule central-run-parallel-dispatch ; 
	?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode COMMITTED))
	=>
	(modify ?gf (mode DISPATCHED))
)


; Selection rule when no machine blockings are considered
(defrule central-run-parallel-subgoals-select ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (class ?sub-class) (mode FORMULATED)
	      (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class ?root-class))

    (test(not (and (eq ?root-class GOAL-ORDER-C1) (eq ?sub-class GOAL-GET-CS))))
    (test(not (and (eq ?root-class GOAL-ORDER-C2) (eq ?sub-class GOAL-PAY-RING2))))
    (test(not (and (eq ?root-class GOAL-ORDER-C3) (eq ?sub-class GOAL-PAY-RING2))))
    (test(not (and (eq ?root-class GOAL-ORDER-C3) (eq ?sub-class GOAL-PAY-RING3))))
    
    =>
    (printout t "test")
    (modify ?sg (mode SELECTED))
)

; Selection rule for selection of get-cs for c1 orders.
(defrule central-run-parallel-subgoals-select-c1-get-cs
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (class GOAL-GET-CS) (mode FORMULATED)
	      (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class GOAL-ORDER-C1))

    ;Facts for cap station:
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (not (machine-used (mps ?cs) (order-id ?some-order-id2)))
    =>
    (assert (machine-used (mps ?cs) (order-id ?ord)))
    (printout t ?cs " is now in use for order " ?ord crlf)
    (modify ?sg (mode SELECTED))
)


; Selection rule for selection of pay-second-ring for c2 orders in case both ringstations needed are different
(defrule central-run-parallel-subgoals-select-c2-pay-second-ring-different-ringstation ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (class GOAL-PAY-RING2) (mode FORMULATED)
	      (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class GOAL-ORDER-C2))

    ; Facts for ring station:
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name rs-ring-spec) (param-values ?rs1 ?ring1col ?num1))
    (domain-fact (name rs-ring-spec) (param-values ?rs2 ?ring2col ?num2))
    (test(not (eq ?rs1 ?rs2)))
    (not (machine-used (mps ?rs2) (order-id ?some-order-id)))
    ; Facts for cap station
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (not (machine-used (mps ?cs) (order-id ?some-order-id)))
    =>
    (assert (machine-used (mps ?cs) (order-id ?ord)))
    (printout t ?cs " is now in use for order " ?ord crlf)
    (assert (machine-used (mps ?rs2) (order-id ?ord)))
    (printout t ?rs2 " is now in use for order " ?ord crlf)
    (modify ?sg (mode SELECTED))
)

; Selection rule for selection of pay-second-ring for c2 orders in case both ringstations needed are the same
(defrule central-run-parallel-subgoals-select-c2-pay-second-ring-same-ringstation ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (class GOAL-PAY-RING2) (mode FORMULATED)
	      (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class GOAL-ORDER-C2))

    ; First ring must be mounted:
    (goal (id ?to-rs-id) (parent ?id) (class GOAL-TO-RS1) (outcome COMPLETED))

    ; Facts for ring station:
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring1col ?num1))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring2col ?num2))
    ; Facts for cap station
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (not (machine-used (mps ?cs) (order-id ?some-order-id)))
    =>
    (assert (machine-used (mps ?cs) (order-id ?ord)))
    (printout t ?cs " is now in use for order " ?ord crlf)
    (modify ?sg (mode SELECTED))
)

; Selection rule for C3: GOAL-PAY-RING2, if first and second ringstation are different
(defrule central-run-parallel-subgoals-select-c3-pay-second-ring-different-ringstation ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (class GOAL-PAY-RING2) (mode FORMULATED)
	      (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class GOAL-ORDER-C3))

    ; Facts for ring station:
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name rs-ring-spec) (param-values ?rs1 ?ring1col ?num1))
    (domain-fact (name rs-ring-spec) (param-values ?rs2 ?ring2col ?num2))
    (test(not (eq ?rs1 ?rs2)))
    ; Facts for cap station
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (not (machine-used (mps ?cs) (order-id ?some-order-id)))
    =>
    (assert (machine-used (mps ?cs) (order-id ?ord)))
    (printout t ?cs " is now in use for order " ?ord crlf)
    (modify ?sg (mode SELECTED))
)

; Selection rule for C3: GOAL-PAY-RING2, if first and second ringstation are the same
(defrule central-run-parallel-subgoals-select-c3-pay-second-ring-same-ringstation ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (class GOAL-PAY-RING2) (mode FORMULATED)
	      (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class GOAL-ORDER-C3))

    ; First ring must be mounted:
    (goal (id ?to-rs-id) (parent ?id) (class GOAL-TO-RS1) (outcome COMPLETED))

    ; Facts for ring station:
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring1col ?num1))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring2col ?num2))
    ; Facts for cap station
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (not (machine-used (mps ?cs) (order-id ?some-order-id)))
    =>
    (assert (machine-used (mps ?cs) (order-id ?ord)))
    (printout t ?cs " is now in use for order " ?ord crlf)
    (modify ?sg (mode SELECTED))
)

; Selection rule for C3: GOAL-PAY-RING3, if second and third ringstation are different
(defrule central-run-parallel-subgoals-select-c3-pay-third-ring-different-ringstation ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (class GOAL-PAY-RING3) (mode FORMULATED)
	      (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class GOAL-ORDER-C3))

    ; Facts for ring station:
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name rs-ring-spec) (param-values ?rs2 ?ring2col ?num1))
    (domain-fact (name rs-ring-spec) (param-values ?rs3 ?ring3col ?num2))
    (test(not (eq ?rs2 ?rs3)))
    =>
    (modify ?sg (mode SELECTED))
)

; Selection rule for C3: GOAL-PAY-RING3, if second and third ringstation are the same
(defrule central-run-parallel-subgoals-select-c3-pay-third-ring-same-ringstation ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (class GOAL-PAY-RING3) (mode FORMULATED)
	      (is-executable TRUE))
    (goal-meta (goal-id ?sub-goal) (order-id ?ord) (root-for-order ?root-id))
    (goal (id ?root-id) (class GOAL-ORDER-C3))

    ; Second ring must be mounted before we pay for third ring at same station
    (goal (id ?to-rs-id) (parent ?id) (class GOAL-TO-RS2) (outcome COMPLETED))

    ; Facts for ring station:
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name rs-ring-spec) (param-values ?rs2 ?ring2col ?num1))
    (domain-fact (name rs-ring-spec) (param-values ?rs2 ?ring3col ?num2))
    =>
    (modify ?sg (mode SELECTED))
)

(defrule central-run-parallel-subgoals-evaluated ; 
    ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    (not (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (outcome ?outcome&~COMPLETED)))

    =>

    (modify ?gf (mode EVALUATED) (outcome COMPLETED))
)

(defrule central-run-parallel-subgoals-retract ; Retract subgoals after the parent goal is evaluated
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode EVALUATED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))

    =>

    (modify ?sg (mode RETRACTED))
)
