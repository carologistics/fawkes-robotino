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

(defrule central-run-parallel-subgoals-select ;
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode FORMULATED)
	      (is-executable TRUE))
    
    =>

    (modify ?sg (mode SELECTED))
)

(defrule central-run-parallel-subgoals-evaluated ; 
    ?gf <- (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode DISPATCHED))
    (not (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode ?mode&~EVALUATED)))

    =>

    (modify ?gf (mode EVALUATED) (outcome COMPLETED))
)

(defrule central-run-parallel-subgoals-retract ; Retract subgoals after the parent goal is evaluated
    (goal (id ?id) (type ACHIEVE) (sub-type CENTRAL-RUN-PARALLEL) (mode EVALUATED))
    ?sg <- (goal (id ?sub-goal) (parent ?id) (type ACHIEVE) (mode EVALUATED))

    =>

    (modify ?sg (mode RETRACTED))
)
