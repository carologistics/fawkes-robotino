; Parent goals
;----------------------------------------------------------------------------

; Root for C0 order

(defrule goal-assign-c0-order
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (order-complexity ?ord C0)
    (not (goal (id ?some-goal-id) (class GOAL-ORDER-C0)))   ; This has the effect, that there is only one C0 root-order formulated at any given point in time
    =>
    (printout t "Goal " GOAL-ORDER-C0 " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-ORDER-C0- ?ord ))
    (assert (goal (class GOAL-ORDER-C0)                     ; This declares the class for this goal, which is a blocking precondition for executing another C0 order
                (id ?goal-id)
                (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?goal-id)))
)

; Parallel goals
;----------------------------------------------------------------------------

; Parallel execution of BS and CS

(defrule goal-parallel-bs-cs
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal-meta (goal-id ?root-goal-id) (order-id ?ord) (root-for-order ?root-goal-id))
    (not (goal-meta (goal-id (sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (assigned-to ?r) (order-id ?ord) (root-for-order ?root-goal-id)))
    =>
    (printout t "Goal " GOAL-PARALLEL-BS-CS " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-PARALLEL-BS-CS- ?ord))
    (assert (goal (class GOAL-PARALLEL-BS-CS)
                (id ?goal-id)
                (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
                (verbosity NOISY) (is-exectuable FALSE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-goal-id)))
)

; Simple goals
;----------------------------------------------------------------------------

; Get base
(defrule goal-get-bs
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal-meta (goal-id (sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id))
    (not (goal-meta (goal-id (sym-cat GOAL-GET-BS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id)))
    =>
    (printout t "Goal " GOAL-GET-BS " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-GET-BS- ?ord))
    (assert (goal (class GOAL-GET-BS)
                (id ?goal-id)
                (sub-type )
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-goal-id)))
)

; Expand goals
;----------------------------------------------------------------------------

; ASSIGN ROBOTS ON EXPANDING STEP ONLY (not goal level)
; DO NOT FORGET TO SET ROBOTS TO not(robot-waiting ?r) WHILE EXECUTING!