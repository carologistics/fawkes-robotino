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
                (type ACHIEVE)
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
    (not (goal-meta (goal-id (sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id)))
    =>
    (printout t "Goal " GOAL-PARALLEL-BS-CS " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-PARALLEL-BS-CS- ?ord))
    (assert (goal (class GOAL-PARALLEL-BS-CS)
                (id ?goal-id)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
                (parent ?root-goal-id)
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
    =>GOAL-PARALLEL-BS-CS-
    (printout t "Goal " GOAL-GET-BS " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-GET-BS- ?ord))
    (assert (goal (class GOAL-GET-BS)
                (id ?goal-id)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent (sym-cat GOAL-PARALLEL-BS-CS- ?ord))
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-goal-id)))
)

; Get CS
(defrule goal-get-cs
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal-meta (goal-id (sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id))
    (not (goal-meta (goal-id (sym-cat GOAL-GET-CS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id)))
    =>
    (printout t "Goal " GOAL-GET-CS " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-GET-CS- ?ord))
    (assert (goal (class GOAL-GET-CS)
                (id ?goal-id)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent (sym-cat GOAL-PARALLEL-BS-CS- ?ord))
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-goal-id)))
)

; Transport BC to CS
(defrule goal-bs-to-cs
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal-meta (goal-id (sym-cat GOAL-ORDER-C0- ?ord)) (order-id ?ord) (root-for-order (sym-cat GOAL-ORDER-C0- ?ord)))
    (not (goal-meta (goal-id (sym-cat GOAL-BS-TO-CS- ?ord)) (order-id ?ord) (root-for-order (sym-cat GOAL-ORDER-C0- ?ord))))
    =>
    (printout t "Goal " GOAL-BS-TO-CS " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-BS-TO-CS- ?ord))
    (assert (goal (class GOAL-BS-TO-CS)
                (id ?goal-id)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent (sym-cat GOAL-ORDER-C0- ?ord))
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order (sym-cat GOAL-ORDER-C0- ?ord))))
)

; Transport C0 to DS
(defrule goal-deliver-c0
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal-meta (goal-id (sym-cat GOAL-ORDER-C0- ?ord)) (order-id ?ord) (root-for-order (sym-cat GOAL-ORDER-C0- ?ord)))
    (not (goal-meta (goal-id (sym-cat GOAL-DELIVER-C0- ?ord)) (order-id ?ord) (root-for-order (sym-cat GOAL-ORDER-C0- ?ord))))
    =>
    (printout t "Goal " GOAL-DELIVER-C0 " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-DELIVER-C0- ?ord))
    (assert (goal (class GOAL-DELIVER-C0)
                (id ?goal-id)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent (sym-cat GOAL-ORDER-C0- ?ord))
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order (sym-cat GOAL-ORDER-C0- ?ord))))
)

; Expand goals
;----------------------------------------------------------------------------

; ASSIGN ROBOTS ON EXPANDING STEP ONLY (not goal level)
; DO NOT FORGET TO SET ROBOTS TO not(robot-waiting ?r) WHILE EXECUTING!

(defrule goal-expander-goal-get-bs
	?g <- (goal (id ?goal-id) (class GOAL-GET-BS) (mode SELECTED) (parent ?parent))
	?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (robot-waiting ?robot)
    ; WIP! select correct BS based on team color
    (domain-object (name ?team-color) (type team-color))
    (domain-fact (name mps-type) (param-values C-BS BS))

    (order-base-color ?ord ?basecol)
    (mps-team ?bs ?team-color)

	=>
    
	(plan-assert-sequential GET-BS-PLAN ?goal-id ?robot
		(plan-assert-action prepare-bs ?bs OUTPUT ?basecol)
        (plan-assert-action spawn-wp ;WIP)
        (plan-assert-action bs-dispense ;WIP)
	)
	(modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
)