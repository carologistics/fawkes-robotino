; Parent goals
;----------------------------------------------------------------------------

; Root for C0 order

(defrule goal-assign-c0-order
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (domain-fact (name order-complexity) (param-values ?ord C0))
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
    (not (goal-meta (goal-id ?name&:(sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id)))
    =>
    (printout t "Goal " GOAL-PARALLEL-BS-CS " formulated" crlf)
    (bind ?goal-id (sym-cat GOAL-PARALLEL-BS-CS- ?ord))
    (assert (goal (class GOAL-PARALLEL-BS-CS)
                (id ?goal-id)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
                (parent ?root-goal-id)
                (verbosity NOISY) 
                (is-executable FALSE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-goal-id)))
)

; Simple goals
;----------------------------------------------------------------------------

; Get base
(defrule goal-get-bs
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal-meta (order-id ?ord) (root-for-order ?root-goal-id))
    (goal-meta (goal-id ?name1&:(sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id))
    (not (goal-meta (goal-id ?name2&:(sym-cat GOAL-GET-BS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id)))
    =>
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
    (goal-meta (order-id ?ord) (root-for-order ?root-goal-id))
    (goal-meta (goal-id ?name1&:(sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id))
    (not (goal-meta (goal-id ?name2&:(sym-cat GOAL-GET-CS- ?ord)) (order-id ?ord) (root-for-order ?root-goal-id)))
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

; Transport BS to CS
(defrule goal-bs-to-cs
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal-meta (order-id ?ord) (root-for-order ?root-goal-id))
    (goal-meta (goal-id ?name1&:(sym-cat GOAL-ORDER-C0- ?ord)) (order-id ?ord) (root-for-order ?root1&:(sym-cat GOAL-ORDER-C0- ?ord)))
    (not (goal-meta (goal-id ?name2&:(sym-cat GOAL-BS-TO-CS- ?ord)) (order-id ?ord) (root-for-order ?root2&:(sym-cat GOAL-ORDER-C0- ?ord))))
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
    (goal-meta (order-id ?ord) (root-for-order ?root-goal-id))
    (goal-meta (goal-id ?name1&:(sym-cat GOAL-ORDER-C0- ?ord)) (order-id ?ord) (root-for-order ?root1&:(sym-cat GOAL-ORDER-C0- ?ord)))
    (not (goal-meta (goal-id ?name2&:(sym-cat GOAL-DELIVER-C0- ?ord)) (order-id ?ord) (root-for-order ?root2&:(sym-cat GOAL-ORDER-C0- ?ord))))
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

;Expand get Base
(defrule goal-expander-goal-get-bs
	?g <- (goal (id ?goal-id) (class GOAL-GET-BS) (mode SELECTED) (parent ?parent))
	?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name mps-type) (param-values ?bs BS))

	=>
    
	(plan-assert-sequential GET-BS-PLAN ?goal-id ?robot
		(plan-assert-action prepare-bs ?bs OUTPUT ?basecol)
        (plan-assert-action spawn-wp (str-cat "wp" ?goal-id) ?robot)
        (plan-assert-action bs-dispense ?bs INPUT wp1 ?basecol)
	)
	(modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    ;(retract (domain-fact (name robot-waiting) (param-values ?robot))) ;We must insert this again when done executing!
)

;Expand get Cap
(defrule goal-expander-goal-get-cs
    ?g <- (goal (id ?goal-id) (class GOAL-GET-CS) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team

    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))

    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))

    =>

    (plan-assert-sequential GET-CS-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs INPUT)
        (plan-assert-action prepare-cs ?cs RETRIEVE_CAP)
        (plan-assert-action wp-get-shelf ?robot ?wp ?cs ?spot)
        (plan-assert-action wp-put ?robot ?wp ?cs INPUT)
        (plan-assert-action cs-retrieve-cap ?cs ?wp ?capcol)
        (plan-assert-action move ?robot ?cs INPUT ?cs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp ?cs OUTPUT)
        (plan-assert-action wp-discard ?robot ?wp)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    ;(retract (domain-fact (name robot-waiting) (param-values ?robot))) ;We must insert this again when done executing!


)
;Expand Transport BS to CS

(defrule goal-expander-goal-bs-to-cs
    ?g <- (goal (id ?goal-id) (class GOAL-BS-TO-CS) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))

    (domain-fact (name wp-at) (param-values ?wp ?bs ?side))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name mps-type) (param-values ?bs BS))
    (domain-fact (name mps-team) (param-values ?bs ?team-color))

    (domain-fact (name cs-buffered) (param-values ?cs ?capcol)) ; PROBLEM:two robots on two different goals could go to the same cs
    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    =>
    (plan-assert-sequential GET-CS-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?bs ?side)
        (plan-assert-action wp-get ?robot ?wp ?bs ?side)
        (plan-assert-action move ?robot ?bs ?side ?cs INPUT)
        (plan-assert-action wp-put ?robot ?wp ?cs INPUT)
        (plan-assert-action cs-mount-cap ?cs ?wp ?capcol)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    ;(retract (domain-fact (name robot-waiting) (param-values ?robot))) ;We must insert this again when done executing!

)

;Expand Transport C0 to DS
(defrule goal-expander-goal-deliver-c0
    ?g <- (goal (id ?goal-id) (class GOAL-DELIVER-C0) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-type) (param-values ?ds DS))

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name order-gate) (param-values ?ord ?gate))

    (domain-fact (name wp-at) (param-values ?wp ?cs OUTPUT))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (domain-fact (name wp-ring1-color) (param-values RING_NONE))
    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    =>
    (plan-assert-sequential GET-CS-PLAN ?goal-id ?robot
        (plan-assert-action wp-get ?robot ?wp ?cs OUTPUT)
        (plan-assert-action move ?robot ?fl1 ?fs1 ?ds INPUT)
        (plan-assert-action prepare-ds ?ds ?ord)
        (plan-assert-action wp-put ?robot ?wp ?ds INPUT)
        (plan-assert-action fulfill-order-c0 ?ord ?wp ?ds ?gate ?basecol ?capcol)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    ;(domain-fact (name robot-waiting) (param-values ?robot)) ;We must insert this again when done executing!

)