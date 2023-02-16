
; Assert goals
;----------------------------------------------------------------------------

(defrule assign-c0-order
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (domain-fact (name order-complexity) (param-values ?ord C0))
    (not (goal (id ?some-goal-id) (class GOAL-ORDER-C0)))   ; This has the effect, that there is only one C0 root-order formulated at any given point in time
    =>
    (printout t "Building C0-Tree ..." crlf)

    (bind ?goal-id-root (sym-cat GOAL-ORDER-C0- ?ord ))
    (bind ?goal-id-parallel (sym-cat GOAL-PARALLEL-BS-CS- ?ord ))
    (bind ?goal-id-bs (sym-cat GOAL-GET-BS- ?ord ))
    (bind ?goal-id-cs (sym-cat GOAL-GET-CS- ?ord ))
    (bind ?goal-id-bscs (sym-cat GOAL-BS-TO-CS- ?ord ))
    (bind ?goal-id-csds (sym-cat GOAL-DELIVER-C0- ?ord ))

    ; Root Goal
    (assert (goal (class GOAL-ORDER-C0)                     ; This declares the class for this goal, which is a blocking precondition for executing another C0 order
                (id ?goal-id-root)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-LINEAR)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-root) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Parallel part goal
    (assert (goal (class GOAL-PARALLEL-BS-CS)
                (id ?goal-id-parallel)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-PARALLEL)
                (parent ?goal-id-root)
                (verbosity NOISY)
                (priority 1.0) 
                (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-parallel) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal get BS
    (assert (goal (class GOAL-GET-BS)
                (id ?goal-id-bs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-parallel)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-bs) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal get CS
    (assert (goal (class GOAL-GET-CS)
                (id ?goal-id-cs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-parallel)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-cs) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal BS -> CS
    (assert (goal (class GOAL-BS-TO-CS)
                (id ?goal-id-bscs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 2.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-bscs) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal CS -> DS
    (assert (goal (class GOAL-DELIVER-C0)
                (id ?goal-id-csds)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 3.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-csds) (order-id ?ord) (root-for-order ?goal-id-root)))
)

; Goal executablity
;----------------------------------------------------------------------------

(defrule execute-bscs
    (domain-fact (name order-complexity) (param-values ?ord C0))
	(goal (id ?id&:(sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (type ACHIEVE) (is-executable TRUE) (mode EVALUATED))
	?g <- (goal (parent ?id&:(sym-cat GOAL-BS-TO-CS- ?ord)) (type ACHIEVE) (is-executable FALSE))
	=>
	(modify ?g (is-executable TRUE))
)

(defrule execute-csds
    (domain-fact (name order-complexity) (param-values ?ord C0))
	(goal (id ?id&:(sym-cat GOAL-BS-TO-CS- ?ord)) (type ACHIEVE) (is-executable TRUE) (mode EVALUATED))
	?g <- (goal (parent ?id&:(sym-cat GOAL-CS-TO-DS- ?ord)) (type ACHIEVE) (is-executable FALSE))
	=>
	(modify ?g (is-executable TRUE))
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
    (bind ?wp (str-cat "wpdoghiduyfghsuiy" ?goal-id))
	(plan-assert-sequential GET-BS-PLAN ?goal-id ?robot
		(plan-assert-action prepare-bs ?bs OUTPUT ?basecol)
        (plan-assert-action spawn-wp ?wp ?robot)
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