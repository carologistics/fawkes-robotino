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

; Transport BS to CS
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

;Expand get Base
(defrule goal-expander-goal-get-bs
	?g <- (goal (id ?goal-id) (class GOAL-GET-BS) (mode SELECTED) (parent ?parent))
	?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (robot-waiting ?robot)
    ; WIP! select correct BS based on team color
    ; in our case this is always CYAN (doesnt matter if we set it to CYAN or leave it at "?team-color")
    (domain-object (name ?team-color) (type team-color))

    (order-base-color ?ord ?basecol)
    (mps-team ?bs&:(mps-type ?bs BS) ?team-color) ;Does this work like this?

	=>
    
	(plan-assert-sequential GET-BS-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl ?fs ?bs INPUT)
		(plan-assert-action prepare-bs ?bs OUTPUT ?basecol)
        (plan-assert-action spawn-wp (str-cat "wp" ?goal-id) ?robot)
        (plan-assert-action bs-dispense ?bs INPUT wp1 ?basecol)
	)
	(modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (retract (robot-waiting ?robot)) ;We must insert this again when done executing!
)

;Expand get Cap
(defrule goal-expander-goal-get-cs
    ?g <- (goal (id ?goal-id) (class GOAL-GET-CS) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (robot-waiting ?robot)
    ; WIP! select correct BS based on team color
    ; in our case this is always CYAN (doesnt matter if we set it to CYAN or leave it at "?team-color")
    (domain-object (name ?team-color) (type team-color))

    (order-cap-color ?ord ?capcol)

    (mps-team ?cs&:(mps-type ?cs CS) ?team-color) ;Does this work like this?
    (order-cap-color ?ord ?capcol)
    (wp-on-shelf ?wp&:(wp-cap-color ?wp ?capcol) ?cs ?spot)

    =>

    (plan-assert-sequential GET-CS-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs INPUT)
        (plan-assert-action prepare-cs ?cs RETRIEVE_CAP)
        (plan-assert-action wp-get-shelf ?robot ?wp ?cs ?spot)
        (plan-assert-action wp-put ?robot ?wp ?cs INPUT)
        (plan-assert-action cs-retrieve-cap ?cs ?wp ?capcol)
        (plan-assert-action move ?robot ?fl2 ?fs2 ?cs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp ?cs OUTPUT)
        (plan-assert-action wp-discard ?robot ?cc)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (retract (robot-waiting ?robot)) ;We must insert this again when done executing!


)
;Expand Transport BS to CS

(defrule goal-expander-goal-bs-to-cs
    ?g <- (goal (id ?goal-id) (class GOAL-BS-TO-CS) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (robot-waiting ?robot)
    ; WIP! select correct BS based on team color
    ; in our case this is always CYAN (doesnt matter if we set it to CYAN or leave it at "?team-color")
    (domain-object (name ?team-color) (type team-color))
    (order-base-color ?ord ?basecol)
    (order-cap-color ?ord ?capcol)

    (wp-at ?wp&:(wp-base-color ?wp ?basecol) ?bs&:(and (mps-type ?bs BS) (mps-team ?bs ?team-color)) ?side) ;and hier Erlaubt?
    (cs-buffered ?cs ?capcol) ;PROBLEM:two robots on two different goals could go to the same cs

    =>
    (plan-assert-sequential GET-CS-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?bs ?side)
        (plan-assert-action wp-get ?robot ?wp ?bs ?side)
        (plan-assert-action move ?robot ?fl2 ?fs2 ?cs INPUT)
        (plan-assert-action wp-put ?robot ?wp ?cs INPUT)
        (plan-assert-action cs-mount-cap ?cs ?wp ?capcol)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (retract (robot-waiting ?robot)) ;We must insert this again when done executing!

)

;Expand Transport C0 to DS
(defrule goal-expander-goal-deliver-c0
    ?g <- (goal (id ?goal-id) (class GOAL-DELIVER-C0) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    (robot-waiting ?robot)
    ; WIP! select correct BS based on team color
    ; in our case this is always CYAN (doesnt matter if we set it to CYAN or leave it at "?team-color")
    (domain-object (name ?team-color) (type team-color))
    (mps-team ?ds&:(mps-type ?ds DS) ?team-color)

    (order-base-color ?ord ?basecol)
    (order-cap-color ?ord ?capcol)
    (order-gate ?ord ?gate)

    (wp-at ?wp&:(and (wp-base-color ?wp ?basecol) (wp-cap-color ?wp ?capcol) (wp-ring1-color RING_NONE)) ?cs&:(mps-type ?cs CS) OUTPUT)

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
    (retract (robot-waiting ?robot)) ;We must insert this again when done executing!

)