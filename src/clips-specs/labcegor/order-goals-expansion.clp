; Expand goals
;----------------------------------------------------------------------------

; ASSIGN ROBOTS ON EXPANDING STEP ONLY (not goal level)

;Expand get Base
(defrule goal-expander-goal-get-bs
	?g <- (goal (id ?goal-id) (class GOAL-GET-BS) (mode SELECTED) (parent ?parent))
	?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name mps-type) (param-values ?bs BS))

	=>
    (bind ?wp (sym-cat wp ?goal-id))
	(plan-assert-sequential GET-BS-PLAN ?goal-id ?robot
		(plan-assert-action prepare-bs ?bs OUTPUT ?basecol)
        (plan-assert-action spawn-wp ?wp ?robot)
        (plan-assert-action bs-dispense ?bs OUTPUT ?wp ?basecol)
	)
	(modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (printout t "Robot " ?robot " was assigned GET-BS-" ?ord crlf)
    (retract ?rw) 
)

;Expand get Cap
(defrule goal-expander-goal-get-cs
    ?g <- (goal (id ?goal-id) (class GOAL-GET-CS) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
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
    (printout t "Robot " ?robot " was assigned GET-CS-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!


)

; expansion rule to do a payment for RING1 from BS
(defrule goal-expander-GOAL-DO-PAYMENT-RING1
    ?g <- (goal (id ?goal-id) (class GOAL-DO-PAYMENT) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (goal (id ?parent) (class GOAL-PAY-RING1))

    (domain-fact (name order-ring1-color ) (param-values ?ord ?r1col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?r1col ?r1cost))


    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name mps-type) (param-values ?bs BS))

    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    

    =>
    (bind ?wp (sym-cat wp-payment- ?goal-id))
    (plan-assert-sequential PAY-RING1 ?goal-id ?robot
        (plan-assert-action prepare-bs ?bs OUTPUT BASE_BLACK)
        (plan-assert-action spawn-wp ?wp ?robot)
        (plan-assert-action bs-dispense ?bs OUTPUT ?wp BASE_BLACK)

        (plan-assert-action move ?robot ?fl1 ?fs1 ?bs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp ?bs OUTPUT)
        (plan-assert-action move ?robot ?bs OUTPUT ?rs INPUT)
        (plan-assert-action wp-put-slide-cc ?robot ?wp ?rs ?rs-before ?rs-after)
    )

    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned to do a payment for GOAL-PAY-RING1-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; expansion rule to do a payment for RING2 from CS
(defrule goal-expander-GOAL-DO-PAYMENT-RING2
    ?g <- (goal (id ?goal-id) (class GOAL-DO-PAYMENT) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (goal (id ?parent) (class GOAL-PAY-RING2))

    (domain-fact (name order-ring2-color ) (param-values ?ord ?r2col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?r2col ?r2cost))

    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))

    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))

    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))

    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol)) ; We select a workpiece from the same shelf that we need later for our cap

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))


    =>
    (plan-assert-sequential PAY-RING2 ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs INPUT)
        (plan-assert-action wp-get-shelf ?robot ?wp ?cs ?spot)
        (plan-assert-action move ?robot ?cs INPUT ?rs INPUT)
        (plan-assert-action wp-put-slide-cc ?robot ?wp ?rs ?rs-before ?rs-after)
    )

    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned to do a payment for GOAL-PAY-RING2-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; expansion rule to do a payment for RING3 from CS
(defrule goal-expander-GOAL-DO-PAYMENT-RING3
    ?g <- (goal (id ?goal-id) (class GOAL-DO-PAYMENT) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (goal (id ?parent) (class GOAL-PAY-RING3))

    (domain-fact (name rs-ring-spec) (param-values ?rs ?r3col ?r3cost))
    (domain-fact (name order-ring2-color ) (param-values ?ord ?r3col))

    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))

    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))

    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))

    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol)) ; We select a workpiece from the same shelf that we need later for our cap

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))


    =>
    (plan-assert-sequential PAY-RING3 ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs INPUT)
        (plan-assert-action wp-get-shelf ?robot ?wp ?cs ?spot)
        (plan-assert-action move ?robot ?cs INPUT ?rs INPUT)
        (plan-assert-action wp-put-slide-cc ?robot ?wp ?rs ?rs-before ?rs-after)
    )

    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned to do a payment for GOAL-PAY-RING3-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; expansion rule for mounting first ring
(defrule goal-expander-goal-to-rs1
    ?g <- (goal (id ?goal-id) (class GOAL-TO-RS1) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))

    (domain-fact (name wp-at) (param-values ?wp ?s ?side))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-ring1-color) (param-values ?wp RING_NONE))

    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring1col ?rs-req))
    (domain-fact (name rs-sub) (param-values ?rs-before ?rs-req ?rs-after))

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))

    =>
    (plan-assert-sequential TRANSPORT-TO-RS1-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?s ?side)
        (plan-assert-action wp-get ?robot ?wp ?s ?side)
        (plan-assert-action move ?robot ?s ?side ?rs INPUT)
        (plan-assert-action prepare-rs ?rs ?ring1col ?rs-before ?rs-after ?rs-req)
        (plan-assert-action wp-put ?robot ?wp ?rs INPUT)
        (plan-assert-action rs-mount-ring1 ?rs ?wp ?ring1col ?rs-before ?rs-after ?rs-req)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned TO-RS1-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; expansion rule for mounting second ring
(defrule goal-expander-goal-to-rs2
    ?g <- (goal (id ?goal-id) (class GOAL-TO-RS2) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))

    (domain-fact (name wp-at) (param-values ?wp ?s ?side))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-ring1-color) (param-values ?wp ?ring1col))
    (domain-fact (name wp-ring2-color) (param-values ?wp RING_NONE))

    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring2col ?rs-req))
    (domain-fact (name rs-sub) (param-values ?rs-before ?rs-req ?rs-after))

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))

    =>
    (plan-assert-sequential TRANSPORT-TO-RS2-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?s ?side)
        (plan-assert-action wp-get ?robot ?wp ?s ?side)
        (plan-assert-action move ?robot ?s ?side ?rs INPUT)
        (plan-assert-action prepare-rs ?rs ?ring2col ?rs-before ?rs-after ?rs-req)
        (plan-assert-action wp-put ?robot ?wp ?rs INPUT)
        (plan-assert-action rs-mount-ring2 ?rs ?wp ?ring2col ?ring1col ?rs-before ?rs-after ?rs-req)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned TO-RS2-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; expansion rule for mounting third ring
(defrule goal-expander-goal-to-rs3
    ?g <- (goal (id ?goal-id) (class GOAL-TO-RS3) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))

    (domain-fact (name wp-at) (param-values ?wp ?s ?side))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-ring1-color) (param-values ?wp ?ring1col))
    (domain-fact (name wp-ring2-color) (param-values ?wp ?ring2col))
    (domain-fact (name wp-ring3-color) (param-values ?wp RING_NONE))

    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring3col ?rs-req))
    (domain-fact (name rs-sub) (param-values ?rs-before ?rs-req ?rs-after))

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))

    =>
    (plan-assert-sequential TRANSPORT-TO-RS3-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?s ?side)
        (plan-assert-action wp-get ?robot ?wp ?s ?side)
        (plan-assert-action move ?robot ?s ?side ?rs INPUT)
        (plan-assert-action prepare-rs ?rs ?ring3col ?rs-before ?rs-after ?rs-req)
        (plan-assert-action wp-put ?robot ?wp ?rs INPUT)
        (plan-assert-action rs-mount-ring3 ?rs ?wp ?ring3col ?ring1col ?ring2col ?rs-before ?rs-after ?rs-req)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned TO-RS3-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

;Expand Transport from any Station with fitting workpiece to CS
(defrule goal-expander-goal-to-cs
    ?g <- (goal (id ?goal-id) (class GOAL-TO-CS) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
    
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))

    (domain-fact (name wp-at) (param-values ?wp ?s ?side))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-ring1-color) (param-values ?wp ?ring1col))
    (domain-fact (name wp-ring2-color) (param-values ?wp ?ring2col))
    (domain-fact (name wp-ring3-color) (param-values ?wp ?ring3col))
    (domain-fact (name mps-team) (param-values ?s ?team-color))

    (domain-fact (name cs-buffered) (param-values ?cs ?capcol)) ; PROBLEM:two robots on two different goals could go to the same cs
    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    =>
    (plan-assert-sequential TRANSPORT-TO-CS-PLAN ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?s ?side)
        (plan-assert-action wp-get ?robot ?wp ?s ?side)
        (plan-assert-action move ?robot ?s ?side ?cs INPUT)
        (plan-assert-action prepare-cs ?cs MOUNT_CAP)
        (plan-assert-action wp-put ?robot ?wp ?cs INPUT)
        (plan-assert-action cs-mount-cap ?cs ?wp ?capcol)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (printout t "Robot " ?robot " was assigned TO-CS-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!

)

;Expand Transport C0 to DS
(defrule goal-expander-goal-deliver-c0
    ?g <- (goal (id ?goal-id) (class GOAL-DELIVER-C0) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-type) (param-values ?ds DS))

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name order-gate) (param-values ?ord ?gate))

    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-at) (param-values ?wp ?cs OUTPUT))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (domain-fact (name wp-ring1-color) (param-values ?wp ?ring1col))
    (domain-fact (name wp-ring2-color) (param-values ?wp ?ring2col))
    (domain-fact (name wp-ring3-color) (param-values ?wp ?ring3col))
    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    =>
    (plan-assert-sequential DELIVER-C0 ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp ?cs OUTPUT)
        (plan-assert-action move ?robot ?cs OUTPUT ?ds INPUT)
        (plan-assert-action prepare-ds ?ds ?ord)
        (plan-assert-action wp-put ?robot ?wp ?ds INPUT)
        (plan-assert-action fulfill-order-c0 ?ord ?wp ?ds ?gate ?basecol ?capcol)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (printout t "Robot " ?robot " was assigned DELIVER-C0-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!

)

; Expand Transport C1 to DS
(defrule goal-expander-goal-deliver-c1
    ?g <- (goal (id ?goal-id) (class GOAL-DELIVER-C1) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-type) (param-values ?ds DS))

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name order-gate) (param-values ?ord ?gate))

    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-at) (param-values ?wp ?cs OUTPUT))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (domain-fact (name wp-ring1-color) (param-values ?wp ?ring1col))
    (domain-fact (name wp-ring2-color) (param-values ?wp ?ring2col))
    (domain-fact (name wp-ring3-color) (param-values ?wp ?ring3col))
    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    =>
    (plan-assert-sequential DELIVER-C1 ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp ?cs OUTPUT)
        (plan-assert-action move ?robot ?cs OUTPUT ?ds INPUT)
        (plan-assert-action prepare-ds ?ds ?ord)
        (plan-assert-action wp-put ?robot ?wp ?ds INPUT)
        (plan-assert-action fulfill-order-c1 ?ord ?wp ?ds ?gate ?basecol ?capcol ?ring1col)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (printout t "Robot " ?robot " was assigned DELIVER-C1-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!

)

; Expand Transport C2 to DS
(defrule goal-expander-goal-deliver-c2
    ?g <- (goal (id ?goal-id) (class GOAL-DELIVER-C2) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-type) (param-values ?ds DS))

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name order-gate) (param-values ?ord ?gate))

    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-at) (param-values ?wp ?cs OUTPUT))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (domain-fact (name wp-ring1-color) (param-values ?wp ?ring1col))
    (domain-fact (name wp-ring2-color) (param-values ?wp ?ring2col))
    (domain-fact (name wp-ring3-color) (param-values ?wp ?ring3col))
    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    =>
    (plan-assert-sequential DELIVER-C2 ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp ?cs OUTPUT)
        (plan-assert-action move ?robot ?cs OUTPUT ?ds INPUT)
        (plan-assert-action prepare-ds ?ds ?ord)
        (plan-assert-action wp-put ?robot ?wp ?ds INPUT)
        (plan-assert-action fulfill-order-c2 ?ord ?wp ?ds ?gate ?basecol ?capcol ?ring1col ?ring2col)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (printout t "Robot " ?robot " was assigned DELIVER-C2-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; Expand Transport C3 to DS
(defrule goal-expander-goal-deliver-c3
    ?g <- (goal (id ?goal-id) (class GOAL-DELIVER-C3) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))
    (domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-type) (param-values ?ds DS))

    (domain-fact (name order-base-color) (param-values ?ord ?basecol))
    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
    (domain-fact (name order-cap-color) (param-values ?ord ?capcol))
    (domain-fact (name order-gate) (param-values ?ord ?gate))

    (domain-fact (name mps-type) (param-values ?cs CS))
    (domain-fact (name wp-at) (param-values ?wp ?cs OUTPUT))
    (domain-fact (name wp-base-color) (param-values ?wp ?basecol))
    (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
    (domain-fact (name wp-ring1-color) (param-values ?wp ?ring1col))
    (domain-fact (name wp-ring2-color) (param-values ?wp ?ring2col))
    (domain-fact (name wp-ring3-color) (param-values ?wp ?ring3col))
    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    =>
    (plan-assert-sequential DELIVER-C3 ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp ?cs OUTPUT)
        (plan-assert-action move ?robot ?cs OUTPUT ?ds INPUT)
        (plan-assert-action prepare-ds ?ds ?ord)
        (plan-assert-action wp-put ?robot ?wp ?ds INPUT)
        (plan-assert-action fulfill-order-c3 ?ord ?wp ?ds ?gate ?basecol ?capcol ?ring1col ?ring2col ?ring3col)
    )
    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))
    (printout t "Robot " ?robot " was assigned DELIVER-C3-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)
