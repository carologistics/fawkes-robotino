
; Assert goals
;----------------------------------------------------------------------------

(defrule assign-c0-order
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (not (and (domain-object (name ?robot) (type robot)) (not (domain-fact (name entered-field) (param-values ?robot)))))
    
    (domain-fact (name order-complexity) (param-values ?ord C0))
    (not (domain-fact (name order-fulfilled) (param-values ?ord)))

    (not (goal-meta (order-id ?ord)))
    (not (goal (id ?some-goal-id) (class GOAL-ORDER-C0) (outcome ~COMPLETED)))
    (not (goal (id ?some-goal-id) (class GOAL-ORDER-C1) (outcome ~COMPLETED)))
    (not (goal (id ?some-goal-id) (class GOAL-ORDER-C2) (outcome ~COMPLETED)))
    (not (goal (id ?some-goal-id) (class GOAL-ORDER-C3) (outcome ~COMPLETED))) ;This has the effect, that always there is only one root-order which is not COMPLETED
    
    =>
    (printout t "Building C0-Tree ..." crlf)

    (bind ?goal-id-root (sym-cat GOAL-ORDER-C0- ?ord ))
    (bind ?goal-id-parallel (sym-cat GOAL-PARALLEL-BS-CS- ?ord ))
    (bind ?goal-id-bs (sym-cat GOAL-GET-BS- ?ord ))
    (bind ?goal-id-cs (sym-cat GOAL-GET-CS- ?ord ))
    (bind ?goal-id-tocs (sym-cat GOAL-TO-CS- ?ord ))
    (bind ?goal-id-csds (sym-cat GOAL-DELIVER-C0- ?ord ))

    ; Root Goal
    (assert (goal (class GOAL-ORDER-C0)                     
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
    (assert (goal (class GOAL-TO-CS)
                (id ?goal-id-tocs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 2.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-tocs) (order-id ?ord) (root-for-order ?goal-id-root)))

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

; (defrule assign-c1-order
;     (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;     (not (and (domain-object (name ?robot) (type robot)) (not (domain-fact (name entered-field) (param-values ?robot)))))
    
;     (domain-fact (name order-complexity) (param-values ?ord C1))
;     (not (domain-fact (name order-fulfilled) (param-values ?ord)))

;     (not (goal-meta (order-id ?ord)))
;     (not (goal (id ?some-goal-id) (class GOAL-ORDER-C0) (outcome ~COMPLETED)))
;     (not (goal (id ?some-goal-id) (class GOAL-ORDER-C1) (outcome ~COMPLETED)))
;     (not (goal (id ?some-goal-id) (class GOAL-ORDER-C2) (outcome ~COMPLETED)))
;     (not (goal (id ?some-goal-id) (class GOAL-ORDER-C3) (outcome ~COMPLETED))) ;This has the effect, that always there is only one root-order which is not COMPLETED

;     =>
;     (printout t "Building C1-Tree ..." crlf)

;     (bind ?goal-id-root (sym-cat GOAL-ORDER-C1- ?ord ))

;     (bind ?goal-id-bs (sym-cat GOAL-GET-BS- ?ord ))
;     (bind ?goal-id-parallel (sym-cat GOAL-PARALLEL-BS-TO-RS-AND-GET-CS- ?ord ))
;     (bind ?goal-id-bsrs (sym-cat GOAL-BS-TO-RS- ?ord ))
;     (bind ?goal-id-cs (sym-cat GOAL-GET-CS- ?ord ))
;     (bind ?goal-id-tocs (sym-cat GOAL-TO-CS- ?ord ))
;     (bind ?goal-id-csds (sym-cat GOAL-DELIVER-C1- ?ord ))

;     ; Root Goal
;     (assert (goal (class GOAL-ORDER-C1)                     
;                 (id ?goal-id-root)
;                 (type ACHIEVE)
;                 (sub-type CENTRAL-RUN-LINEAR)
;                 (verbosity NOISY) (is-executable TRUE)
;                 (meta-template goal-meta)
;     ))
;     (assert (goal-meta (goal-id ?goal-id-root) (order-id ?ord) (root-for-order ?goal-id-root)))
    
; )


; Goal executablity
;----------------------------------------------------------------------------
;CHECK LATER IF THIS IS NEEDED

; (defrule execute-bscs
;     (domain-fact (name order-complexity) (param-values ?ord C0))
; 	(goal (id ?id&:(sym-cat GOAL-PARALLEL-BS-CS- ?ord)) (type ACHIEVE) (is-executable TRUE) (mode EVALUATED))
; 	?g <- (goal (parent ?id&:(sym-cat GOAL-BS-TO-CS- ?ord)) (type ACHIEVE) (is-executable FALSE))
; 	=>
; 	(modify ?g (is-executable TRUE))
; )

; (defrule execute-csds
;     (domain-fact (name order-complexity) (param-values ?ord C0))
; 	(goal (id ?id&:(sym-cat GOAL-BS-TO-CS- ?ord)) (type ACHIEVE) (is-executable TRUE) (mode EVALUATED))
; 	?g <- (goal (parent ?id&:(sym-cat GOAL-CS-TO-DS- ?ord)) (type ACHIEVE) (is-executable FALSE))
; 	=>
; 	(modify ?g (is-executable TRUE))
; )

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