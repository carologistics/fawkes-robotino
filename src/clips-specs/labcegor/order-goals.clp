
; Assert goals
;----------------------------------------------------------------------------

(defrule assign-c0-order
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (not (and (domain-object (name ?robot) (type robot)) (not (domain-fact (name entered-field) (param-values ?robot)))))
    
    (domain-fact (name order-complexity) (param-values ?ord C0))
    (not (domain-fact (name order-fulfilled) (param-values ?ord)))

    (not (goal-meta (order-id ?ord)))
    
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

    ; Goal -> CS
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

(defrule assign-c1-order
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (not (and (domain-object (name ?robot) (type robot)) (not (domain-fact (name entered-field) (param-values ?robot)))))
    
    (domain-fact (name order-complexity) (param-values ?ord C1))
    (not (domain-fact (name order-fulfilled) (param-values ?ord)))

    (not (goal-meta (order-id ?ord)))

    =>
    (printout t "Building C1-Tree ..." crlf)

    (bind ?goal-id-root (sym-cat GOAL-ORDER-C1- ?ord ))
    (bind ?goal-id-payring (sym-cat GOAL-PAY-FIRST-RING- ?ord )) ;needs expansion
    (bind ?goal-id-bs (sym-cat GOAL-GET-BS- ?ord ))
    (bind ?goal-id-parallel (sym-cat GOAL-PARALLEL-TO-RS-AND-GET-CS- ?ord ))
    (bind ?goal-id-tors (sym-cat GOAL-TO-RS- ?ord )) ;needs expansion
    (bind ?goal-id-cs (sym-cat GOAL-GET-CS- ?ord ))
    (bind ?goal-id-tocs (sym-cat GOAL-TO-CS- ?ord ))
    (bind ?goal-id-csds (sym-cat GOAL-DELIVER-C1- ?ord )) ;needs expansion (copy paste)

    ; Root Goal
    (assert (goal (class GOAL-ORDER-C1)                     
                (id ?goal-id-root)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-LINEAR)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-root) (order-id ?ord) (root-for-order ?goal-id-root)))
    ; Goal Pay One Ring
    (assert (goal (class GOAL-PAY-FIRST-RING)
                (id ?goal-id-payring)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) 
                (priority 1.0)
                (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-payring) (order-id ?ord) (root-for-order ?goal-id-root)))
    ; Goal get BS
    (assert (goal (class GOAL-GET-BS)
                (id ?goal-id-bs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (priority 2.0)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-bs) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Parallel part goal
    (assert (goal (class GOAL-PARALLEL-TO-RS-AND-GET-CS)
                (id ?goal-id-parallel)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-PARALLEL)
                (parent ?goal-id-root)
                (verbosity NOISY)
                (priority 3.0) 
                (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-parallel) (order-id ?ord) (root-for-order ?goal-id-root)))
    ; Goal TO-RS
    (assert (goal (class GOAL-TO-RS1)
                (id ?goal-id-tors)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-parallel)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-tors) (order-id ?ord) (root-for-order ?goal-id-root)))
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
    ; Goal to CS
    (assert (goal (class GOAL-TO-CS)
                (id ?goal-id-tocs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 4.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-tocs) (order-id ?ord) (root-for-order ?goal-id-root)))
    ; Goal CS -> DS
    (assert (goal (class GOAL-DELIVER-C1)
                (id ?goal-id-csds)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 5.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-csds) (order-id ?ord) (root-for-order ?goal-id-root)))
)

(defrule assign-c2-order
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (not (and (domain-object (name ?robot) (type robot)) (not (domain-fact (name entered-field) (param-values ?robot)))))
    
    (domain-fact (name order-complexity) (param-values ?ord C2))
    (not (domain-fact (name order-fulfilled) (param-values ?ord)))

    (not (goal-meta (order-id ?ord)))

    =>
    (printout t "Building C2-Tree ..." crlf)

    (bind ?goal-id-root (sym-cat GOAL-ORDER-C2- ?ord ))
    (bind ?goal-id-payring1 (sym-cat GOAL-PAY-FIRST-RING- ?ord ))
    (bind ?goal-id-payring2 (sym-cat GOAL-PAY-SECOND-RING- ?ord ))
    (bind ?goal-id-bs (sym-cat GOAL-GET-BS- ?ord ))
    (bind ?goal-id-parallel1 (sym-cat GOAL-PARALLEL-TO-RS1-AND-PREPARE-RS2- ?ord))
    (bind ?goal-id-parallel2 (sym-cat GOAL-PARALLEL-TO-RS2-AND-GET-CS- ?ord ))
    (bind ?goal-id-tors1 (sym-cat GOAL-TO-RS1- ?ord ))
    (bind ?goal-id-tors2 (sym-cat GOAL-TO-RS2- ?ord ))
    (bind ?goal-id-cs (sym-cat GOAL-GET-CS- ?ord ))
    (bind ?goal-id-tocs (sym-cat GOAL-TO-CS- ?ord ))
    (bind ?goal-id-csds (sym-cat GOAL-DELIVER-C2- ?ord ))

    ; Root Goal
    (assert (goal (class GOAL-ORDER-C2)
                (id ?goal-id-root)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-LINEAR)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-root) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal Pay First Ring
    (assert (goal (class GOAL-PAY-FIRST-RING)
                (id ?goal-id-payring1)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY)
                (priority 1.0)
                (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-payring1) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal get BS
    (assert (goal (class GOAL-GET-BS)
                (id ?goal-id-bs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (priority 2.0)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-bs) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Parallel part goal 1
    (assert (goal (class GOAL-PARALLEL-TO-RS1-AND-PREPARE-RS2)
                (id ?goal-id-parallel1)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-PARALLEL)
                (parent ?goal-id-root)
                (verbosity NOISY)
                (priority 3.0)
                (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-parallel1) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal TO-RS1
    (assert (goal (class GOAL-TO-RS1)
                (id ?goal-id-tors1)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-parallel1)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-tors1) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal Pay Second Ring
    (assert (goal (class GOAL-PAY-SECOND-RING)
                (id ?goal-id-payring2)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-parallel1)
                (verbosity NOISY)
                (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-payring2) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Parallel part goal 2
    (assert (goal (class GOAL-PARALLEL-TO-RS2-AND-GET-CS)
                (id ?goal-id-parallel2)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-PARALLEL)
                (parent ?goal-id-root)
                (verbosity NOISY)
                (priority 4.0)
                (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-parallel2) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal TO-RS2
    (assert (goal (class GOAL-TO-RS2)
                (id ?goal-id-tors2)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-parallel2)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-tors2) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal get CS
    (assert (goal (class GOAL-GET-CS)
                (id ?goal-id-cs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-parallel2)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-cs) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal to CS
    (assert (goal (class GOAL-TO-CS)
                (id ?goal-id-tocs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 5.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-tocs) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal CS -> DS
    (assert (goal (class GOAL-DELIVER-C2)
                (id ?goal-id-csds)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 6.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-csds) (order-id ?ord) (root-for-order ?goal-id-root)))

)


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

; expansion rule to pay for first ring (nothing)
(defrule goal-expander-goal-pay-first-ring-cc0
    ?g <- (goal (id ?goal-id) (class GOAL-PAY-FIRST-RING) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))

    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring1col ZERO))

    =>

    (modify ?g (mode EVALUATED) (outcome COMPLETED))
)

; expansion rule to pay for second ring (nothing)
(defrule goal-expander-goal-pay-second-ring-cc0
    ?g <- (goal (id ?goal-id) (class GOAL-PAY-SECOND-RING) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))

    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring2col ZERO))

    =>

    (modify ?g (mode EVALUATED) (outcome COMPLETED))
)

; expansion rule to pay for first ring (one payment)
(defrule goal-expander-goal-pay-first-ring-cc1
    ?g <- (goal (id ?goal-id) (class GOAL-PAY-FIRST-RING) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name mps-type) (param-values ?bs BS))

    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring1col ONE))
    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    

    =>
    (bind ?wp (sym-cat wp-1st-pay ?goal-id))
    (plan-assert-sequential PAY-FIRST-RING-CC1 ?goal-id ?robot
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

    (printout t "Robot " ?robot " was assigned PAY-FIRST-RING-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; expansion rule to pay for second ring (one payment) (ATTENTION: THIS SELECTS AN ARBITRARY CAP STATION, TODO: ONLY SELECT NON-LOCKED STATION!)
(defrule goal-expander-goal-pay-second-ring-cc1
    ?g <- (goal (id ?goal-id) (class GOAL-PAY-SECOND-RING) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (domain-fact (name mps-team) (param-values ?cs ?team-color))
    (domain-fact (name mps-type) (param-values ?cs CS))

    (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))

    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring2col ONE))
    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))


    =>
    (plan-assert-sequential PAY-SECOND-RING-CC1 ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs INPUT)
        (plan-assert-action wp-get-shelf ?robot ?wp ?cs ?spot)
        (plan-assert-action move ?robot ?cs INPUT ?rs INPUT)
        (plan-assert-action wp-put-slide-cc ?robot ?wp ?rs ?rs-before ?rs-after)
    )

    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned PAY-SECOND-RING-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; expansion rule to pay for first ring (two payments)
(defrule goal-expander-goal-pay-first-ring-cc2
    ?g <- (goal (id ?goal-id) (class GOAL-PAY-FIRST-RING) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name mps-type) (param-values ?bs BS))

    (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring1col TWO))
    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-inc) (param-values ?rs-before ?rs-after1))
    (domain-fact (name rs-inc) (param-values ?rs-after1 ?rs-after2))

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    

    =>
    (bind ?wp1 (sym-cat wp-1st-pay ?goal-id))
    (bind ?wp2 (sym-cat wp-2nd-pay ?goal-id))
    (plan-assert-sequential PAY-FIRST-RING-CC2 ?goal-id ?robot
        (plan-assert-action prepare-bs ?bs OUTPUT BASE_BLACK)
        (plan-assert-action spawn-wp ?wp1 ?robot)
        (plan-assert-action bs-dispense ?bs OUTPUT ?wp1 BASE_BLACK)

        (plan-assert-action move ?robot ?fl1 ?fs1 ?bs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp1 ?bs OUTPUT)
        (plan-assert-action move ?robot ?bs OUTPUT ?rs INPUT)
        (plan-assert-action wp-put-slide-cc ?robot ?wp1 ?rs ?rs-before ?rs-after1)

        (plan-assert-action prepare-bs ?bs OUTPUT BASE_BLACK)
        (plan-assert-action spawn-wp ?wp2 ?robot)
        (plan-assert-action bs-dispense ?bs OUTPUT ?wp2 BASE_BLACK)

        (plan-assert-action move ?robot ?rs INPUT ?bs OUTPUT)
        (plan-assert-action wp-get ?robot ?wp2 ?bs OUTPUT)
        (plan-assert-action move ?robot ?bs OUTPUT ?rs INPUT)
        (plan-assert-action wp-put-slide-cc ?robot ?wp2 ?rs ?rs-after1 ?rs-after2)
    )

    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned PAY-FIRST-RING-" ?ord crlf)
    (retract ?rw)  ;We must insert this again when done executing!
)

; expansion rule to pay for second ring (two payments) (ATTENTION: THIS SELECTS TWO ARBITRARY CAP STATIONS, TODO: ONLY SELECT NON-LOCKED STATIONS!)
(defrule goal-expander-goal-pay-second-ring-cc2
    ?g <- (goal (id ?goal-id) (class GOAL-PAY-SECOND-RING) (mode SELECTED) (parent ?parent))
    ?m <- (goal-meta (goal-id ?goal-id) (order-id ?ord))
    ?rw <- (domain-fact (name robot-waiting) (param-values ?robot))

    (domain-fact (name mps-team) (param-values ?cs1 ?team-color))
    (domain-fact (name mps-type) (param-values ?cs1 CS))

    (domain-fact (name mps-team) (param-values ?cs2 ?team-color))
    (domain-fact (name mps-type) (param-values ?cs2 CS))

    (domain-fact (name wp-on-shelf) (param-values ?wp1 ?cs1 ?spot1))
    (domain-fact (name wp-on-shelf) (param-values ?wp2 ?cs2 ?spot2))

    (not (eq ?wp1 ?wp2)) ; We need two separate shelf workpieces!

    (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?ring2col TWO))
    (domain-fact (name rs-filled-with) (param-values ?rs ?rs-before))
    (domain-fact (name rs-inc) (param-values ?rs-before ?rs-after1))
    (domain-fact (name rs-inc) (param-values ?rs-after1 ?rs-after2))

    (domain-fact (name at) (param-values ?robot ?fl1 ?fs1))
    

    =>
    (plan-assert-sequential PAY-SECOND-RING-CC2 ?goal-id ?robot
        (plan-assert-action move ?robot ?fl1 ?fs1 ?cs1 INPUT)
        (plan-assert-action wp-get-shelf ?robot ?wp1 ?cs1 ?spot1)
        (plan-assert-action move ?robot ?cs1 INPUT ?rs INPUT)
        (plan-assert-action wp-put-slide-cc ?robot ?wp1 ?rs ?rs-before ?rs-after1)

        (plan-assert-action move ?rs INPUT ?cs2 INPUT)
        (plan-assert-action wp-get-shelf ?robot ?wp2 ?cs2 ?spot2)
        (plan-assert-action move ?robot ?cs2 INPUT ?rs INPUT)
        (plan-assert-action wp-put-slide-cc ?robot ?wp2 ?rs ?rs-after1 ?rs-after2)
    )

    (modify ?g (mode EXPANDED))
    (modify ?m (assigned-to ?robot))

    (printout t "Robot " ?robot " was assigned PAY-SECOND-RING-" ?ord crlf)
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