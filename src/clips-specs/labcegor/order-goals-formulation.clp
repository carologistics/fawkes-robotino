
; Assert goals
;----------------------------------------------------------------------------

(deffunction assert-payment-goals (?goal-id-payring ?cost ?ord ?goal-id-root)

    (printout t "Calculated cost of " ?cost " for " ?goal-id-payring crlf)
    (if (eq ?cost ZERO)
        then
        (bind ?goal-id (sym-cat GOAL-NOOP- ?goal-id-payring - (gensym*)))
        (assert (goal (class GOAL-NOOP)
                    (id ?goal-id)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (parent ?goal-id-payring)
                    (verbosity NOISY)
                    (priority 1.0) 
                    (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?goal-id-root)))
    )
    (if (or (eq ?cost ONE) (eq ?cost TWO))
        then 
        (bind ?goal-id1 (sym-cat GOAL-DO-PAYMENT1- ?goal-id-payring - (gensym*)))
        (assert (goal (class GOAL-DO-PAYMENT)
                    (id ?goal-id1)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (parent ?goal-id-payring)
                    (verbosity NOISY)
                    (priority 1.0) 
                    (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id1) (order-id ?ord) (root-for-order ?goal-id-root)))
        (if (eq ?cost TWO)
            then 
            (bind ?goal-id2 (sym-cat GOAL-DO-PAYMENT2- ?goal-id-payring - (gensym*)))
            (assert (goal (class GOAL-DO-PAYMENT)
                        (id ?goal-id2)
                        (type ACHIEVE)
                        (sub-type SIMPLE)
                        (parent ?goal-id-payring)
                        (verbosity NOISY)
                        (priority 2.0) 
                        (is-executable TRUE)
                        (meta-template goal-meta)
            ))
            (assert (goal-meta (goal-id ?goal-id2) (order-id ?ord) (root-for-order ?goal-id-root)))
        )
    )
)

(defrule assign-order
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (not (and (domain-object (name ?robot) (type robot)) (not (domain-fact (name entered-field) (param-values ?robot)))))
    
    (domain-fact (name order-complexity) (param-values ?ord ?complexity))
    (not (domain-fact (name order-fulfilled) (param-values ?ord)))

    (not (goal-meta (order-id ?ord)))

    (domain-fact (name order-ring1-color ) (param-values ?ord ?r1col))
    (domain-fact (name rs-ring-spec) (param-values ? ?r1col ?r1cost))

    (domain-fact (name order-ring2-color ) (param-values ?ord ?r2col))
    (domain-fact (name rs-ring-spec) (param-values ? ?r2col ?r2cost))

    (domain-fact (name order-ring3-color ) (param-values ?ord ?r3col))
    (domain-fact (name rs-ring-spec) (param-values ? ?r3col ?r3cost))

    =>
    (printout t "Building " ?complexity "-Tree ..." crlf)

    (bind ?goal-id-root (sym-cat GOAL-ORDER- ?complexity - ?ord ))
    (bind ?goal-id-tocs (sym-cat GOAL-TO-CS- ?ord ))
    (bind ?goal-id-csds (sym-cat GOAL-DELIVER- ?complexity - ?ord ))
    (bind ?goal-id-bs (sym-cat GOAL-GET-BS- ?ord ))
    (bind ?goal-id-cs (sym-cat GOAL-GET-CS- ?ord ))

    ; Root Goal
    (assert (goal (class (sym-cat GOAL-ORDER- ?complexity))                     
                (id ?goal-id-root)
                (type ACHIEVE)
                (sub-type CENTRAL-RUN-LINEAR)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-root) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal to CS
    (assert (goal (class GOAL-TO-CS)
                (id ?goal-id-tocs)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 6.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-tocs) (order-id ?ord) (root-for-order ?goal-id-root)))

    ; Goal CS -> DS
    (assert (goal (class (sym-cat GOAL-DELIVER- ?complexity))
                (id ?goal-id-csds)
                (type ACHIEVE)
                (sub-type SIMPLE)
                (parent ?goal-id-root)
                (verbosity NOISY) (priority 7.0) (is-executable TRUE)
                (meta-template goal-meta)
    ))
    (assert (goal-meta (goal-id ?goal-id-csds) (order-id ?ord) (root-for-order ?goal-id-root)))

    (if (eq ?complexity C0)
        then
        (bind ?goal-id-parallel (sym-cat GOAL-PARALLEL-BS-CS- ?ord ))

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

    )


    (if (eq ?complexity C1) 
        then
        (bind ?goal-id-parallel1 (sym-cat GOAL-PARALLEL-TO-RS-AND-GET-CS- ?ord ))

        ; Parallel part goal
        (assert (goal (class GOAL-PARALLEL-TO-RS-AND-GET-CS)
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

        ; Goal get CS
        (assert (goal (class GOAL-GET-CS)
                    (id ?goal-id-cs)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (parent ?goal-id-parallel1)
                    (verbosity NOISY) (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id-cs) (order-id ?ord) (root-for-order ?goal-id-root)))
    )

    (if (eq ?complexity C2)
        then
        (bind ?goal-id-parallel2 (sym-cat GOAL-PARALLEL-TO-RS2-AND-GET-CS- ?ord ))

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
    )

    (if (eq ?complexity C3)
        then
        (bind ?goal-id-parallel2 (sym-cat GOAL-PARALLEL-TO-RS2-AND-PREPARE-RS3- ?ord))
        (bind ?goal-id-payring3 (sym-cat GOAL-PAY-RING3- ?ord ))
        (bind ?goal-id-parallel3 (sym-cat GOAL-PARALLEL-TO-RS3-AND-GET-CS- ?ord ))
        (bind ?goal-id-tors3 (sym-cat GOAL-TO-RS3- ?ord ))

        ; Parallel part goal 2
        (assert (goal (class GOAL-PARALLEL-TO-RS2-AND-PREPARE-RS3)
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

        ; Goal Pay Third Ring
        (assert (goal (class GOAL-PAY-RING3)
                    (id ?goal-id-payring3)
                    (type ACHIEVE)
                    (sub-type CENTRAL-RUN-LINEAR)
                    (parent ?goal-id-parallel2)
                    (verbosity NOISY)
                    (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id-payring3) (order-id ?ord) (root-for-order ?goal-id-root)))
        (assert-payment-goals ?goal-id-payring3 ?r3cost ?ord ?goal-id-root)

        ; Parallel part goal 3
        (assert (goal (class GOAL-PARALLEL-TO-RS3-AND-GET-CS)
                    (id ?goal-id-parallel3)
                    (type ACHIEVE)
                    (sub-type CENTRAL-RUN-PARALLEL)
                    (parent ?goal-id-root)
                    (verbosity NOISY)
                    (priority 5.0)
                    (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id-parallel3) (order-id ?ord) (root-for-order ?goal-id-root)))

        ; Goal TO-RS3
        (assert (goal (class GOAL-TO-RS3)
                    (id ?goal-id-tors3)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (parent ?goal-id-parallel3)
                    (verbosity NOISY) (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id-tors3) (order-id ?ord) (root-for-order ?goal-id-root)))

        ; Goal get CS
        (assert (goal (class GOAL-GET-CS)
                    (id ?goal-id-cs)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (parent ?goal-id-parallel3)
                    (verbosity NOISY) (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id-cs) (order-id ?ord) (root-for-order ?goal-id-root)))
    )

    (if (or (eq ?complexity C2) (eq ?complexity C3))
        then
        (bind ?goal-id-parallel1 (sym-cat GOAL-PARALLEL-TO-RS1-AND-PREPARE-RS2- ?ord))
        (bind ?goal-id-payring2 (sym-cat GOAL-PAY-RING2- ?ord ))
        (bind ?goal-id-tors2 (sym-cat GOAL-TO-RS2- ?ord ))

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

        ; Goal Pay Second Ring
        (assert (goal (class GOAL-PAY-RING2)
                    (id ?goal-id-payring2)
                    (type ACHIEVE)
                    (sub-type CENTRAL-RUN-LINEAR)
                    (parent ?goal-id-parallel1)
                    (verbosity NOISY)
                    (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id-payring2) (order-id ?ord) (root-for-order ?goal-id-root)))
        (assert-payment-goals ?goal-id-payring2 ?r2cost ?ord ?goal-id-root)

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
    )

    (if (or (eq ?complexity C1) (eq ?complexity C2) (eq ?complexity C3))
        then
        (bind ?goal-id-payring1 (sym-cat GOAL-PAY-RING1- ?ord ))
        (bind ?goal-id-tors1 (sym-cat GOAL-TO-RS1- ?ord ))

        ; Goal Pay RING1
        (bind ?goal-id-payring (sym-cat GOAL-PAY-RING1- ?ord ))
        (assert (goal (class GOAL-PAY-RING1)
                    (id ?goal-id-payring1)
                    (type ACHIEVE)
                    (sub-type CENTRAL-RUN-LINEAR)
                    (parent ?goal-id-root)
                    (verbosity NOISY) 
                    (priority 1.0)
                    (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id-payring1) (order-id ?ord) (root-for-order ?goal-id-root)))
        (assert-payment-goals ?goal-id-payring1 ?r1cost ?ord ?goal-id-root)
        
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

        ; Goal TO-RS
        (assert (goal (class GOAL-TO-RS1)
                    (id ?goal-id-tors1)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (parent ?goal-id-parallel1)
                    (verbosity NOISY) (is-executable TRUE)
                    (meta-template goal-meta)
        ))
        (assert (goal-meta (goal-id ?goal-id-tors1) (order-id ?ord) (root-for-order ?goal-id-root)))
    )
)

