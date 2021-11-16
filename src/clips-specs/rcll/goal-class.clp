;---------------------------------------------------------------------------
;  goal-classes.clp - Define the goal classes and their preconditions
;
;  Created: Tue 02 Nov 2021 19:05:00 CET
;  Copyright  2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;


; ------------------------- ASSERT GOAL CLASSES -----------------------------------

(defrule goal-class-create-mount-first-ring
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))

    (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&C1|C2|C3))
    (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
    (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring1-color rn ?bases-needed))
    (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?other-color rn ?other-bases))

    (test (not (eq ?ring1-color ?other-color)))
    (test (not (eq ?ring1-color RING_NONE)))
    (test (not (eq ?other-color RING_NONE)))
    (test (not (eq ?bases-needed NA)))
    (test (not (eq ?other-bases NA)))
    (not (goal-class (class MOUNT-FIRST-RING) (meta order ?order)))
    =>
    (assert
        (goal-class (class MOUNT-FIRST-RING)
                    (id (sym-cat MOUNT-FIRST-RING- ?order))
                    (meta order ?order)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     rs  bases-needed  other-color  ring1-color  bs  wp        side     order  robot  base-color)
                    (param-constants ?rs ?bases-needed ?other-color ?ring1-color nil nil       nil      ?order ?robot ?base-color)
                    (param-types     rs  ring-num      ring-color   ring-color   bs  workpiece mps-side order  robot  base-color)
                    (param-quantified )
                    (preconditions "
                        (and
                            (not (mps-state ?rs BROKEN))
                            (rs-paid-for ?rs ?bases-needed)
                            (mps-side-free ?rs INPUT)
                            (not
                                (and
                                    (rs-prepared-color ?rs ?other-color)
                                    (rs-prepared-color ?rs ?ring1-color)
                                )
                            )
                            (mps-has-side ?bs ?side)
                            (order-producible ?order)
                            (or
                                (and
                                    (not (order-has-wp ?order))
                                    (can-hold ?robot)
                                    (not (mps-state ?bs DOWN))
                                    (not (mps-state ?bs BROKEN))
                                    (wp-spawned-for ?wp ?robot)
                                )
                                (and
                                    (holding ?robot ?wp)
                                    (wp-base-color ?wp ?base-color)
                                    (wp-for-order ?wp ?order)
                                    (wp-ring1-color ?wp RING_NONE)
                                )
                            )
                            ;exclusivity
                        )

                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-mount-next-ring2
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact self args? r ?robot))

    (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&C2|C3))
    (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
    (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
    (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
    (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring2-color rn ?bases-needed))
    (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?other-color&~?ring2-color $?))

    (wm-fact (key domain fact rs-ring-spec args? m ?prev-rs r ?ring1-color $?))

    (test (neq ?ring1-color RING_NONE))
    (test (neq ?ring2-color RING_NONE))
    (test (neq ?other-color RING_NONE))
    (test (neq ?bases-needed NA))

    (not (goal-class (class MOUNT-NEXT-RING) (meta order ?order ring ring2)))
    =>
    (assert
        (goal-class (class MOUNT-NEXT-RING)
                    (id (sym-cat MOUNT-NEXT-RING-2- ?order))
                    (meta order ?order ring ring2)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     order  robot  wp        base-color  ring1-color  ring2-color  ring3-color  other-color  rs  prev-rs  bases-needed)
                    (param-constants ?order ?robot nil       ?base-color ?ring1-color ?ring2-color ?ring3-color ?other-color ?rs ?prev-rs ?bases-needed)
                    (param-types     order  robot  workpiece base-color  ring-color   ring-color   ring-color   ring-color   rs  rs       ring-num)
                    (param-quantified )
                    (preconditions "
                        (and
                            (order-producible ?order)
                            (wp-for-order ?wp ?order)
                            (wp-base-color ?wp ?base-color)
                            (wp-ring1-color ?wp ?ring1-color)
                            (wp-ring2-color ?wp RING_NONE)
                            (wp-ring3-color ?wp RING_NONE)
                            (wp-cap-color ?wp CAP_NONE)
                            (rs-paid-for ?rs ?bases-needed)

                            (not (mps-state ?rs BROKEN))
                            (mps-side-free ?rs INPUT)
                            (not
                                (and
                                    (rs-prepared-color ?rs ?ring2-color)
                                    (rs-prepared-color ?rs ?other-color)
                                )
                            )

                            (or
                                (and
                                    (can-hold ?robot)
                                    (wp-at ?wp ?prev-rs OUTPUT)
                                )
                                (and
                                    (holding ?robot ?wp)
                                    (mps-type ?prev-rs RS)
                                )
                            )
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-mount-next-ring3
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact self args? r ?robot))

    (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
    (wm-fact (key domain fact order-complexity args? ord ?order com C3))
    (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
    (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
    (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
    (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring3-color rn ?bases-needed))
    (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?other-color&~?ring3-color $?))

    (wm-fact (key domain fact rs-ring-spec args? m ?prev-rs r ?ring2-color $?))

    (test (neq ?ring1-color RING_NONE))
    (test (neq ?ring2-color RING_NONE))
    (test (neq ?ring3-color RING_NONE))
    (test (neq ?other-color RING_NONE))
    (test (neq ?bases-needed NA))

    (not (goal-class (class MOUNT-NEXT-RING) (meta order ?order ring ring3)))
    =>
    (assert
        (goal-class (class MOUNT-NEXT-RING)
                    (id (sym-cat MOUNT-NEXT-RING-3- ?order))
                    (meta order ?order ring ring3)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     order  robot  wp        base-color  ring1-color  ring2-color  ring3-color  other-color  rs  prev-rs  bases-needed)
                    (param-constants ?order ?robot nil       ?base-color ?ring1-color ?ring2-color ?ring3-color ?other-color ?rs ?prev-rs ?bases-needed)
                    (param-types     order  robot  workpiece base-color  ring-color   ring-color   ring-color   ring-color   rs  rs       ring-num)
                    (param-quantified )
                    (preconditions "
                        (and
                            (order-producible ?order)
                            (wp-for-order ?wp ?order)
                            (wp-base-color ?wp ?base-color)
                            (wp-ring1-color ?wp ?ring1-color)
                            (wp-ring2-color ?wp ?ring2-color)
                            (wp-ring3-color ?wp RING_NONE)
                            (wp-cap-color ?wp CAP_NONE)
                            (rs-paid-for ?rs ?bases-needed)

                            (not (mps-state ?rs BROKEN))
                            (mps-side-free ?rs INPUT)
                            (not
                                (and
                                    (rs-prepared-color ?rs ?ring3-color)
                                    (rs-prepared-color ?rs ?other-color)
                                )
                            )

                            (or
                                (and
                                    (can-hold ?robot)
                                    (wp-at ?wp ?prev-rs OUTPUT)
                                )
                                (and
                                    (holding ?robot ?wp)
                                    (mps-type ?prev-rs RS)
                                )
                            )
                        )
                    ")
                    (effects "")
        )
    )
)



(defrule goal-class-create-deliver
    "If there exists an order for a product of a certain configuration,
    assert a goal class fact for it that holds the preconditions for the formulation of
    its deliver goal. "
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?comp))
    (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
    (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
    (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
    (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
    (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
    (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))

    (not (goal-class (class DELIVER) (meta order ?order)))
    =>
    (assert
        (goal-class (class DELIVER)
                    (id (sym-cat DELIVER- ?order))
                    (meta order ?order)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     team-color  robot  ds  fs wp         base-color  ring1-color  ring2-color  ring3-color  cap-color  order  complexity             gate)
                    (param-constants ?team-color ?robot nil nil nil       ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color ?order ?comp                  ?gate)
                    (param-types     team-color  robot  ds  fs  workpiece base-color  ring-color   ring-color   ring-color   cap-color  order  order-complexity-value ds-gate)
                    (param-quantified )
                    (preconditions "
                        (and
                            ;mps CEs
                            (mps-team ?ds ?team-color)
                            (not (mps-state ?fs BROKEN))
                            (mps-team ?fs ?team-color)
                            (mps-side-free ?ds INPUT)

                            ;wp CEs
                            (wp-for-order ?wp ?order)
                            (wp-base-color ?wp ?base-color)
                            (wp-ring1-color ?wp ?ring1-color)
                            (wp-ring2-color ?wp ?ring2-color)
                            (wp-ring3-color ?wp ?ring3-color)
                            (wp-cap-color ?wp ?cap-color)

                            ;order CEs
                            (order-gate ?order ?gate)
                            (order-deliverable ?order)

                            ;positional CEs
                            (or
                                (and
                                    (wp-at ?wp ?fs OUTPUT)
                                    (can-hold ?robot)
                                )
                                (holding ?robot ?wp)
                            )
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-produce-c0
    "If there exists an order for a C0 product of a certain configuration,
    assert a goal class fact for it that holds the preconditions for the formulation of
    its production goal. "
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact order-complexity args? ord ?order com C0))
    (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
    (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
    (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))

    (not (goal-class (class PRODUCE-C0) (meta order ?order) (sub-type SIMPLE)))
    =>
    (assert
        (goal-class (class PRODUCE-C0)
                    (id (sym-cat PRODUCE-C0- ?order))
                    (meta order ?order)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     team-color  robot  cs wp         cap-color  bs  side     order  base-color)
                    (param-constants ?team-color ?robot ?cs nil       ?cap-color nil nil      ?order ?base-color)
                    (param-types     team-color  robot  cs  workpiece cap-color  bs  mps-side order  base-color)
                    (param-quantified)
                    (preconditions "
                        (and
                            ;cs CEs
                            (mps-side-free ?cs INPUT)
                            (not (mps-state ?cs BROKEN))
                            (mps-team ?cs ?team-color)
                            (cs-buffered ?cs ?cap-color)
                            (cs-can-perform ?cs MOUNT_CAP)
                            ;bs CEs
                            (mps-has-side ?bs ?side)
                            (mps-team ?bs ?team-color)
                            ;order CEs
                            (order-producible ?order)
                            (order-deliverable ?order)
                            ;wp CEs
                            (or
                                (and
                                    (wp-spawned-for ?wp ?robot)
                                    (not (mps-state ?bs BROKEN))
                                    (not (mps-state ?bs DOWN))
                                    (can-hold ?robot)
                                    (not (order-has-wp ?order))
                                )
                                (and
                                    (holding ?robot ?wp)
                                    (wp-base-color ?wp ?base-color)
                                    (wp-cap-color ?wp CAP_NONE)
                                    (wp-for-order ?wp ?order)
                                )
                            )
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-produce-cx
    "If there exists an order for a C1, C2, or C3 product of a certain configuration,
    assert a goal class fact for it that holds the preconditions for the formulation of
    its production goal. "
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?com&C1|C2|C3))
    (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
    (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
    (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
    (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
    (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
    (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))

    (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?ring1-color $?))
    (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?ring2-color $?))
    (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?ring3-color $?))

    (not (goal-class (class PRODUCE-CX) (meta order ?order) (sub-type SIMPLE)))
    =>
    (bind ?rs nil)
    (if (eq ?com C1) then
        (bind ?rs ?rs1)
    )
    (if (eq ?com C2) then
        (bind ?rs ?rs2)
    )
    (if (eq ?com C3) then
        (bind ?rs ?rs3)
    )
    (assert
        (goal-class (class PRODUCE-CX)
                    (id (sym-cat PRODUCE-CX- ?order))
                    (meta order ?order)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     team-color  robot  cs  order  base-color  ring1-color  ring2-color  ring3-color  cap-color  wp        rs)
                    (param-constants ?team-color ?robot ?cs ?order ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color nil       ?rs)
                    (param-types     team-color  robot  cs  order  base-color  ring-color   ring-color   ring-color   cap-color  workpiece rs)
                    (param-quantified)
                    (preconditions "
                        (and
                            ;cs CEs
                            (mps-side-free ?cs INPUT)
                            (mps-team ?cs ?team-color)
                            (not (mps-state ?cs BROKEN))
                            (cs-buffered ?cs ?cap-color)
                            (cs-can-perform ?cs MOUNT_CAP)
                            ;wp CEs
                            (wp-for-order ?wp ?order)
                            (wp-base-color ?wp ?base-color)
                            (wp-ring1-color ?wp ?ring1-color)
                            (wp-ring2-color ?wp ?ring2-color)
                            (wp-ring3-color ?wp ?ring3-color)
                            (wp-cap-color ?wp CAP_NONE)
                            (or
                                (and
                                    (mps-team ?rs ?team-color)
                                    (wp-at ?wp ?rs OUTPUT)
                                    (can-hold ?robot)
                                )
                                (holding ?robot ?wp)
                            )
                            ;order CEs
                            (order-producible ?order)
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-clear-bs
    "Assert a goal class for CLEAR-MPS goals that holds the precondition for formulation
    of potential BS clear goals."
    (not (goal-class (class CLEAR-MPS) (sub-type SIMPLE)))
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    =>
    (assert
        (goal-class (class CLEAR-MPS)
                    (id CLEAR-MPS)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     team-color  robot  mps wp        side)
                    (param-constants ?team-color ?robot nil nil       nil)
                    (param-types     team-color  robot  bs  workpiece mps-side)
                    (param-quantified)
                    (preconditions "
                        (and
                            (can-hold ?robot)

                            (mps-type ?mps BS)
                            (mps-team ?mps ?team-color)
                            (not (mps-state ?mps BROKEN))
                            (wp-at ?wp ?mps ?side)
                        )
                    ")
                    (effects "")
        )
    )
)

; ------------------------- ASSERT GOALS -----------------------------------

(defrule goal-class-assert-goal-mount-first-ring
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))

    (goal-class (class MOUNT-FIRST-RING) (id ?cid) (meta order ?order))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?rs ?bases-needed ?other-color ?ring1-color ?bs ?wp ?side ?order ?robot ?base-color))

    (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
    (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                            subtrahend ?bases-needed
                                            difference ?bases-remain&ZERO|ONE|TWO|THREE))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
    (not (and (wm-fact (key domain fact wp-for-order args? wp ?ord-wp&~?wp ord ?any-order))
            (wm-fact (key domain fact order-complexity args? ord ?any-order com ?other-complexity))
            (wm-fact (key config rcll exclusive-complexities) (values $?other-exclusive&:(member$ (str-cat ?other-complexity) ?other-exclusive)))
            (wm-fact (key config rcll exclusive-complexities) (values $?exclusive&:(member$ (str-cat ?complexity) ?exclusive)))))
    (or (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
        (allowed ?complexity)
    )

    (not (goal (class MOUNT-FIRST-RING)
                (parent ?production-id)
                (params robot ?robot $?
                        bs-side ?side $?
                        order ?order
                        wp ?wp)))
    =>
    (bind ?required-resources ?order ?wp)
    (if (any-factp ((?exclusive-complexities wm-fact))
            (and (wm-key-prefix ?exclusive-complexities:key (create$ config rcll exclusive-complexities))
                (neq FALSE (member$ (str-cat ?complexity) ?exclusive-complexities:values))))
        then
        (bind ?required-resources ?rs ?order ?wp PRODUCE-EXCLUSIVE-COMPLEXITY)
        (printout t "Goal " MOUNT-FIRST-RING " formulated from PDDL for order " ?order ", it needs the PRODUCE-EXCLUSIVE-COMPLEXITY token" crlf)
        else
        (printout t "Goal " MOUNT-FIRST-RING " formulated from PDDL for order " ?order crlf)
    )
    (bind ?distance (node-distance (str-cat ?bs - (if (eq ?side INPUT) then I else O))))

    (assert (goal (id (sym-cat MOUNT-FIRST-RING- (gensym*)))
                    (class MOUNT-FIRST-RING) (sub-type SIMPLE)
                    (priority (+ ?*PRIORITY-MOUNT-FIRST-RING* (goal-distance-prio ?distance)))
                    (parent ?production-id)
                    (params robot ?robot
                            bs ?bs
                            bs-side ?side
                            bs-color ?base-color
                            mps ?rs
                            ring-color ?ring1-color
                            rs-before ?bases-filled
                            rs-after ?bases-remain
                            rs-req ?bases-needed
                            order ?order
                            wp ?wp
                    )
                    (required-resources (sym-cat ?rs -INPUT) ?required-resources)
    ))
)

(defrule goal-class-assert-goal-mount-next-ring
    (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))

    (goal-class (class MOUNT-NEXT-RING) (id ?cid) (meta order ?order ring ?ring))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?order ?robot ?wp ?base-color ?ring1-color ?ring2-color ?ring3-color ?other-color ?rs ?prev-rs ?bases-needed))

    (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
    (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                            subtrahend ?bases-needed
                                            difference ?bases-remain&ZERO|ONE|TWO|THREE))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))


    (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
    (not (wm-fact (key strategy keep-mps-side-free args? m ?mps-rs side INPUT cause ~?wp)))
    (not (goal (class MOUNT-NEXT-RING) (parent ?maintain-id) (params robot ?robot $?
                                                                     wp ?wp $?
                                                                     order ?order)))
    =>
    (bind ?curr-ring-color ?ring2-color)
    (bind ?ring-pos 2)
    (if (eq ?ring ring3) then
        (bind ?curr-ring-color ?ring3-color)
        (bind ?ring-pos 3)
    )

    (printout t "Goal " MOUNT-NEXT-RING " formulated from PDDL for order " ?order " (Ring " ?ring-pos ") " crlf)
    (assert (goal (id (sym-cat MOUNT-NEXT-RING- (gensym*)))
                    (class MOUNT-NEXT-RING) (priority (+ ?ring-pos ?*PRIORITY-MOUNT-NEXT-RING*))
                    (parent ?production-id) (sub-type SIMPLE)
                    (params robot ?robot
                            prev-rs ?prev-rs
                            prev-rs-side OUTPUT
                            wp ?wp
                            rs ?rs
                            ring1-color ?ring1-color
                            ring2-color ?ring2-color
                            ring3-color ?ring3-color
                            curr-ring-color ?curr-ring-color
                            ring-pos (int-to-sym ?ring-pos)
                            rs-before ?bases-filled
                            rs-after ?bases-remain
                            rs-req ?bases-needed
                            order ?order
                    )
                    (required-resources (sym-cat ?rs -INPUT) (sym-cat ?prev-rs -OUTPUT) ?wp)
    ))
)

(defrule goal-class-assert-goal-deliver
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class DELIVER-PRODUCTS) (mode FORMULATED))

    (goal-class (class DELIVER) (id ?cid) (meta order ?order))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?ds ?mps ?wp ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color ?order ?complexity ?gate))

    (not (goal (class DELIVER) (params $? robot ?robot $?
                                          order ?order
                                          wp ?wp
                                          ds ?ds
                                          ds-gate ?gate $?)))
    =>
    (printout t "Goal " DELIVER " formulated from PDDL for order " ?order crlf)
    (assert (goal (id (sym-cat DELIVER- (gensym*)))
                    (class DELIVER) (sub-type SIMPLE)
                    (priority ?*PRIORITY-DELIVER*)
                    (parent ?production-id)
                    (params robot ?robot
                            mps ?mps
                            order ?order
                            wp ?wp
                            ds ?ds
                            ds-gate ?gate
                            base-color ?base-color
                            ring1-color ?ring1-color
                            ring2-color ?ring2-color
                            ring3-color ?ring3-color
                            cap-color ?cap-color
                    )
                    (required-resources (sym-cat ?mps -OUTPUT) ?order ?wp (sym-cat ?ds -INPUT))
    ))
)

(defrule goal-class-assert-goal-produce-c0
    "If the precondition of a goal-class for a C0 order is fulfilled, assert the goal fact
    and thus formulate the goal. Determine the priority of the goal through meta reasoning
    on additional facts."
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))
    (goal (id ?urgent) (class URGENT) (mode FORMULATED))

    (goal-class (class PRODUCE-C0) (id ?cid) (meta order ?order))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?mps ?wp ?cap-color ?bs ?side ?order ?base-color))

    (not (goal (class PRODUCE-C0) (params $? order ?order $?)))

    (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))
    (wm-fact (key config rcll competitive-order-priority) (value ?comp-prio))
    =>
    (printout t "Goal " PRODUCE-C0 " formulated from PDDL for order " ?order crlf)
    (bind ?distance (node-distance (str-cat ?bs - (if (eq ?side INPUT) then I else O))))
    (bind ?priority-decrease 0)
    (bind ?parent ?production-id)
    (if (and (eq ?comp-prio "HIGH") ?competitive)
    then
        (bind ?parent ?urgent))
    (if (eq ?comp-prio "LOW")
    then
        (bind ?priority-decrease 1)
    )
    (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
                    (class PRODUCE-C0) (sub-type SIMPLE)
                    (priority (+ (- ?*PRIORITY-PRODUCE-C0* ?priority-decrease) (goal-distance-prio ?distance)))
                    (parent ?parent)
                    (params robot ?robot
                            bs ?bs
                            bs-side ?side
                            bs-color ?base-color
                            mps ?mps
                            cs-color ?cap-color
                            order ?order
                            wp ?wp
                    )
                    (required-resources (sym-cat ?mps -INPUT) ?order ?wp)
    ))
)

(defrule goal-class-assert-goal-produce-cx
    "If the precondition of a goal-class for a CX order is fulfilled, assert the goal
    fact and thus formulate the goal. Determine the priority of the goal based on
    its complexity."
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))
    (goal (id ?urgent) (class URGENT) (mode FORMULATED))

    (goal-class (class PRODUCE-CX) (id ?cid) (meta order ?order))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?cs ?order ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color ?wp ?rs))

    (not (goal (class PRODUCE-CX)
                (parent ?production-id)
                (params robot ?robot
                        wp ?wp $?
                        mps ?cs $?
                        order ?order)))
    =>
    (bind ?prio ?*PRIORITY-PRODUCE-C1*)
    (if (eq ?com C2) then (bind ?prio ?*PRIORITY-PRODUCE-C2*))
    (if (eq ?com C3) then (bind ?prio ?*PRIORITY-PRODUCE-C3*))
    (printout t "Goal " PRODUCE-CX " formulated from PDDL for order " ?order crlf)
    (assert (goal (id (sym-cat PRODUCE-CX- (gensym*)))
                    (class PRODUCE-CX) (sub-type SIMPLE)
                    (priority ?prio)
                    (parent ?production-id)
                    (params robot ?robot
                            wp ?wp
                            rs ?rs
                            mps ?cs
                            cs-color ?cap-color
                            order ?order
                    )
                    (required-resources (sym-cat ?cs -INPUT) (sym-cat ?rs -OUTPUT) ?wp)
    ))
)

(defrule goal-class-assert-goal-clear-bs
    "If the precondition of a goal-class for a CLEAR-MPS type is fulfilled, assert
    the goal fact and thus formulate the goal. "
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class URGENT) (mode FORMULATED))

    (goal-class (class CLEAR-MPS) (id ?cid))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?mps ?wp ?side))
    =>
    (printout t "Goal " CLEAR-MPS " ("?mps") formulated from PDDL" crlf)
    (assert (goal (id (sym-cat CLEAR-MPS- (gensym*)))
                    (class CLEAR-MPS) (sub-type SIMPLE)
                    (priority ?*PRIORITY-CLEAR-BS*)
                    (parent ?production-id)
                    (params robot ?robot
                            mps ?mps
                            wp ?wp
                            side ?side
                    )
                    (required-resources (sym-cat ?mps - ?side) ?wp)
    ))
)

; ------------------------- CLEAN UP GOAL CLASSES -----------------------------------


; ------------------------- META CHECKS FOR GOALS -----------------------------------

(defrule goal-class-order-producible-C0
    (wm-fact (key domain fact order-complexity args? ord ?order com C0))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox game-time) (values $?game-time))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
        (value ?qd&:(> ?qr ?qd)))
    (wm-fact (key refbox order ?order delivery-begin) (type UINT)
        (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-AHEAD-TIME*))))
    (wm-fact (key refbox order ?order delivery-end) (type UINT)
        (value ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-LATEST-TIME*))))
    (not (domain-fact (name order-producible) (param-values ?order)))
    =>
    (assert (domain-fact (name order-producible) (param-values ?order)))
)

(defrule goal-class-order-not-producible-C0
    (wm-fact (key domain fact order-complexity args? ord ?order com C0))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox game-time) (values $?game-time))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (or
        (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
            (value ?qd&:(<= ?qr ?qd)))
        (wm-fact (key refbox order ?order delivery-end) (type UINT)
            (value ?end&:(< ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-LATEST-TIME*))))
    )
    ?wmf <- (domain-fact (name order-producible) (param-values ?order))
    =>
    (retract ?wmf)
)

(defrule goal-class-order-producible-CX
    (wm-fact (key domain fact order-complexity args? ord ?order com ~C0))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
        (value ?qd&:(> ?qr ?qd)))
    (not (domain-fact (name order-producible) (param-values ?order)))
    =>
    (assert (domain-fact (name order-producible) (param-values ?order)))
)

(defrule goal-class-order-not-producible-CX
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
        (value ?qd&:(<= ?qr ?qd)))
    ?wmf <- (domain-fact (name order-producible) (param-values ?order))
    =>
    (retract ?wmf)
)

(defrule goal-class-order-deliverable
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
        (value ?qd&:(> ?qr ?qd)))
    (not (domain-fact (name order-deliverable) (param-values ?order)))
    (wm-fact (key refbox game-time) (values $?game-time))
    (wm-fact (key refbox order ?order delivery-begin) (type UINT)
             (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*))))
    =>
    (assert (domain-fact (name order-deliverable) (param-values ?order)))
)

(defrule goal-class-order-not-deliverable
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
        (value ?qd&:(<= ?qr ?qd)))
    ?wmf <- (domain-fact (name order-deliverable) (param-values ?order))
    =>
    (retract ?wmf)
)
