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
                            (not (mps-state ?mps BROKEN))
                            (mps-team ?fs ?team-color)
                            (mps-side-free ?ds INPUT)

                            ;wp CEs
                            (wp-for-order ?wp ?order)
                            (wp-base-color ?wp ?base-color)
                            (wp-ring1-color ?wp ?ring1-color)
                            (wp-ring2-color ?wp ?ring2-color)
                            (wp-ring3-color ?wp ?ring3-color)
                            (wp-cap-color ?wp ?cap-color)
                            (wp-at ?wp ?fs OUTPUT)

                            ;order CEs
                            (order-gate ?order ?gate)
                            (order-deliverable ?order)
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
                            (order-deliverable ?order)
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

(defrule goal-class-order-deliverable
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
        (value ?qd&:(> ?qr ?qd)))
    (not (domain-fact (name order-deliverable) (param-values ?order)))
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
