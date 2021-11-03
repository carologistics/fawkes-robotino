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
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    ; (wm-fact (key domain fact order-complexity args? ord ?order com ?comp))
    ; (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
    ; (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
    ; (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
    ; (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
    ; (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
    ; (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))

    (not (goal-class (class DELIVER))) ;(meta order ?order)))
    =>
    (assert
        (goal-class (class DELIVER)
                    (id DELIVER)
                    ;(id (sym-cat DELIVER- ?order))
                    ;(meta order ?order)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names team-color robot ds mps wp base-color ring1-color ring2-color ring3-color cap-color any-wp order complexity gate)
                    ;(param-constants ?team-color ?robot nil nil nil ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color nil ?order ?comp ?gate)
                    (param-constants ?team-color ?robot nil nil nil nil nil nil nil nil nil nil nil nil)
                    (param-types team-color robot ds fs workpiece base-color ring-color ring-color ring-color cap-color workpiece order order-complexity-value ds-gate)
                    (param-quantified any-wp)
                    (preconditions "
                        (and
                            (refbox-team-color ?team-color)
                            (self ?robot)
                            (mps-type ?ds DS)
                            (mps-team ?ds ?team-color)
                            (or
                                (and
                                    (mps-type ?mps CS)
                                    (not (mps-state ?mps BROKEN))
                                    (mps-team ?mps ?team-color)
                                )
                                (and
                                    (mps-type ?mps SS)
                                    (not (mps-state ?mps BROKEN))
                                    (mps-team ?mps ?team-color)
                                )
                            )
                            (wp-base-color ?wp ?base-color)
                            (wp-ring1-color ?wp ?ring1-color)
                            (wp-ring2-color ?wp ?ring2-color)
                            (wp-ring3-color ?wp ?ring3-color)
                            (wp-cap-color ?wp ?cap-color)
                            (not (exists (?any-wp - workpiece) (wp-at ?any-wp ?ds INPUT)))
                            (wp-for-order ?wp ?order)
                            (order-complexity ?order ?complexity)
                            (order-base-color ?order ?base-color)
                            (order-ring1-color ?order ?ring1-color)
                            (order-ring2-color ?order ?ring2-color)
                            (order-ring3-color ?order ?ring3-color)
                            (order-cap-color ?order ?cap-color)
                            (order-gate ?order ?gate)
                            (order-deliverable ?order)
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-produce-c0
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    ; (wm-fact (key domain fact order-complexity args? ord ?order com C0))
    ; (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
    ; (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

    (not (goal-class (class PRODUCE-C0))) ;(meta order ?order) (sub-type SIMPLE)))
    =>
    (assert
        (goal-class (class PRODUCE-C0)
                    (id PRODUCE-C0)
                    ;(id (sym-cat PRODUCE-C0- ?order))
                    ;(meta order ?order)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names team-color robot any-wp mps wp cap-color bs side order base-color any-hold-wp any-ord-wp)
                    ;(param-constants ?team-color ?robot nil nil nil ?cap-color nil nil ?order ?base-color nil nil)
                    (param-constants ?team-color ?robot nil nil nil nil nil nil nil nil nil nil)
                    (param-types team-color robot workpiece cs workpiece cap-color bs mps-side order base-color workpiece workpiece)
                    (param-quantified any-wp any-hold-wp any-ord-wp)
                    (preconditions "
                        (and
                            (refbox-team-color ?team-color)
                            (self ?robot)
                            (mps-type ?mps CS)
                            (not (exists (?any-wp - workpiece) (wp-at ?any-wp ?mps INPUT)))
                            (not (mps-state ?mps BROKEN))
                            (mps-team ?mps ?team-color)
                            (cs-buffered ?mps ?cap-color)
                            (cs-can-perform ?mps MOUNT_CAP)
                            (mps-type ?bs BS)
                            (mps-side-free ?bs ?side);redo
                            (mps-team ?bs ?team-color)
                            (order-complexity ?order C0)
                            (order-base-color ?order ?base-color)
                            (order-cap-color ?order ?cap-color)
                            (order-producible ?order)
                            (or
                                (and
                                    (refbox-order-quantity-requested ?order 1)
                                    (not (refbox-order-quantity-delivered ?order 1))
                                )
                                (and;
                                    (refbox-order-quantity-requested ?order 2)
                                    (not (refbox-order-quantity-delivered ?order 2))
                                )
                            )
                            (or
                                (and
                                    (wp-spawned-for ?wp ?robot)
                                    (not (mps-state ?bs BROKEN))
                                    (not (mps-state ?bs DOWN))
                                    (not (exists (?any-hold-wp - workpiece) (holding ?robot ?any-hold-wp)))
                                    (not (exists (?any-ord-wp - workpiece) (wp-for-order ?any-ord-wp ?order)))
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

(defrule goal-class-create-clear-bs
    (not (goal-class (class CLEAR-MPS) (sub-type SIMPLE)))
    (wm-fact (key domain fact self args? r ?robot))
    =>
    (assert
        (goal-class (class CLEAR-MPS)
                    (id CLEAR-MPS)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names team-color robot any-wp mps wp side)
                    (param-constants nil ?robot nil nil nil nil)
                    (param-types team-color robot workpiece bs workpiece mps-side)
                    (param-quantified any-wp)
                    (preconditions "
                        (and
                            (refbox-team-color ?team-color)
                            (self ?robot)
                            (not (exists (?any-wp - workpiece) (holding ?robot ?any-wp)))
                            (mps-type ?mps BS)
                            (mps-team ?mps ?team-color)
                            (not (mps-state ?mps BROKEN))
                            (wp-at ?wp ?mps ?side)
                        )
                    ");"
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
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?ds ?mps ?wp ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color ?any-wp ?order ?complexity ?gate))

    (not (goal (class DELIVER) (params $? robot ?robot $?
                                          order ?order
                                          wp ?wp
                                          ds ?ds
                                          ds-gate ?gate $?)))
    =>
    (printout t "PDDL: Goal " DELIVER " formulated from PDDL for order " ?order crlf)
    (printout t ?robot ?mps ?order ?wp ?ds ?gate ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color crlf)
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
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))
    (goal (id ?urgent) (class URGENT) (mode FORMULATED))

    (goal-class (class PRODUCE-C0) (id ?cid) (meta order ?order))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?any-wp ?mps ?wp ?cap-color ?bs ?side ?order ?base-color ?any-hold-wp ?any-ord-wp))

    (not (goal (class PRODUCE-C0) (params $? order ?order $?)))
    =>
    (printout t "PDDL: Goal " PRODUCE-C0 " formulated from PDDL for order " ?order crlf)
    (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
                    (class PRODUCE-C0) (sub-type SIMPLE)
                    (priority ?*PRIORITY-PRODUCE-C0*)
                    (parent ?production-id)
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

(defrule goal-class-assert-goal-clear-bs
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class URGENT) (mode FORMULATED))

    (goal-class (class CLEAR-MPS) (id ?cid))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?any-wp ?mps ?wp ?side))
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


