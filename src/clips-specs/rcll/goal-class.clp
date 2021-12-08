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

; MPS INTERACTION GOALS ARE KEPT AT OLD STATE FORE NOW


; CLEANUP GOALS

(defrule goal-class-create-clear-rs-from-expired-product
    "Assert a goal class for CLEAR-MPS goals that holds the precondition for formulation
    in case an expired product blocks a RS."
    (not (goal-class (class CLEAR-MPS) (id CLEAR-MPS-RS) (sub-type SIMPLE)))
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact order-complexity args? ord ?order $?))
    =>
    (assert
        (goal-class (class CLEAR-MPS)
                    (id (sym-cat CLEAR-MPS-RS - ?order))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta order ?order)
                    (param-names     team-color  robot  rs  wp          side   order)
                    (param-constants ?team-color ?robot nil nil         OUTPUT ?order)
                    (param-types     team-color  robot  rs  workpiece mps-side order)
                    (param-quantified)
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (can-hold ?robot)
                            (not (mps-state ?rs BROKEN))
                            (wp-at ?wp ?rs OUTPUT)
                            (wp-cap-color ?wp CAP_NONE)
                            (wp-for-order ?wp ?order)
                            (order-out-of-delivery ?order)
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-clear-cs-from-capless-carrier
    "Assert a goal class for CLEAR-MPS goals that holds the precondition for formulation
    in case a CC blocks a CS."
    (not (goal-class (class CLEAR-MPS) (id CLEAR-MPS-CS-CC) (sub-type SIMPLE)))
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    =>
    (assert
        (goal-class (class CLEAR-MPS)
                    (id CLEAR-MPS-CS-CC)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     team-color  robot  cs  cc          side)
                    (param-constants ?team-color ?robot nil nil         OUTPUT)
                    (param-types     team-color  robot  cs  cap-carrier mps-side)
                    (param-quantified)
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (can-hold ?robot)
                            (not (mps-state ?cs BROKEN))
                            (wp-at ?cc ?cs OUTPUT)
                            (wp-cap-color ?cc CAP_NONE)
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-clear-cs-from-finished-product
    "Assert a goal class for CLEAR-MPS goals that holds the precondition for formulation
    in case a finished product blocks a CS."
    (not (goal-class (class CLEAR-MPS) (id CLEAR-MPS-CS-WP) (sub-type SIMPLE)))
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    =>
    (assert
        (goal-class (class CLEAR-MPS)
                    (id CLEAR-MPS-CS-WP)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     team-color  robot  cs  wp        side)
                    (param-constants ?team-color ?robot nil nil       OUTPUT)
                    (param-types     team-color  robot  cs  workpiece mps-side)
                    (param-quantified)
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (can-hold ?robot)
                            (not (mps-side-free ?cs INPUT))
                            (not (mps-state ?cs BROKEN))
                            (wp-at ?wp ?cs OUTPUT)
                            (not (wp-cap-color ?wp CAP_NONE))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-clear-bs
    "Assert a goal class for CLEAR-MPS goals that holds the precondition for formulation
    of potential BS clear goals."
    (not (goal-class (class CLEAR-MPS) (id CLEAR-MPS-BS) (sub-type SIMPLE)))
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    =>
    (assert
        (goal-class (class CLEAR-MPS)
                    (id CLEAR-MPS-BS)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     team-color  robot  bs  wp        side)
                    (param-constants ?team-color ?robot nil nil       nil)
                    (param-types     team-color  robot  bs  workpiece mps-side)
                    (param-quantified)
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (can-hold ?robot)
                            (mps-team ?bs ?team-color)
                            (not (mps-state ?bs BROKEN))
                            (wp-at ?wp ?bs ?side)
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-discard-wp
    "Create a goal-class for discarding workpieces."
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))

    (not (goal-class (class DISCARD-UNKNOWN) (id DISCARD-UNKNOWN-WP)))
    =>
    (assert
        (goal-class (class DISCARD-UNKNOWN)
                    (id DISCARD-UNKNOWN-WP)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     robot  wp        rs)
                    (param-constants ?robot nil       nil)
                    (param-types     robot  workpiece rs)
                    (param-quantified )
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (holding ?robot ?wp)
                            (or
                                (rs-failed-put-slide ?rs ?robot ?wp)
                                (or
                                    (not (wp-has-order ?wp))
                                    (and
                                        (wp-has-order ?wp)
                                        (wp-cap-color ?wp CAP_NONE)
                                        (wp-ring1-color ?wp RING_NONE)
                                    )
                                )
                            )
                        )
                    ")
        )
    )
)

(defrule goal-class-create-discard-cc
    "Create a goal-class for discarding cap-carriers."
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))

    (not (goal-class (class DISCARD-UNKNOWN) (id DISCARD-UNKNOWN-CC)))
    =>
    (assert
        (goal-class (class DISCARD-UNKNOWN)
                    (id DISCARD-UNKNOWN-CC)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     robot  wp        rs)
                    (param-constants ?robot nil       nil)
                    (param-types     robot  cap-carrier rs)
                    (param-quantified )
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (holding ?robot ?wp)
                            (or
                                (rs-failed-put-slide ?rs ?robot ?wp)
                                (or
                                    (not (wp-has-order ?wp))
                                    (and
                                        (wp-has-order ?wp)
                                        (wp-cap-color ?wp CAP_NONE)
                                        (wp-ring1-color ?wp RING_NONE)
                                    )
                                )
                            )
                        )
                    ")
        )
    )
)



; PRODUCTION MAINTENANCE GOALS

(defrule goal-class-create-get-from-bs-for-rs
    "Create a goal-class for getting WPs from the BS to fill an RS with."
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))

    (not (goal-class (class GET-BASE-TO-FILL-RS)))
    =>
    (assert
        (goal-class (class GET-BASE-TO-FILL-RS)
                    (id GET-BASE-TO-FILL-RS)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     robot  wp        rs  bs  side)
                    (param-constants ?robot nil       nil nil nil)
                    (param-types     robot  workpiece rs  bs  mps-side)
                    (param-quantified )
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (can-hold ?robot)
                            (wp-spawned-for ?wp ?robot)
                            (or
                                (rs-filled-with ?rs ZERO)
                                (rs-filled-with ?rs ONE)
                                (rs-filled-with ?rs TWO)
                            )
                            (not (mps-state ?bs BROKEN))
                            (not (mps-state ?bs DOWN))
                            (mps-has-side ?bs ?side)
                        )
                    ")
        )
    )
)

(defrule goal-class-create-get-from-shelf-for-rs
    "Create a goal-class for getting CCs from the shelf to fill an RS with."
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))

    (not (goal-class (class GET-SHELF-TO-FILL-RS)))
    =>
    (assert
        (goal-class (class GET-SHELF-TO-FILL-RS)
                    (id GET-SHELF-TO-FILL-RS)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     robot  rs  cc          cs  spot)
                    (param-constants ?robot nil nil         nil nil)
                    (param-types     robot  rs  cap-carrier cs  shelf-spot)
                    (param-quantified )
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (can-hold ?robot)
                            (not (mps-state ?rs BROKEN))
                            (or
                                (rs-filled-with ?rs ZERO)
                                (rs-filled-with ?rs ONE)
                                (rs-filled-with ?rs TWO)
                            )
                            (wp-on-shelf ?cc ?cs ?spot)
                        )
                    ")
        )
    )
)

(defrule goal-class-create-fill-rs
    "Create a goal-class for an RS to formulate FILL-RS goals to fill it with WPs."
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
    (wm-fact (key domain fact mps-type args? m ?rs t RS))

    (not (goal-class (class FILL-RS) (meta rs ?rs cc FALSE)))
    =>
    (assert
        (goal-class (class FILL-RS)
                    (id (sym-cat FILL-RS-WP- ?rs))
                    (meta rs ?rs cc FALSE)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     wp        robot  rs  filled)
                    (param-constants nil       ?robot ?rs nil)
                    (param-types     workpiece robot  rs  ring-num)
                    (param-quantified )
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (wp-usable ?wp)
                            (holding ?robot ?wp)
                            (not (wp-has-order ?wp))
                            (not (mps-state ?rs BROKEN))
                            (rs-filled-with ?rs ?filled)
                            (or
                                (rs-filled-with ?rs ZERO)
                                (rs-filled-with ?rs ONE)
                                (rs-filled-with ?rs TWO)
                            )
                        )
                    ")
        )
    )
)

(defrule goal-class-create-fill-rs-cc
    "Create a goal-class for an RS to formulate FILL-RS goals to fill it with CCs."
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
    (wm-fact (key domain fact mps-type args? m ?rs t RS))

    (not (goal-class (class FILL-RS) (meta rs ?rs cc TRUE)))
    =>
    (assert
        (goal-class (class FILL-RS)
                    (id (sym-cat FILL-RS-CC- ?rs))
                    (meta rs ?rs cc TRUE)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     wp        robot  rs  filled)
                    (param-constants nil       ?robot ?rs nil)
                    (param-types     cap-carrier robot  rs  ring-num)
                    (param-quantified )
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (wp-usable ?wp)
                            (holding ?robot ?wp)
                            (not (mps-state ?rs BROKEN))
                            (rs-filled-with ?rs ?filled)
                            (or
                                (rs-filled-with ?rs ZERO)
                                (rs-filled-with ?rs ONE)
                                (rs-filled-with ?rs TWO)
                            )
                        )
                    ")
        )
    )
)

(defrule goal-class-create-fill-cap
    "Assert a FILL-CAP goal class for each CS based on its assigned cs-color."
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))

    (not (goal-class (class FILL-CAP) (meta cs ?cs)))
    =>
    (assert
        (goal-class (class FILL-CAP)
                    (id (sym-cat FILL-CAP- ?cs))
                    (meta cs ?cs)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     robot  cs  cc          spot       cap-color)
                    (param-constants ?robot ?cs nil         nil        ?cap-color)
                    (param-types     robot  cs  cap-carrier shelf-spot cap-color)
                    (param-quantified )
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (can-hold ?robot)
                            (not (mps-state ?cs BROKEN))
                            (cs-can-perform ?cs RETRIEVE_CAP)
                            (not (cs-buffered ?cs CAP_BLACK))
                            (not (cs-buffered ?cs CAP_GREY))
                            (mps-side-free ?cs INPUT)
                            (wp-on-shelf ?cc ?cs ?spot)
                            (wp-cap-color ?cc ?cap-color)
                        )
                    ")
        )
    )
)

; MAINLINE PRODUCTION GOALS

(defrule goal-class-create-mount-first-ring
    "If there exists an order for a product of complexity C1, C2 or C3, assert the
    MFR goal-class for this order and the mounting operation of the first ring."
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
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (not (mps-state ?rs BROKEN))
                            (rs-paid-for ?rs ?bases-needed)
                            (mps-side-free ?rs INPUT)
                            (not
                                (or
                                    (rs-prepared-color ?rs ?other-color)
                                    (rs-prepared-color ?rs ?ring1-color)
                                )
                            )
                            ;missing deadlock prevention
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
                        )

                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-mount-next-ring2
    "If there exists an order for a product of complexity C2 or C3, assert the MNR goal-class
    for this order and the mounting operation of the second ring."
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
                    (lookahead-time 0)
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
                                (or
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
    "If there exists an order for a product of complexity C3, assert the MNR goal-class
    for this order and the mounting operation of the third ring."
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
                    (lookahead-time 0)
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
                                (or
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
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            ;mps CEs
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
                                    (not (mps-state ?fs BROKEN))
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
                    (lookahead-time 0)
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
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            ;cs CEs
                            (mps-side-free ?cs INPUT)
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


; ------------------------- ASSERT GOALS -----------------------------------

; MPS INTERACTION GOALS - KEPT AT OLD STATE FOR NOW


; CLEANUP GOALS

(defrule goal-class-assert-goal-clear-mps
    "If the precondition of a goal-class for a CLEAR-MPS type is fulfilled, assert
    the goal fact and thus formulate the goal. "
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (or
        (goal (class URGENT) (mode FORMULATED))
        (goal (class CLEAR) (mode FORMULATED))

    )
    (goal-class (class ?class&CLEAR-MPS) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?mps ?wp ?side))

    (wm-fact (key domain fact mps-type args? m ?mps t ?mps-type))
    =>
    (printout t "Goal " CLEAR-MPS " ("?mps") formulated from PDDL" crlf)

    (bind ?parent nil)
    (bind ?priority nil)

    (if (eq ?mps-type CS)
        then
        (do-for-fact ((?goal goal)) (and (eq ?goal:class CLEAR) (eq ?goal:mode FORMULATED))
            (bind ?parent ?goal:id)
            (bind ?priority ?*PRIORITY-CLEAR-CS*)
            (if (and (any-factp ((?wm wm-fact)) (and (wm-key-prefix ?wm:key (create$ domain fact wp-at))
                                                (eq (wm-key-arg ?wm:key m) ?mps)
                                                (eq (wm-key-arg ?wm:key side) INPUT)))
                (eq ?cid CLEAR-MPS-CS-CC))
                then
                (bind ?priority (+ 1 ?priority))
                (printout warn "Enhance CLEAR-MPS priority, since there is a product at the input already" crlf)
            )
        )
        else
        (if (eq ?mps-type BS) then
            (do-for-fact ((?goal goal)) (and (eq ?goal:class URGENT) (eq ?goal:mode FORMULATED))
                (bind ?parent ?goal:id)
                (bind ?priority ?*PRIORITY-CLEAR-BS*)
            )
            else
            (if (eq ?mps-type RS) then
                (do-for-fact ((?goal goal)) (and (eq ?goal:class CLEAR) (eq ?goal:mode FORMULATED))
                    (bind ?parent ?goal:id)
                    (bind ?priority ?*PRIORITY-CLEAR-RS*)
                )
            )
        )
    )

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (priority ?priority)
                    (parent ?parent)
                    (params robot ?robot
                            mps ?mps
                            wp ?wp
                            side ?side
                    )
                    (required-resources (sym-cat ?mps - ?side) ?wp)
    ))

    ;assert promises resulting from the plan-action of this goal
    (assert
        ;wp-get
        (domain-promise (name wp-at) (param-values ?wp ?mps ?side) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name mps-state) (param-values ?mps READY-AT-OUTPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name mps-state) (param-values ?mps IDLE) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name mps-side-free) (param-values ?mps ?side) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot ?mps ?side) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name at) (param-values ?robot ?mps (wait-pos ?mps ?side)) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
    )
)

(defrule goal-class-assert-goal-discard
    "If the preconditions of a DISCARD-UNKNOWN goal class are satisfied, assert the goal."
    (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))
    (goal (id ?parent) (class NO-PROGRESS) (mode FORMULATED))
    (goal (id ?urgent) (class URGENT) (mode FORMULATED))

    (goal-class (class ?class&DISCARD-UNKNOWN) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?robot ?wp ?rs))

    (not (goal (class ?class) (params robot ?robot wp ?wp)))
    =>
    (do-for-fact ((?wm wm-fact)) (wm-key-prefix ?wm:key (create$ monitoring safety-discard))
        (bind ?parent ?urgent)
        (retract ?wm)
    )
    (printout t "Goal " ?class " formulated from PDDL" crlf)
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (priority ?*PRIORITY-DISCARD-UNKNOWN*)
                    (parent ?parent)
                    (params robot ?robot
                            wp ?wp
                    )
                    (required-resources ?wp)
    ))

    ;assert promises resulting from the plan-action of this goal
    (assert
        ;wp-discard
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
     )
)


; PRODUCTION MAINTENANCE GOALS

(defrule goal-class-assert-goal-get-from-bs-for-rs
    "If the preconditions of a get-base-to-fill-rs goal class is met assert the goal."
    (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))
    (goal (id ?maintain-id) (class PREPARE-RINGS) (mode FORMULATED))

    (goal-class (class ?class&GET-BASE-TO-FILL-RS) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values  ?robot ?wp ?rs ?bs ?side))

    (not (goal (class ?class) (params robot ?robot
                                            bs ?bs
                                            bs-side ?side
                                            base-color ?any-base
                                            wp ?wp)))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)
    (bind ?distance (node-distance (str-cat ?bs - (if (eq ?side INPUT) then I else O))))
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class)
                    (priority  (+ ?*PRIORITY-PREFILL-RS-WITH-FRESH-BASE* (goal-distance-prio ?distance)))
                    (parent ?maintain-id) (sub-type ?subtype)
                    (params robot ?robot
                            bs ?bs
                            bs-side ?side
                            base-color BASE_RED
                            wp ?wp
                    )
                    (required-resources ?wp)
            )
    )

    ;assert promises resulting from the plan-action of this goal
    (assert
        ;bs-dispense-trash
        (domain-promise (name wp-base-color) (param-values ?wp BASE_RED) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name wp-spawned-for) (param-values ?wp ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        ;wp-get
        (domain-promise (name wp-at) (param-values ?wp ?bs ?side) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name mps-state) (param-values ?bs READY-AT-OUTPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name mps-state) (param-values ?bs IDLE) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name mps-side-free) (param-values ?bs ?side) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?bs ?side) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?bs ?side) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)

(defrule goal-class-assert-goal-get-from-shelf-for-rs
    "If the preconditions of a get-shelf-to-fill-rs goal class is met assert the goal."
    (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))
    (goal (id ?maintain-id) (class PREPARE-RINGS) (mode FORMULATED))

    (goal-class (class ?class&GET-SHELF-TO-FILL-RS) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?robot ?rs ?cc ?cs ?spot))

    (not (goal (class ?class) (parent ?maintain-id) (params robot ?robot
                                                                          cs ?cs
                                                                          wp ?cc
                                                                          spot ?spot)))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)
    (bind ?distance (node-distance (str-cat ?rs -I)))
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class)
                    (priority (+ ?*PRIORITY-PREFILL-RS* (goal-distance-prio ?distance)))
                    (parent ?maintain-id) (sub-type ?subtype)
                    (params robot ?robot
                            cs ?cs
                            wp ?cc
                            spot ?spot
                    )
                    (required-resources ?cc)
            )
    )
    ;assert promises resulting from the plan-action of this goal
    (assert
        ;wp-get
        (domain-promise (name holding) (param-values ?robot ?cc) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name wp-on-shelf) (param-values ?cc ?cs ?spot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name wp-usable) (param-values ?cc) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name spot-free) (param-values ?cs ?spot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?cs INPUT) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?cs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)

(defrule goal-class-assert-goal-fill-rs
    "If the preconditions of a fill-rs goal class is met collect ring payment information
    and assert the goal."
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class PREPARE-RINGS) (mode FORMULATED))

    (goal-class (class ?class&FILL-RS) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?wp ?robot ?rs ?filled))

    (wm-fact (key domain fact mps-state args? m ?rs s ?state))
    (wm-fact (key domain fact rs-inc args? summand ?filled sum ?after))
    =>
    ;Check if this ring station should be filled with increased priority.
    (bind ?priority-increase 0)
    (do-for-all-facts ((?prio wm-fact)) (and (wm-key-prefix ?prio:key (create$ evaluated fact rs-fill-priority))
                                            (eq (wm-key-arg ?prio:key m) ?rs))
        (if (< ?priority-increase ?prio:value)
            then
            (bind ?priority-increase ?prio:value)
        ))
    ;
    (if (eq ?state DOWN)
        then
        (bind ?priority-increase (- ?priority-increase 1))
    )
    (bind ?distance (node-distance (str-cat ?rs -I)))
    (printout t "Goal " ?class " formulated from PDDL" crlf)
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (priority (+ ?*PRIORITY-PREFILL-RS* ?priority-increase (goal-distance-prio ?distance)))
                    (parent ?production-id)
                    (params robot ?robot
                            mps ?rs
                            wp ?wp
                            rs-before ?filled
                            rs-after ?after
                    )
                    (required-resources ?rs ?wp)
    ))
    ;assert promises resulting from the plan-action of this goal
    (assert
        ;wp-put-slide-cc
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name wp-usable) (param-values ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name rs-filled-with) (param-values ?rs ?after) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name rs-filled-with) (param-values ?rs ?filled) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
         ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?rs INPUT) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?rs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)

(defrule goal-class-assert-goal-fill-cap
    "If the preconditions of a fill-cap goal class is met assert the goal."
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class PREPARE-CAPS) (mode FORMULATED))

    (goal-class (class ?class&FILL-CAP) (id ?cid) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?robot ?cs ?cc ?spot ?cap-color))
    =>
    (bind ?priority-increase 0)
    ;increase priority if there is a product being produced that requires this cap-color
    (if (any-factp ((?order-cap-color wm-fact))
                    (and (wm-key-prefix ?order-cap-color:key (create$ domain fact order-cap-color))
                        (eq (wm-key-arg ?order-cap-color:key col) ?cap-color)
                        (any-factp ((?wp-for-order wm-fact))
                            (and (wm-key-prefix ?wp-for-order:key (create$ domain fact wp-for-order))
                                (eq (wm-key-arg ?wp-for-order:key ord) (wm-key-arg ?order-cap-color:key ord)))))
        )
    then
        (bind ?priority-increase 1)
        (printout t "Goal " ?class " formulated from PDDL with higher priority" crlf)
    else
        (printout t "Goal " ?class " formulated from PDDL" crlf)
    )
    (bind ?distance (node-distance (str-cat ?cs -I)))
    (bind ?goal-id  (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (priority (+ ?priority-increase ?*PRIORITY-PREFILL-CS* (goal-distance-prio ?distance)))
                    (parent ?production-id)
                    (params robot ?robot
                            mps ?cs
                            cc ?cc
                    )
                    (required-resources (sym-cat ?cs -INPUT) ?cc)
    ))
    ;assert promises resulting from the plan-action of this goal
    (assert
        ;wp-get-shelf
        (domain-promise (name wp-on-shelf) (param-values ?cc ?cs ?spot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name wp-usable) (param-values ?cc) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name spot-free) (param-values ?cs ?spot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        ;wp-put
        (domain-promise (name holding) (param-values ?robot ?cc) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name wp-at) (param-values ?cc ?cs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name mps-side-free) (param-values ?cs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?cs INPUT) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?cs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)


; MAINLINE PRODUCTION GOALS

(defrule goal-class-assert-goal-mount-first-ring
    "If the preconditions of a mount-first-ring goal class for an order is satisfied,
    the complexity is allowed, and the keep-mps-side-free condition is met and there
    is no such goal yet, collect the ring-payment information and assert the goal."
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))

    (goal-class (class ?class&MOUNT-FIRST-RING) (id ?cid) (meta order ?order) (sub-type ?subtype))
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
    (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))

    (not (wm-fact (key strategy keep-mps-side-free args? m ?rs side INPUT cause ~?wp)))
    (not (goal (class ?class) (parent ?production-id) (params robot ?robot $?
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
        (printout t "Goal " ?class " formulated from PDDL for order " ?order ", it needs the PRODUCE-EXCLUSIVE-COMPLEXITY token" crlf)
        else
        (printout t "Goal " ?class " formulated from PDDL for order " ?order crlf)
    )
    (bind ?distance (node-distance (str-cat ?bs - (if (eq ?side INPUT) then I else O))))

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
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
    ;assert promises resulting from the plan-action of this goal
    (assert
        ;handle the case where we get the base from the BS
        ;wp-put
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name wp-at) (param-values ?wp ?rs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name mps-side-free) (param-values ?rs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?rs INPUT) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?rs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)

(defrule goal-class-assert-goal-mount-next-ring
    "If the preconditions of a mount-next-ring goal class for an order is satisfied,
    the complexity is allowed, and the keep-mps-side-free condition is met and there
    is no such goal yet, collect the ring-payment information and assert the goal."
    (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))

    (goal-class (class ?class&MOUNT-NEXT-RING) (id ?cid) (meta order ?order ring ?ring) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?order ?robot ?wp ?base-color ?ring1-color ?ring2-color ?ring3-color ?other-color ?rs ?prev-rs ?bases-needed))

    (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
    (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                            subtrahend ?bases-needed
                                            difference ?bases-remain&ZERO|ONE|TWO|THREE))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
    (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))

    (not (wm-fact (key strategy keep-mps-side-free args? m ?rs side INPUT cause ~?wp)))
    (not (goal (class ?class) (parent ?maintain-id) (params robot ?robot $?
                                                                     wp ?wp $?
                                                                     order ?order)))
    =>
    (bind ?curr-ring-color ?ring2-color)
    (bind ?ring-pos 2)
    (if (eq ?ring ring3) then
        (bind ?curr-ring-color ?ring3-color)
        (bind ?ring-pos 3)
    )

    (printout t "Goal " ?class " formulated from PDDL for order " ?order " (Ring " ?ring-pos ") " crlf)
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (priority (+ ?ring-pos ?*PRIORITY-MOUNT-NEXT-RING*))
                    (parent ?production-id) (sub-type ?subtype)
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
    ;assert promises resulting from the plan-action of this goal
    (assert
        ;handle the case where we get the wp from the RS
        ;wp-put
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name wp-at) (param-values ?wp ?rs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name mps-side-free) (param-values ?rs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?rs INPUT) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?rs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)

(defrule goal-class-assert-goal-deliver
    "If the precondition for a DELIVER goal-class for an order has been met and there is
    no such DELIVER goal yet, assert and formulate the goal."
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class DELIVER-PRODUCTS) (mode FORMULATED))

    (goal-class (class ?class&DELIVER) (id ?cid) (meta order ?order) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?ds ?mps ?wp ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color ?order ?complexity ?gate))

    (not (goal (class ?class) (params $? robot ?robot $?
                                          order ?order
                                          wp ?wp
                                          ds ?ds
                                          ds-gate ?gate $?)))
    =>
    (printout t "Goal " ?class " formulated from PDDL for order " ?order crlf)
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
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
    ;assert promises resulting from the plan-action of this goal
    (assert
        ;handle the case where we get the product from a machine
        ;wp-put
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name wp-at) (param-values ?wp ?ds INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name mps-side-free) (param-values ?ds INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?ds INPUT) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?ds INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)

(defrule goal-class-assert-goal-produce-c0
    "If the precondition of a goal-class for a C0 order is fulfilled, assert the goal fact
    and thus formulate the goal. Determine the priority of the goal through meta reasoning
    on additional facts."
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))
    (goal (id ?urgent) (class URGENT) (mode FORMULATED))

    (goal-class (class ?class&PRODUCE-C0) (id ?cid) (meta order ?order) (sub-type ?subtype))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?mps ?wp ?cap-color ?bs ?side ?order ?base-color))

    (not (goal (class ?class) (params $? order ?order $?)))

    (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))
    (wm-fact (key config rcll competitive-order-priority) (value ?comp-prio))
    =>
    (printout t "Goal " ?class " formulated from PDDL for order " ?order crlf)
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
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
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
    ;assert promises resulting from the plan-action of this goal
    (assert
        ;handle the case where we get the product from a machine
        ;wp-put
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name wp-at) (param-values ?wp ?mps INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name mps-side-free) (param-values ?mps INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?mps INPUT) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?mps INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)

(defrule goal-class-assert-goal-produce-cx
    "If the precondition of a goal-class for a CX order is fulfilled, assert the goal
    fact and thus formulate the goal. Determine the priority of the goal based on
    its complexity."
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))
    (goal (id ?urgent) (class URGENT) (mode FORMULATED))

    (goal-class (class ?class&PRODUCE-CX) (id ?cid) (meta order ?order) (sub-type ?subtype))
    (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied TRUE) (grounding ?grounding-id))
    (pddl-grounding (id ?grounding-id) (param-values ?team-color ?robot ?cs ?order ?base-color ?ring1-color ?ring2-color ?ring3-color ?cap-color ?wp ?rs))

    (not (goal (class ?class)
                (parent ?production-id)
                (params robot ?robot
                        wp ?wp $?
                        mps ?cs $?
                        order ?order)))
    =>
    (bind ?prio ?*PRIORITY-PRODUCE-C1*)
    (if (eq ?com C2) then (bind ?prio ?*PRIORITY-PRODUCE-C2*))
    (if (eq ?com C3) then (bind ?prio ?*PRIORITY-PRODUCE-C3*))
    (printout t "Goal " ?class " formulated from PDDL for order " ?order crlf)
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
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
    ;assert promises resulting from the plan-action of this goal
    (assert
        ;handle the case where we get the product from a machine
        ;wp-put
        (domain-promise (name holding) (param-values ?robot ?wp) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        (domain-promise (name can-hold) (param-values ?robot) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name wp-at) (param-values ?wp ?cs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name mps-side-free) (param-values ?cs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
        ;go-wait
        (domain-promise (name at) (param-values ?robot (wait-pos ?cs INPUT) WAIT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated FALSE))
        (domain-promise (name at) (param-values ?robot ?cs INPUT) (promising-goal ?goal-id) (valid-at (+ 30 ?game-time)) (negated TRUE))
    )
)

; ------------------------- CLEAN UP GOAL CLASSES -----------------------------------
; TODO retract goal classes that are order dependend when they have been fulfilled
; and prevent reformulation to keep the fact base clean. This does not hurt performance
; for now.

; ------------------------- META CHECKS FOR GOALS -----------------------------------

(defrule goal-class-order-producible-C0
    "Assert an order-producible fact for a product of complexity C0 when it has not been
    fulfileld yet and the produce-ahead time has started."
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
    "Retract an order-producible fact for a product of complexity C0 when it has been
    fulfilled or the produce-latest time has been reached ."
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
    "Assert an order-producible fact for a product of complexity C1, C2, or C3 when it has
    not been fulfilled yet."
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
    "Retract an order-producible fact for a product of complexity C1, C2, or C3 when it has
    been fulfilled."
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
        (value ?qd&:(<= ?qr ?qd)))
    ?wmf <- (domain-fact (name order-producible) (param-values ?order))
    =>
    (retract ?wmf)
)

(defrule goal-class-order-deliverable
    "Assert an order-deliverable fact when it has not been fulfilled and delivery ahead
    time has started."
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
    "Retract an order-deliverable fact when the order has been fulfilled."
    (wm-fact (key refbox team-color) (value ?team-color))
    (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
    (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
        (value ?qd&:(<= ?qr ?qd)))
    ?wmf <- (domain-fact (name order-deliverable) (param-values ?order))
    =>
    (retract ?wmf)
)

(defrule goal-class-failed-put-slide
    "Assert an rs-failed-put-slide fact, if the robot failed putting a wp on a slide
    for *MAX-RETRIES-PICK* attempts."
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key domain fact mps-type args? m ?rs t RS))
    ?t <- (wm-fact (key monitoring action-retried args? r ?robot a wp-put-slide-cc m ?rs wp ?wp)
                   (value ?tried&:(>= ?tried ?*MAX-RETRIES-PICK*)))
    (not (domain-fact (name rs-failed-put-slide) (param-values ?rs ?robot ?wp)))
    =>
    (assert (domain-fact (name rs-failed-put-slide) (param-values ?rs ?robot ?wp)))
)

(defrule goal-class-order-out-of-delivery
    "Assert an order-out-of-delivery fact if the delivery time is over."
    (wm-fact (key refbox game-time) (values $?game-time))
    (wm-fact (key refbox order ?order delivery-end) (type UINT)
        (value ?end&:(< ?end (nth$ 1 ?game-time))))
    (not (domain-fact (name order-out-of-delivery) (param-values ?order)))
    =>
    (assert (domain-fact (name order-out-of-delivery) (param-values ?order)))
)
