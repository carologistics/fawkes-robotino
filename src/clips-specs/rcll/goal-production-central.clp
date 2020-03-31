;---------------------------------------------------------------------------
;  goal-maintain-production.clp - Generate production goals of RCLL
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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


(defrule goal-production-create-order
  "Keep waiting at one of the waiting positions."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key domain fact self args? r ?self))
  (wm-fact (key domain fact quantity-delivered args? ord O1 team CYAN))
  (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  (not (goal (class ORDER)))
  =>
  (printout t "Goal " ORDER " formulated" crlf)
  (assert (goal (id ORDER-g)
                (params ord O1)
                (class ORDER)
				(sub-type RUN-ALL-OF-SUBGOALS)
                (parent ?production-id)
  ))
)

(defrule goal-production-create-operations
  "Keep waiting at one of the waiting positions."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class ORDER) (id ?goal-id) (mode SELECTED))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  (not (goal (class WP-OPERATIONS) (parent ?goal-id)))
  (not (goal (class PREPARE-MACHINE) (parent ?goal-id)))
  =>
  (printout t "Goal " WPOPERATTIONS " formulated" crlf)
  (assert (goal (id  WP-OPERATIONS-g)
                (class WP-OPERATIONS)
                (sub-type RUN-ALL-OF-SUBGOALS)
                (parent ?goal-id)
                (params (create$ wp ?spawned-wp))
  ))
  (assert (goal (id PREPARE-MACHINE-g)
                (class PREPARE-MACHINE)
                (sub-type RUN-ALL-OF-SUBGOALS)
                (parent ?goal-id)
  ))
)

(defrule goal-production-create-prepare-cs
" Fill a cap into a cap station.
  Use a capcarrier from the corresponding shelf to feed it into a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?goal-id) (class ORDER) (params ord ?ord $?))
         ;(committed-to $? ?prepare-machine $))
  (goal (id ?prepare-machine) (class PREPARE-MACHINE) (parent ?goal-id)
        (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS CEs (CS)
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ; (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  ; (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color)))
  ; (not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side INPUT))
  ;Capcarrier CEs
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (not (goal (class PREPARE-CS) (parent ?prepare-machine)))
  =>
  (printout t "Goals " related to CAP " formulated" crlf)
  ;; sche (bind ?distance (node-distance (str-cat ?mps -I)))
  (assert (goal (id PREPARE-CS-g)
                (class PREPARE-CS) (sub-type RUN-ALL-OF-SUBGOALS)
                (parent ?prepare-machine)
                (params (create$  cc ?cc cs ?mps ))
			))
)

(defrule goal-production-create-prepare-cs-simple-goals
" Fill a cap into a cap station.
  Use a capcarrier from the corresponding shelf to feed it into a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?goal-id) (class ORDER) (params ord ?ord $?))
        ;(committed-to $? ?prepare-machine $?))
  (goal (id ?prepare-machine) (class PREPARE-MACHINE) (parent ?goal-id))
        ;(committed-to $? ?prepare-cs $?))
  (goal (id ?prepare-cs) (class PREPARE-CS) (parent ?prepare-machine)
        (params $? cc ?cc $? cs ?cs $?) (mode SELECTED))
 ; (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  ; (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color)))
  ; (not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side INPUT))
  ;Capcarrier CEs
  (not (goal (class FILL-CAP) (parent ?prepare-cs)))
  (not (goal (class CLEAR-MPS) (parent ?prepare-cs)))
  =>
  (assert (goal (id FILL-CAP-g)
                (class FILL-CAP) (sub-type SIMPLE)
                (parent ?prepare-cs)
                (params (create$ cc ?cc cs ?cs))
				))
  (assert (goal (id CLEAR-MPS-g)
                (class CLEAR-MPS) (sub-type SIMPLE)
                (parent ?prepare-cs)
                (params (create$ wp ?cc mps ?cs))
  ))
  ;(required-resources (sym-cat ?mps -OUTPUT) ?cc)
)

(defrule goal-production-create-cs-operations-simple-goals
" Fill a cap into a cap station.
  Use a capcarrier from the corresponding shelf to feed it into a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?goal-id) (class ORDER) (params ord ?ord $?))
        ;(committed-to $? ?wp-operations $?))
  (goal (id ?wp-operations) (class WP-OPERATIONS) (parent ?goal-id)
        (params wp ?wp) (mode SELECTED))
  (goal (id ?prepare-machine) (class PREPARE-MACHINE) (parent ?goal-id))
  (goal (id ?prepare-cs) (class PREPARE-CS) (parent ?prepare-machine)
        (params $? cs ?cs $?))
 ; (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  ; (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color)))
  ; (not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side INPUT))
  ;Capcarrier CEs
  (not (goal (class MOUNT-CAP) (parent ?wp-operations)))
  (not (goal (class DELIVER) (parent ?wp-operations)))
  =>
  (assert (goal (id MOUNT-CAP-g)
                (class MOUNT-CAP) (sub-type SIMPLE)
                (parent ?wp-operations)
                (params (create$ wp ?wp cs ?cs))
  ))
  (assert (goal (id DELIVER-g)
                (class DELIVER) (sub-type SIMPLE)
                (parent ?wp-operations)
                (params (create$ wp ?wp cs ?cs))
  ))
)


(defrule goal-production-create-produce-c0
" Produce a C0 product: Get the correct base and mount the right cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?order) (class ORDER) (mode FORMULATED))
  (goal (id ?wp-operations) (parent ?order) (class WP-OPERATIONS) (mode FORMULATED))
  (goal (id ?mount-cap) (parent ?wp-operations) (class MOUNT-CAP) (mode FORMULATED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  ;; (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side INPUT)))
  ;; (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;; (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  ;; (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (domain-object (name ?bs-side&:(or (eq ?bs-side INPUT) (eq ?bs-side OUTPUT))) (type mps-side))
  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  
  (or (and (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
           (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN))
           (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
           (not (wm-fact (key order meta wp-for-order args? wp ?any-ord-wp ord ?order))))
      (and (wm-fact (key domain fact holding args? r ?robot wp ?spawned-wp))
           (wm-fact (key domain fact wp-base-color args? wp ?spawned-wp col ?base-color))
           (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
           (wm-fact (key domain fact wp-cap-color args? wp ?wp cap-color CAP_NONE))))
  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  (test (eq ?complexity C0))
  (not (goal (class PRODUCE-C0)(parent ?mount-cap)))
  =>
  (bind ?required-resources ?order ?spawned-wp)
  ;If this order complexity should be produced exclusively ...
  (if (any-factp ((?exclusive-complexities wm-fact))
        (and (wm-key-prefix ?exclusive-complexities:key (create$ config rcll exclusive-complexities))
             (neq FALSE (member$ (str-cat ?complexity) ?exclusive-complexities:values))))
    then
      ;... then an exclusive order token is required.
      (bind ?required-resources ?mps ?order ?spawned-wp PRODUCE-EXCLUSIVE-COMPLEXITY)
      (printout t "Goal " PRODUCE-C0 " formulated, it needs the PRODUCE-EXCLUSIVE-COMPLEXITY token" crlf)
    else
      (printout t "Goal " PRODUCE-C0 " formulated" crlf))
  (assert (goal (id PRODUCE-C0-g)
                (class PRODUCE-C0) (sub-type SIMPLE)
                (parent ?mount-cap)
                (params robot ?robot
                        bs ?bs
                        bs-side ?bs-side
                        bs-color ?base-color
                        mps ?mps
                        cs-color ?cap-color
                        order ?order
                        wp ?spawned-wp
                )
                (required-resources (sym-cat ?mps -INPUT) ?required-resources)
  ))
)

(defrule goal-production-create-deliver
  "Deliver a fully produced workpiece."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?order) (class ORDER) (mode FORMULATED))
  (goal (id ?wp-operations) (parent ?order) (class WP-OPERATIONS) (mode FORMULATED))
  (goal (id ?deliver) (parent ?wp-operations) (class DELIVER) (mode FORMULATED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS-DS CEs
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  ;MPS-CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;WP-CEs
  ;(wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  ;(wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  ;(wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  ;(wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  ;(wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  ;Order-CEs
  ;(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  ;note: could be moved to rejected checks
  ;(wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
  ;         (value ?qd&:(> ?qr ?qd)))
  ;(wm-fact (key refbox order ?order delivery-begin) (type UINT)
  ;         (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*))))
  ;(or (and (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  ;         (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp))))
  ;    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
  (not (goal (class DELIVER)
             (parent ?deliver)
  ))
  =>
  (printout t "Goal " DELIVER " formulated" crlf)
  (assert (goal (id DELIVER-g)
                (class DELIVER) (sub-type SIMPLE)
                (parent ?deliver)
                (params robot ?robot
                        mps ?mps
                        order ?order
                        wp X-wp
                        ds ?ds
                        ds-gate ?gate
                        base-color ?base-color
                        ring1-color ?ring1-color
                        ring2-color ?ring2-color
                        ring3-color ?ring3-color
                        cap-color ?cap-color
                )
                (required-resources (sym-cat ?mps -OUTPUT) ?order X-wp)
  ))
)






;::Rejecting goals::;

(defrule goal-production-reject-mount-3rd-ring
"Reject 3rd ring compount goals when for lower complexities"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?order) (class ORDER) (mode FORMULATED))
  (goal (id ?wp-operations) (parent ?order) (class WP-OPERATIONS) (mode FORMULATED))
  (goal (id ?prepare-machines) (parent ?order) (class PREPARE-MACHINE) (mode FORMULATED))
  ?gf1 <-  (goal (id ?prepare-ring3) (parent ?prepare-machine)
                 (class PREPARE-RING3) (mode FORMULATED))
  ?gf2 <-  (goal (id ?mount-ring3) (parent ?wp-operations)
                 (class MOUNT-RING3) (mode FORMULATED))
  ;Game CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (not (eq ?complexity C3))
  =>
  (modify ?gf1 (mode RETRACTED) (outcome REJECTED))
  (modify ?gf2 (mode RETRACTED) (outcome REJECTED))
)

(defrule goal-production-reject-mount-2rd-ring
"Reject 2rd ring compount goals when for lower complexities"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?order) (class ORDER) (mode FORMULATED))
  (goal (id ?wp-operations) (parent ?order) (class WP-OPERATIONS) (mode FORMULATED))
  (goal (id ?prepare-machines) (parent ?order) (class PREPARE-MACHINE) (mode FORMULATED))
  ?gf1 <-  (goal (id ?prepare-ring2) (parent ?prepare-machine)
                 (class PREPARE-RING2) (mode FORMULATED))
  ?gf2 <-  (goal (id ?mount-ring2) (parent ?wp-operations)
                 (class MOUNT-RING2) (mode FORMULATED))
  ;Game CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (and (not (eq ?complexity C3))
       (not (eq ?complexity C2))
  )
  =>
  (modify ?gf1 (mode RETRACTED) (outcome REJECTED))
  (modify ?gf2 (mode RETRACTED) (outcome REJECTED))
)

(defrule goal-production-reject-mount-1st-ring
"Reject 2rd ring compount goals when for lower complexities"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?order) (class ORDER) (mode FORMULATED))
  (goal (id ?wp-operations) (parent ?order) (class WP-OPERATIONS) (mode FORMULATED))
  (goal (id ?prepare-machines) (parent ?order) (class PREPARE-MACHINE) (mode FORMULATED))
  ?gf1 <-  (goal (id ?prepare-ring1) (parent ?prepare-machine)
                 (class PREPARE-RING1) (mode FORMULATED))
  ?gf2 <-  (goal (id ?mount-ring1) (parent ?wp-operations)
                 (class MOUNT-RING1) (mode FORMULATED))
  ;Game CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (and (not (eq ?complexity C3))
       (not (eq ?complexity C2))
       (not (eq ?complexity C1))
  )
  =>
  (modify ?gf1 (mode RETRACTED) (outcome REJECTED))
  (modify ?gf2 (mode RETRACTED) (outcome REJECTED))
)
