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
  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  (not (goal (class ORDER)))
  =>
  (printout t "Goal " ORDER " formulated" crlf)
  (assert (goal (id O1)
               (class ORDER) (sub-type RUN-ALL-OF-SUBGOALS)
               (parent ?production-id)
  ))
)


(defrule goal-production-create-prepare-cap
" Fill a cap into a cap station.
  Use a capcarrier from the corresponding shelf to feed it into a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?order) (class ORDER) (mode FORMULATED))
  (goal (id ?prepare-cap) (class PREPARE-CAP) (mode FORMULATED))
  (goal (id ?fill-cap) (parent ?prepare-cap) (class FILL-CAP) (mode FORMULATED))
  (goal (id ?clear-cap) (parent ?prepare-cap) (class CLEAR-CAP) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;MPS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  ;; ex (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;; ex (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color)))
  ; ex (not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side INPUT))
  ;Capcarrier CEs
  ;; ex (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))
  ;; ex (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
 ;; order-selection (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
 ;; order-selection (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
 ;; order-selection (value ?qd&:(> ?qr ?qd)))
 ;; order-selection (wm-fact (key refbox order ?order delivery-end) (type UINT)
 ;; order-selection (value ?end&:(> ?end (nth$ 1 ?game-time))))
  =>
  (printout t "Goals " related to CAP " formulated" crlf)
  ;; sche (bind ?distance (node-distance (str-cat ?mps -I)))
  (assert (goal (id (sym-cat FILL-CAP- (gensym*)))
                (class FILL-CAP) (sub-type SIMPLE)
                (parent ?fill-cap)
                (params robot X-R
                        mps ?mps
                        cc X-CC
                )
                (required-resources (sym-cat ?mps -INPUT) ?cc)
  ))

  (assert (goal (id (sym-cat CLEAR-CAP- (gensym*)))
                (class CLEAR-CAP) (sub-type SIMPLE)
                (parent ?clear-cap)
                (params robot X-R
                        mps ?mps
						cc X-CC
                )
                (required-resources (sym-cat ?mps -OUTPUT) X-CC)
  ))
)

(defrule goal-production-create-produce-c0
" Produce a C0 product: Get the correct base and mount the right cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?order) (class MOUNT-CAP) (mode FORMULATED))
  (goal (id ?produciton-id) (class MOUNT-CAP) (mode FORMULATED))
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
  (not (goal (class PRODUCE-C0)
             (parent ?parent)
             (params robot ?robot
                     bs ?bs
                     bs-side ?bs-side $?
                     mps ?mps $?
                     order ?order
                     wp ?spawned-wp
             )
  ))
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
  (bind ?parent ?production-id)
  (bind ?priority-decrease 0)
  (if (and (eq ?comp-prio "HIGH") ?competitive)
    then
     (bind ?parent ?urgent))
  (if (eq ?comp-prio "LOW")
    then
      (bind ?priority-decrease 1))
  (bind ?distance (node-distance (str-cat ?bs - (if (eq ?bs-side INPUT) then I else O))))

  (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
                (class PRODUCE-C0) (sub-type SIMPLE)
                (priority (+ (- ?*PRIORITY-PRODUCE-C0* ?priority-decrease) (goal-distance-prio ?distance)))
                (parent ?parent)
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
