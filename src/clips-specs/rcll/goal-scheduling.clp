;---------------------------------------------------------------------------
;  production-scheduling.clp - Generate production goals of RCLL
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


(defglobal ?*V* = 0.5)

(defrule goal-scheduling-create-order
  "Keep waiting at one of the waiting positions."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (class ORDER)))

  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  (wm-fact (key domain fact self args? r ?self))

  ;Order-CEs
  (wm-fact (key domain fact order-complexity args? ord ?ord com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?ord col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?ord col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?ord col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?ord col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?ord col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?ord gate ?gate))
  (wm-fact (key refbox order ?ord quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? ord ?ord team ?team-color)
           (value ?del&:(> ?qr ?del)))

  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))

  (test (eq ?ord O2))
  =>
  (bind ?g-id (sym-cat ROOT_ ?ord))

  (bind ?v-wp FVAR_WP)
  (bind ?v-robot FVAR_WP_R)
  (assert (wm-fact (key meta grounding wp-spawned-for args? wp ?v-wp r ?v-robot)))

  (assert (goal (id ?g-id)
                (parent ?production-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class ORDER)
                (params ord ?ord
                        com ?complexity
                        base-color ?base-color
                        ring1-color ?ring1-color
                        ring2-color ?ring2-color
                        ring3-color ?ring3-color
                        cap-color ?cap-color
                        gate ?gate
                        wp ?v-wp
                        base-station ?bs)))

  (printout t "Goal " ORDER " formulated" crlf)
)

(defrule goal-scheduling-create-deliver
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class ORDER) (mode SELECTED))
  ?gf <- (goal (class MOUNT-CAP) (parent ?root-id) (params $?params) (mode SELECTED))
  (not (goal (parent ?root-id) (class DELIVER)))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  =>
  (bind ?ord (nth$ (+ 1 (member$ ord ?params)) ?params))
  (bind ?goal-id (sym-cat DELIVER_ ?ord))

  (modify ?gf (parent ?goal-id))

  (assert (goal (id ?goal-id )
                (parent ?root-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class DELIVER)
                (params (create$ ?params delivery-station ?ds ))))

  (printout t "Goal " DELIVER " formulated" crlf)
)

(defrule goal-scheduling-create-mount-cap
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class ORDER) (mode SELECTED))

  ?gf <- (goal (parent ?priori-parent) (class ?priori-class)
               (params $?params) (mode SELECTED))

  (or (and (test (member$ (create$ com C0) ?params))
           (not (goal (parent ?root-id))))
       (and (test (member$ (create$ com C1) ?params))
            (test (eq ?priori-class MOUNT-RING1))
            (test (eq ?priori-parent ?root-id)))
       (and (test (member$ (create$ com C2) ?params))
            (test (eq ?priori-class MOUNT-RING2))
            (test (eq ?priori-parent ?root-id)))
       (and (test (member$ (create$ com C3) ?params))
            (test (eq ?priori-class MOUNT-RING3))
            (test (eq ?priori-parent ?root-id))))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))

  (test (member$ (create$ cap-color ?cap-color) ?params))
  =>
  (bind ?ord (nth$ (+ 1 (member$ ord ?params)) ?params))
  (bind ?goal-id (sym-cat MOUNT_CAP ?ord))

  (if (eq ?priori-parent ?root-id) then (modify ?gf (parent ?goal-id)))

  (assert (goal (id ?goal-id)
                (parent ?root-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class MOUNT-CAP)
                (params (create$ ?params cap-station ?cs))))
  (printout t "Goal " MOUNT-CAP " formulated" crlf)
)

;; Prepare CS goals
(defrule goal-scheduling-create-prepare-cs
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?goal-id) (params $?params) (class MOUNT-CAP) (mode SELECTED))
  (not (goal (parent ?goal-id) (class PREPARE-CS)))

  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
  (wm-fact (key domain fact cs-can-perform args? m ?cs op RETRIEVE_CAP))
  (not (wm-fact (key domain fact cs-buffered args? m ?cs col ?cap-color)))

  (test (member$ (create$ cap-station ?cs) ?params))
  (test (member$ (create$ cap-color ?cap-color) ?params))
  =>
  (bind ?ord (nth$ (+ 1 (member$ ord ?params)) ?params))
  (bind ?g-id (sym-cat PREPARE_CS_ ?ord))

  (assert (goal (id ?g-id)
                (parent ?goal-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class PREPARE-CS)
                (params (create$ ?params
                                 cc ?cc
                                 shelf-spot ?shelf-spot))))

  (printout t "Goals " PREPARE-CS " formulated" crlf)
)

(defrule goal-scheduling-create-prepare-cs-fill-cap
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?goal-id) (params $?params) (class PREPARE-CS) (mode SELECTED))
  (not (goal (parent ?goal-cs) (class FILL-CAP)))
  =>
  (bind ?ord (nth$ (+ 1 (member$ ord ?params)) ?params))
  (bind ?g-id (sym-cat FILL_CAP ?ord))

  (assert (goal (id ?g-id)
                (parent ?goal-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class FILL-CAP)
                (params ?params)))

  (printout t "Goals " FILL-CS " formulated" crlf)
)

(defrule goal-scheduling-create-mount-rings
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class ORDER) (mode SELECTED))

  ?gf <- (goal (parent ?priori-parent) (class ?priori-class)
               (params $?params) (mode SELECTED))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring-color rn ?req-num))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?rs t RS))

  (or (and (test (member$ (create$ ring1-color ?ring-color) ?params))
           (not (goal (parent ?root-id))))
       (and (test (member$ (create$ ring2-color ?ring-color) ?params))
            (test (eq ?priori-class MOUNT-RING1))
            (test (eq ?priori-parent ?root-id)))
       (and (test (member$ (create$ ring3-color ?ring-color) ?params))
            (test (eq ?priori-class MOUNT-RING2))
            (test (eq ?priori-parent ?root-id))))

  (test (neq ?ring-color RING_NONE))
  =>
  (bind ?ord (nth$ (+ 1 (member$ ord ?params)) ?params))

  (bind ?ring# 1)
  (if (eq ?priori-class MOUNT-RING1) then  (bind ?ring# 2))
  (if (eq ?priori-class MOUNT-RING2) then  (bind ?ring# 3))

  (bind ?goal-id (sym-cat MOUNT_RING ?ring# ?ord))
  (bind ?params (create$ ?params (sym-cat ring ?ring# -station) ?rs))

  ;Steal parentship of lastest goal
  (if (eq ?priori-parent ?root-id) then
      (modify ?gf (parent ?goal-id)))

  (assert (goal (id ?goal-id)
                (parent ?root-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class (sym-cat MOUNT-RING ?ring#))
                (params (create$ ?params (sym-cat ring ?ring# -station) ?rs))))

  (printout t "Goal " (sym-cat MOUNT-RING ?ring#) " formulated" crlf)

  ;;Create Fill-RS goals
  (if (neq ?req-num ZERO) then
    (bind ?set (create$ na THREE TWO ONE))
    (bind ?req-set (delete$ ?set 1 (- (member$ ?req-num ?set) 1)))

    (progn$ (?fill-base# ?req-set)
      (bind ?parent-id ?goal-id)
      (bind ?goal-id (sym-cat FILL_RS ?ring# _ ?fill-base# _ ?ord))

      (assert (goal (id ?goal-id)
                    (parent ?parent-id)
                    (class FILL-RS)
                    (sub-type SCHEDULE-SUBGOALS)
                    (params (create$ ?params
                                     fill-base# ?fill-base#
                                     fill-rs ?rs))))

      (printout t "Goal " (sym-cat FILL-RS- ?fill-base#) " formulated" crlf)
    )
  )
)
