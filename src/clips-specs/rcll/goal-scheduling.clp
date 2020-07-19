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

(defrule goal-scheduling-create-scheduler-root
  "Keep waiting at one of the waiting positions."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (class SCHEDULE-ORDERS)))

  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  (wm-fact (key domain fact self args? r ?self))

  ;Order-CEs
  (wm-fact (key domain fact order-complexity args? ord ?ord com ?complexity))
  =>
  (bind ?goal-id (sym-cat ROOT))

  (assert (goal (id ?goal-id)
                (parent ?production-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class SCHEDULE-ORDERS)
                (params ords [ O1 O2 ])))

  (printout t "Goal " SCHEDULE-ORDERS " formulated" crlf)
)

(defrule goal-scheduling-create-order
  "Keep waiting at one of the waiting positions."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class SCHEDULE-ORDERS) (params ords [ $? ?ord $? ]) (mode SELECTED))

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


  (not (wm-fact (key domain fact robot-waiting $?)))

  ;(test (eq ?complexity ?ord1-com))
  ;(test (member$ ?ord ?ords))
  ;(not (goal (id ?ord) (parent ?root-id) (class ORDER)))
  =>
  (bind ?g-id (sym-cat ?ord (gensym*)))

  (bind ?wp (sym-cat WP- (gensym*)))
  (assert (domain-object (name ?wp) (type workpiece))
          (domain-fact (name wp-base-color) (param-values ?wp BASE_NONE))
          (domain-fact (name wp-cap-color) (param-values ?wp CAP_NONE))
          (domain-fact (name wp-ring1-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-ring2-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-ring3-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-unused) (param-values ?wp)))


  (assert (goal (id ?g-id)
                (parent ?root-id)
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
                        wp ?wp)))

  (printout t "Goal " ?g-id " formulated" crlf)
)

(defrule goal-scheduling-order-production-goals
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class ORDER) (params $?params) (mode SELECTED))
  (not (goal (parent ?root-id)))

  (wm-fact (key refbox team-color) (value ?team-color))
  ;DS
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  ;CS
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?cs op ?cs-can-perform))
  (test (member$ (create$ cap-color ?cap-color) ?params))
  ;BS
  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  =>
  (bind ?ord (nth$ (+ 1 (member$ ord ?params)) ?params))
  (bind ?complexity (nth$ (+ 1 (member$ com ?params)) ?params))
  (bind ?params (append$ ?params delivery-station ?ds
                                 base-station ?bs
                                 cap-station ?cs))
  (bind ?ring# 0)
  (switch ?complexity
     (case C1 then (bind ?ring# 1))
     (case C2 then (bind ?ring# 2))
     (case C3 then (bind ?ring# 3))
  )
  (bind ?i ?ring#)
  (while (> ?i 0)
    (bind ?ring-color (nth$ (+ 1 (member$ (sym-cat ring ?i -color) ?params)) ?params))
    (do-for-fact ((?wm1 wm-fact) (?wm2 wm-fact))
                 (and (wm-key-prefix ?wm1:key (create$ domain fact mps-team))
                      (wm-key-prefix ?wm2:key (create$ domain fact rs-ring-spec))
                      (eq (wm-key-arg ?wm1:key m) (wm-key-arg ?wm2:key m))
                      (eq (wm-key-arg ?wm1:key col) ?team-color)
                      (eq (wm-key-arg ?wm2:key r) ?ring-color))
       (bind ?rs (wm-key-arg ?wm2:key m))
       (bind ?params (create$ ?params (sym-cat ring ?i -station) ?rs ))
    )
    (bind ?i (- ?i 1))
  )



  ;;DELIVER
  (bind ?parent-id ?root-id)
  (bind ?goal-id (sym-cat ?ord DELIVER _ (gensym*)))
  (assert (goal (id ?goal-id )
                (parent ?parent-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class DELIVER)
                (params ?params)))
  (printout t "Goal " ?goal-id " formulated" crlf)
  ;;MOUNT-CAP
  (bind ?parent-id ?goal-id)
  (bind ?goal-id (sym-cat ?ord MOUNTCAP _ (gensym*)))
  (assert (goal (id ?goal-id)
                (parent ?parent-id)
                (sub-type SCHEDULE-SUBGOALS)
                (class MOUNT-CAP)
                (params ?params)))
  (printout t "Goal " ?goal-id " formulated" crlf)
  ;; Prepare CS goals
  (if (eq ?cs-can-perform RETRIEVE_CAP)
       then
       (bind ?binding-id (sym-cat X (gensym*)))
       (bind ?shelf-spot (sym-cat ?binding-id #spot))
       (bind ?cc  (sym-cat ?binding-id #wp))
       (assert (domain-object (name ?cc) (type cap-carrier)))
       (assert (wm-fact (key meta binding args? id ?binding-id policy BIND-UNIQUE)
                        (values (create$ domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
                        (is-list TRUE)))
       (bind ?params (append$ ?params  cc ?cc shelf-spot ?shelf-spot))

       ;;CLEAR-GOAL
       (bind ?parent-id ?goal-id)
       (bind ?subgoal-id (sym-cat ?ord CLEARCAP _ (gensym*) ))
       (assert (goal (id ?subgoal-id)
                     (parent ?parent-id)
                     (sub-type SCHEDULE-SUBGOALS)
                     (class PREPARE-CS)
                     (params ?params)))
       (printout t "Goal " ?subgoal-id " formulated" crlf)
       ;; FILL-CAP
       (bind ?parent-id ?subgoal-id)
       (bind ?subgoal-id (sym-cat ?ord FILLCAP _ (gensym*)))
       (assert (goal (id ?subgoal-id)
                     (parent ?parent-id)
                     (sub-type SCHEDULE-SUBGOALS)
                     (class FILL-CAP)
                     (params ?params)))
       (printout t "Goal " ?subgoal-id " formulated" crlf)
  )

  ;;MOUNT-RING
  (while (> ?ring# 0)
    (bind ?ring-color (nth$ (+ 1 (member$ (sym-cat ring ?ring# -color) ?params)) ?params))
    (do-for-fact ((?wm1 wm-fact) (?wm2 wm-fact))
                 (and (wm-key-prefix ?wm1:key (create$ domain fact mps-team))
                      (wm-key-prefix ?wm2:key (create$ domain fact rs-ring-spec))
                      (eq (wm-key-arg ?wm1:key m) (wm-key-arg ?wm2:key m))
                      (eq (wm-key-arg ?wm1:key col) ?team-color)
                      (eq (wm-key-arg ?wm2:key r) ?ring-color))
       (bind ?rs (wm-key-arg ?wm2:key m))
       (bind ?req-num (wm-key-arg ?wm2:key rn))
    )

    (bind ?parent-id ?goal-id)
    (bind ?goal-id (sym-cat ?ord MOUNTRING ?ring#  _ (gensym*)))
    (assert (goal (id ?goal-id)
                  (parent ?parent-id)
                  (sub-type SCHEDULE-SUBGOALS)
                  (class (sym-cat MOUNT-RING ?ring#))
                  (params ?params)))
    (printout t "Goal " ?goal-id " formulated" crlf)

     ;;Create Fill-RS sub-goals
     (if (neq ?req-num ZERO)
         then
         (bind ?fill# (member$ ?req-num (create$ ONE TWO THREE)))
         (bind ?subgoal-id ?goal-id)
         (while (> ?fill# 0)
                (bind ?parent-id ?subgoal-id)
                (bind ?subgoal-id (sym-cat ?ord FILLRS ?ring# ?fill#  _ (gensym*)))
                (assert (goal (id ?subgoal-id)
                              (parent ?parent-id)
                              (class FILL-RS)
                              (sub-type SCHEDULE-SUBGOALS)
                              (params (create$ ?params fill-rs ?rs))))
                (bind ?fill# (- ?fill# 1))
                (printout t "Goal " ?subgoal-id  " formulated" crlf)
         )
     )
     (bind ?ring# (- ?ring# 1))
  )
)
