;---------------------------------------------------------------------------
;  fixed-sequence.clp - Goal expander for RCLL goals
;
;  Created: Tue 08 Mar 2019 17:03:31 CET
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

; An event is initialized with a type (start|end) and related to the id of the
; fact that it represents.
;(wm-fact (key scheduling event args? id ?g-id type end) (value ?event-name)))
; sets
; (wm-fact (key scheduling dataset events)    (values ))
; (wm-fact (key scheduling dataset resources) (values ))
; (wm-fact (key scheduling dataset goals)     (values ))
; (wm-fact (key scheduling dataset plans)     (values ))
;
; (wm-fact (key scheduling dataset goal-plans args? g ?g-id) (values ))
; (wm-fact (key scheduling dataset goal-events args? p ?p-id)) plan-events + lg
; (wm-fact (key scheduling dataset plan-events args? p ?p-id))
; (wm-fact (key scheduling dataset resource-consuming-events args? r ?r-id))
; (wm-fact (key scheduling dataset resource-producing-events args? r ?r-id))
;
; In principle each EVENT has a location, duration, resource consumption or
; supply. Those are tracked by parameters.
; PARAMETERS:
; (wm-fact (key scheduling param event-duration args? e ?e-id) (v))
; (wm-fact (key scheduling param event-location args? e ?e-id) (v))
; (wm-fact (key scheduling param event-precedence args? e-a ?e-before e-b ?after) ())
; (wm-fact (key scheduling param demand-supply args? r ?r-id e-b ?e-id) (v +1 -1))
; (wm-fact (key scheduling param traveling-duration args? e-a ?e-id e-b ?e-id) (v))

;(defrule scheduling-calc-action-duration-move
;?p <- (plan-action (action-name move) (duration 0.0)
;                   (param-values ?robot ?from ?from-side ?to ?to-side))
;=>
; (bind ?from-node (node-name ?from ?from-side))
; (bind ?to-node   (node-name ?to ?to-side))
; (bind ?distance  (nodes-distance ?from-node ?to-node))
; (modify ?p  (duration (/ ?distance 0.03)))
;)

;(defrule scheduling-calc-plan-duration
; "Add calculate total plan duration"
; (plan-action (id ?id) (plan-id ?plan-id) (duration ?duration&~0.0))
; ?fa <- (wm-fact (key meta fact plan durative-action args? id ?plan-id)
;                 (values $?actions))
; ?fd <- (wm-fact (key meta fact plan duration args? id ?plan-id)
;                 (value ?d))
; (test (not (member$ ?id ?actions)))
; =>
; (modify ?fd (value (+ ?d ?duration)))
; (modify ?fa (values (create$ ?actions ?id)))
;)


(deffunction plan-duration (?plan-id)
 (bind ?duration 0)
 (do-for-all-facts ((?pa plan-action))
                    (eq ?pa:plan-id ?plan-id)
    (bind ?duration (+ ?duration ?pa:duration))
  )
  (return ?duration)
)


(defrule scheduling-goal-class-precedence
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (goal (id ?order-id) (parent ?goal-id) (class ORDER) (mode FORMULATED))
  (not (wm-fact (key meta precedence goal-class args? $?)))
=>
  (assert
   (wm-fact (key meta precedence goal-class args? a PREPARE-RING1 b MOUNT-RING1))
   (wm-fact (key meta precedence goal-class args? a PREPARE-RING2 b MOUNT-RING2))
   (wm-fact (key meta precedence goal-class args? a PREPARE-RING3 b MOUNT-RING3))

   (wm-fact (key meta precedence goal-class args? a FILL-CAP b CLEAR-MPS))
   (wm-fact (key meta precedence goal-class args? a CLEAR-MPS b MOUNT-CAP))

   (wm-fact (key meta precedence goal-class args? a MOUNT-RING1 b MOUNT-RING2))
   (wm-fact (key meta precedence goal-class args? a MOUNT-RING2 b MOUNT-RING3))
   (wm-fact (key meta precedence goal-class args? a MOUNT-RING3 b MOUNT-CAP))

   (wm-fact (key meta precedence goal-class args? a MOUNT-CAP b DELIVER))
  )
)

(defrule scheduling-init-dataset-goal-events
 (goal (id ?g-id) (sub-type SIMPLE) (mode EXPANDED) (class ?class))
 (wm-fact (key meta precedence goal-class args? $?  ?class $?))
 (not (wm-fact (key scheduling event args? id ?g-id type ?)))
 =>
 (bind ?event-name (sym-cat E- ?g-id -end))
 (assert (wm-fact (key scheduling event args? id ?g-id type end) (value ?event-name))
         (wm-fact (key scheduling dataset goal-events args? g ?g-id)
                  (type SYMBOL) (is-list TRUE) (values ?event-name)))
)

(defrule scheduling-populate-dataset-goal-events
 (goal (id ?g-id) (sub-type SIMPLE) (mode EXPANDED) (class ?class))
 (plan (id ?p-id) (goal-id ?g-id))
 (wm-fact (key meta precedence goal-class args? $?  ?class $?))
 (wm-fact (key scheduling event args? id ?p-id type ?) (value ?e))
 ?geset <- (wm-fact (key scheduling dataset goal-events args? g ?g-id)
                  (values $?events&:(not (member$ ?e ?events))))
 =>
 (modify ?geset (values (create$ ?events ?e)))
)

;;;Goal Events
(defrule scheduling-init-dataset-goal-events
 (goal (id ?g-id) (sub-type SIMPLE) (mode EXPANDED) (class ?class))
 (wm-fact (key meta precedence goal-class args? $?  ?class $?))
 (not (wm-fact (key scheduling event args? id ?g-id type ?)))
 =>
 (bind ?event-name (sym-cat E- ?g-id -end))
 (assert (wm-fact (key scheduling event args? id ?g-id type end) (value ?event-name))
         (wm-fact (key scheduling dataset goal-events args? g ?g-id)
                  (type SYMBOL) (is-list TRUE) (values ?event-name)))
)

(defrule scheduling-populate-dataset-goal-events
 (goal (id ?g-id) (sub-type SIMPLE) (mode EXPANDED) (class ?class))
 (plan (id ?p-id) (goal-id ?g-id))
 (wm-fact (key meta precedence goal-class args? $?  ?class $?))
 (wm-fact (key scheduling event args? id ?p-id type ?) (value ?e))
 ?geset <- (wm-fact (key scheduling dataset goal-events args? g ?g-id)
                  (values $?events&:(not (member$ ?e ?events))))
 =>
 (modify ?geset (values (create$ ?events ?e)))
)

;;;Goal Plans
(defrule scheduling-init-datasets-goal-plans
 (goal (id ?g-id) (class ?class))
 (plan (id ?p-id) (goal-id ?g-id))
 (wm-fact (key meta precedence goal-class args? $?  ?class $?))
 (not (wm-fact (key scheduling dataset goal-plans args? g ?g-id)))
 =>
 (assert (wm-fact (key scheduling dataset goal-plans args? g ?g-id)
                  (type SYMBOL) (is-list TRUE)))
)

(defrule scheduling-populate-dataset-goal-plans
 (goal (id ?g-id) (class ?class))
 (plan (id ?p-id) (goal-id ?g-id))
 (wm-fact (key meta precedence goal-class args? $?  ?class $?))
 ?gpset <- (wm-fact (key scheduling dataset goal-plans args? g ?g-id)
                    (values $?plans&:(not (member$ ?p-id ?plans))))
 =>
 (modify ?gpset (values (create$ ?plans ?p-id)))
)

;;;;Resource Set
(defrule scheduling-init-dataset-resource-events
 (or  (wm-fact (key scheduling param event-supply args? e ? r ?r))
      (wm-fact (key scheduling param event-demand args? e ? r ?r))
 )
 (not (wm-fact (key scheduling event args? id ?r type ?)))
=>
 (bind ?start-event-name (sym-cat E- ?r -start))
 (bind ?end-event-name (sym-cat E- ?r -end))
 (assert (wm-fact (key scheduling event args? id ?r type start) (value ?start-event-name)))
 (assert (wm-fact (key scheduling event args? id ?r type end) (value ?end-event-name)))

 (assert (wm-fact (key scheduling param event-supply args? e ?start-event-name r ?r)
                  (type INT) (value 1)))
 (assert (wm-fact (key scheduling param event-demand args? e ?end-event-name r ?r)
                  (type INT) (value 1)))

 (assert (wm-fact (key scheduling dataset resource-producing-events args? r ?r)
                  (is-list TRUE) (type SYMBOL) (values ?start-event-name)))
 (assert (wm-fact (key scheduling dataset resource-consuming-events args? r ?r)
                  (is-list TRUE) (type SYMBOL) (values ?end-event-name)))
)

(defrule scheduling-populate-dataset-resource-conmsuing-events
 (wm-fact (key scheduling param event-demand args? e ?e r ?r))
 ?rset <- (wm-fact (key scheduling dataset resource-consuming-events args? r ?r)
           (values $?events&:(not (member$ ?e ?events))))
=>
 (modify ?rset (values (create$ ?events ?e)))
)

(defrule scheduling-populate-dataset-resource-producing-events
 (wm-fact (key scheduling param event-supply args? e ?e r ?r))
 ?rset <- (wm-fact (key scheduling dataset resource-producing-events args? r ?r)
           (values $?events&:(not ( member$ ?e ?events))))
=>
 (modify ?rset (values (create$ ?events ?e)))
)

;;;Plan Events
(defrule scheduling-plan-events
 (goal (id ?goal-id) (sub-type SIMPLE) (class ?class)(mode EXPANDED))
 (plan (id ?plan-id) (goal-id ?goal-id))
 (wm-fact (key meta precedence goal-class args? $?  ?class $?))
 (wm-fact (key meta plan start-location args? id ?plan-id) (values ?ls ?ls-side))
 (wm-fact (key meta plan end-location args? id ?plan-id) (values ?le ?le-side))
 (wm-fact (key meta plan required-resources args? id ?plan-id) (values $?resources))
 (wm-fact (key scheduling event args? id ?goal-id type end) (value ?e-goal-end))
 (not (wm-fact (key scheduling dataset plan-events args? p ?plan-id)))
 =>
 ;events
 (bind ?e-plan-start (sym-cat E- ?plan-id -start))
 (bind ?e-plan-end   (sym-cat E- ?plan-id -end))
 (assert (wm-fact (key scheduling event args? id ?plan-id type start) (value ?e-plan-start)))
 (assert (wm-fact (key scheduling event args? id ?plan-id type end) (value ?e-plan-end)))
 ;plan-events
 (assert (wm-fact (key scheduling dataset plan-events args? p ?plan-id)
                  (type SYMBOL) (is-list TRUE)
                  (values ?e-plan-start ?e-plan-end)))
 ;Location param
 (assert (wm-fact (key scheduling param event-location args? e ?e-plan-start l (node-name ?ls ?ls-side)))
         (wm-fact (key scheduling param event-location args? e ?e-plan-end l (node-name ?le ?le-side)))
         )
 ;Duration param
 (assert (wm-fact (key scheduling param event-duration args? e ?e-plan-start)
                  (type INT) (value (plan-duration ?plan-id)))
         (wm-fact (key scheduling param event-duration args? e ?e-plan-end)
                  (type INT) (value 0)))
 ;Resources param
 (progn$ (?r ?resources)
   (assert (wm-fact (key scheduling param event-demand args? e ?e-plan-start r ?r) (type INT) (value 1)))
   (assert (wm-fact (key scheduling param event-supply args? e ?e-plan-end r ?r) (type INT) (value 1))))
 ;Precedence within plan
   (assert (wm-fact (key scheduling param event-precedence args? e-a ?e-plan-start e-b ?e-plan-end)))
 ;Precedence plan goals
   (assert (wm-fact (key scheduling param event-precedence args? e-a ?e-plan-end e-b ?e-goal-end)))
)

;Precedence across goals
(defrule scheduling-precedence-across-goals
 (wm-fact (key meta precedence goal-class args? a  ?class-a b ?class-b))
 (goal (id ?g-id-a) (sub-type SIMPLE) (class ?class-a)(mode EXPANDED))
 (goal (id ?g-id-b) (sub-type SIMPLE) (class ?class-b)(mode EXPANDED))
 (plan (id ?p-id-b) (goal-id ?g-id-b))
 (wm-fact (key scheduling event args? id ?g-id-a type end) (value ?e-goal-end))
 (wm-fact (key scheduling event args? id ?p-id-b type start) (value ?e-plan-start))
 (not (wm-fact (key scheduling param event-precedence args? e-a ?e-goal-end e-b ?e-plan-start)))
 =>
 (assert (wm-fact (key scheduling param event-precedence args? e-a ?e-goal-end e-b ?e-plan-start)))
)
