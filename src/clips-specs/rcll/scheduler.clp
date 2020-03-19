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
; sets
; (wm-fact (key scheduling events)    (values ))
; (wm-fact (key scheduling goal-plans args? g ?g-id) (values ))
; (wm-fact (key scheduling goal-events args? p ?p-id)) plan-events + lg
; (wm-fact (key scheduling plan-events args? p ?p-id))
; (wm-fact (key scheduling resource-consuming-events args? r ?r-id))
; (wm-fact (key scheduling resource-producing-events args? r ?r-id))
;
; In principle each EVENT has a location, duration, resource consumption or
; supply. Those are tracked by parameters.
; PARAMETERS:
; (wm-fact (key scheduling event-duration args? e ?e-id) (v))
; (wm-fact (key scheduling event-location args? e ?e-id) (v))
; (wm-fact (key scheduling event-requiremnt args? e ?e-id r ?r) (value 1/-1))
; (wm-fact (key scheduling event-precedence args? e-a ?e-before e-b ?after) ())
; (wm-fact (key scheduling traveling-duration args? e-a ?e-id e-b ?e-id) (v))




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

(defrule scheduling-expand-gaol
 ?g <- (goal (id ?g-id) (sub-type SIMPLE) (mode SELECTED) (class ?class))
 (forall (plan (id ?p-id) (goal-id ?g-id))
         (wm-fact (key scheduling goal-plan args? g ?g-id p ?p-id)))
 (wm-fact (key meta precedence goal-class args? $? ?class $?))
  =>
  (modify ?g (mode EXPANDED))
)

;;;Goal Events
(defrule scheduling-goal-completion-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (sub-type SIMPLE) (mode SELECTED) (class ?class))
 (not (wm-fact (key scheduling goal-event args? g ?g-id e ?)))
 ;Is production goal
 (wm-fact (key meta precedence goal-class args? $? ?class $?))
 =>
 (bind ?event-name (sym-cat E- ?g-id -end))
 (assert
   (wm-fact (key scheduling event args? e ?event-name) (type INT))
   (wm-fact (key scheduling goal-event args? g ?g-id e ?event-name)))
)

;(defrule scheduling-plan-events-as-goal-event
;"add plan event as events of the goal events"
; (goal (id ?g-id) (sub-type SIMPLE) (mode SELECTED) (class ?class))
; (plan (id ?p-id) (goal-id ?g-id))
; (wm-fact (key scheduling event args? e ?e))
; (wm-fact (key scheduling plan-event args? p ?p-id e ?e))
; (not (wm-fact (key scheduling goal-event args? g ?g-id e ?e)))
; =>
;)

(defrule scheduling-resource-production-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (wm-fact (key scheduling event-requirment args? e ? r ?r))
 (not (wm-fact (key scheduling resource-producing-event args? e ? r ?r)))
=>
 (bind ?start-event-name (sym-cat E- ?r -start))
 (assert
   (wm-fact (key scheduling event args? e ?start-event-name))
   (wm-fact (key scheduling event-requirment args? e ?start-event-name r ?r)
            (type INT) (value 1))
   (wm-fact (key scheduling resource-producing-event args? e ?start-event-name r ?r))
 )
)

(defrule scheduling-resource-consumption-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (wm-fact (key scheduling event-requirment args? e ? r ?r))
 (not (wm-fact (key scheduling resource-consuming-event args? e ? r ?r)))
=>
 (bind ?end-event-name (sym-cat E- ?r -end))
 (assert
   (wm-fact (key scheduling event args? e ?end-event-name))
   (wm-fact (key scheduling event-requirment args? e ?end-event-name r ?r)
            (type INT) (value -1))
   (wm-fact (key scheduling resource-consuming-event args? e ?end-event-name r ?r))
   )
)

;;;Plan Events
(defrule scheduling-plan-events
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?goal-id) (sub-type SIMPLE) (class ?class) (mode SELECTED))
 (plan (id ?plan-id) (goal-id ?goal-id))
 (wm-fact (key meta precedence goal-class args? $?  ?class $?))
 (wm-fact (key meta plan start-location args? id ?plan-id) (values ?ls ?ls-side))
 (wm-fact (key meta plan end-location args? id ?plan-id) (values ?le ?le-side))
 (wm-fact (key meta plan required-resources args? id ?plan-id) (values $?resources))
 (wm-fact (key scheduling event args? e ?e-goal-end))
 (wm-fact (key scheduling goal-event args? g ?goal-id e ?e-goal-end))
 (not (wm-fact (key scheduling goal-plan args? g ? p ?plan-id)))
 =>
 (assert (wm-fact (key scheduling goal-plan args? g ?goal-id p ?plan-id)))
 ;events
 (bind ?e-plan-start (sym-cat E- ?plan-id -start))
 (bind ?e-plan-end   (sym-cat E- ?plan-id -end))
 ;plan-events
 (assert
    (wm-fact (key scheduling event args? e ?e-plan-start))
    (wm-fact (key scheduling event args? e ?e-plan-end))
    (wm-fact (key scheduling plan-event args? p ?plan-id e ?e-plan-start))
    (wm-fact (key scheduling plan-event args? p ?plan-id e ?e-plan-end))
    (wm-fact (key scheduling goal-event args? g ?goal-id e ?e-plan-start))
    (wm-fact (key scheduling goal-event args? g ?goal-id e ?e-plan-end))
    (wm-fact (key scheduling event-location args? e ?e-plan-start)
             (type SYMBOL) (value (node-name ?ls ?ls-side)))
    (wm-fact (key scheduling event-location args? e ?e-plan-end)
             (type SYMBOL) (value (node-name ?le ?le-side)))
    (wm-fact (key scheduling event-duration args? e ?e-plan-start)
             (type INT) (value (plan-duration ?plan-id)))
    (wm-fact (key scheduling event-duration args? e ?e-plan-end)
             (type INT) (value 0)))
 ;Resources param
 (progn$ (?r ?resources)
   (assert
     (wm-fact (key scheduling event-requirment args? e ?e-plan-start r ?r)
              (type INT) (value -1))
     (wm-fact (key scheduling event-requirment args? e ?e-plan-end r ?r)
              (type INT) (value 1))))
 ;Precedence within plan
 (assert (wm-fact (key scheduling event-precedence args? e-a ?e-plan-start e-b ?e-plan-end)))
 ;Precedence plan goals
 (assert (wm-fact (key scheduling event-precedence args? e-a ?e-plan-end e-b ?e-goal-end)))
)

;Precedence across goals
(defrule scheduling-precedence-across-goals
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id-a) (sub-type SIMPLE) (class ?class-a))
 (goal (id ?g-id-b) (sub-type SIMPLE) (class ?class-b))
 (plan (id ?p-id-b) (goal-id ?g-id-b))
 ;from end of goal 'a'
 (wm-fact (key scheduling event args? e ?e-goal-end))
 (wm-fact (key scheduling goal-event args? g ?g-id-a e ?e-goal-end))
 (not (wm-fact (key scheduling plan-event args? g ?g-id-a e ?e-goal-end)))
 ;to all events of goal 'b'
 (wm-fact (key scheduling event args? e ?e-plan))
 (wm-fact (key scheduling plan-event args? p ?p-id-b e ?e-plan))
 (not (wm-fact (key scheduling event-precedence args? e-a ?e-goal-end e-b ?e-plan)))
 (wm-fact (key meta precedence goal-class args? a  ?class-a b ?class-b))
 =>
 (assert (wm-fact (key scheduling event-precedence args? e-a ?e-goal-end e-b ?e-plan)))
)
