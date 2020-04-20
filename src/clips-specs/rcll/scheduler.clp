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


(deftemplate scheduler-info
  (slot sched-id (type SYMBOL))
  (slot type (type SYMBOL) (allowed-values EVENT-TIME EVENT-SEQUENCE PLAN-SELECTION))
  (multislot descriptors (type SYMBOL))
  (slot value (type FLOAT))
)

(deftemplate schedule
  (slot id (type SYMBOL))
  (slot goal-id (type SYMBOL))
  (slot mode (type SYMBOL))
  (slot duration (type INTEGER))
  (slot start-time (type INTEGER))
)

(deftemplate schedule-event
  (slot sched-id (type SYMBOL))
  (slot id (type SYMBOL))
  (slot entity (type SYMBOL))
  (slot at (type SYMBOL) (allowed-values START END))
  (slot scheduled (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (slot scheduled-start (type INTEGER) (default 0))
  (slot duration (type INTEGER) (default 0))
)


(deftemplate schedule-resource
  (slot sched-id (type SYMBOL))
  (slot id (type SYMBOL))
  (slot type (type SYMBOL))
  (slot consumable (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (slot producible (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (multislot states (type SYMBOL))
  (slot units (type INTEGER))
)

(deftemplate schedule-requirment
  (slot sched-id (type SYMBOL))
  (slot event-id (type SYMBOL))
  (slot resource-id (type SYMBOL))
  (slot resource-setup (type SYMBOL))
  (slot resource-units (type INTEGER))
)

(deffunction plan-duration (?plan-id)
 (bind ?duration 0)
 (do-for-all-facts ((?pa plan-action))
                    (eq ?pa:plan-id ?plan-id)
    (bind ?duration (+ ?duration ?pa:duration))
  )
  (return ?duration)
)

(deffunction formate-event-name (?e)
  (bind ?e (str-replace ?e "-" ""))
  (bind ?e (str-replace ?e "_" ""))
  return ?e
)

(deffunction formate-resource-name (?n)
  (bind ?r (str-replace ?n "C-" ""))
  (bind ?r (str-replace ?r "M-" ""))
  (bind ?r (formate-event-name ?r))
  ;(if (eq (str-index "$" ?r) FALSE) then
  ;  (bind ?r (str-cat "$" ?r))
  ;)
  return ?r
)

(defrule scheduling-create-resource-source-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule-resource (id ?r-id) (producible FALSE) (units ?units))
=>
 (bind ?event-id  (sym-cat ?r-id @start))
 (assert
   (schedule-event (id ?event-id) (entity ?r-id) (at START))
   (schedule-requirment (event-id ?event-id)
                        (resource-id ?r-id)
                        (resource-units ?units)))
)

(defrule scheduling-create-resource-sink-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule-resource (id ?r-id) (consumable FALSE) (units ?units))
=>
 (bind ?event-id  (sym-cat ?r-id @end))
 (assert
   (schedule-event (id ?event-id) (entity ?r-id) (at END))
   (schedule-requirment (event-id ?event-id)
                        (resource-id ?r-id)
                        (resource-units (* -1 ?units))))
)


(defrule scheduling-create-resource-from-req
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule-requirment (event-id ?event-id) (resource-id ?r-id) )
 (not (schedule-resource (id ?r-id)))
=>
 (assert (schedule-resource (id ?r-id)
                            (units 1)
                            (type WP)
                            (consumable FALSE)
                            (producible FALSE)))
)

;We start off by defining the Events that happen at time 0
; (Ex: resource producing events)
(defrule scheduling-init-resources-machines
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (wm-fact (key refbox team-color) (value ?team-color))
 (wm-fact (key domain fact mps-type args? m ?mps t ?type))
 (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
=>
 (bind ?r (formate-resource-name ?mps))
 (assert (schedule-resource (id ?r)
                            (units 1)
                            (type ?type)
                            (consumable FALSE)
                            (producible FALSE)))
)

(defrule scheduling-init-resources-robots
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (wm-fact (key domain fact self args? r ?robot))
=>
 (bind ?r (formate-resource-name ?robot))
 (assert (schedule-resource (id ?r)
                            (units 1)
                            (type ROBOT)
                            (consumable FALSE)
                            (producible FALSE)))
)


;;TODO: Error on requirment with no resource
;;TODO: Error on event requires both -ve and +ve

(defrule scheduling-goal-class-precedence
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

(defrule scheduling-goal-events
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
 (plan (goal-id ?g-id))
 (not (schedule-event (entity ?g-id)))
 =>
 (bind ?goal-end (sym-cat (formate-event-name ?g-id) @end))
 (assert (schedule-event (id ?goal-end) (entity ?g-id) (at END)))
)

(defrule scheduling-plan-events
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
 (plan (id ?p-id) (goal-id ?g-id))
 (not (schedule-event (entity ?p-id)))
 =>
 (bind ?plan-start (sym-cat  (formate-event-name ?p-id) @start))
 (bind ?plan-end   (sym-cat  (formate-event-name ?p-id) @end))
 (bind ?duration (plan-duration ?p-id))
 (assert
  (schedule-event (id ?plan-start) (entity ?p-id) (at START) (duration ?duration))
  (schedule-event (id ?plan-end) (entity ?p-id) (at END))
 )
)

(defrule scheduling-plan-resources-at-start
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
 (plan (id ?p-id) (goal-id ?g-id))
 (schedule-event (id ?e-id) (entity ?p-id) (at START))
 (wm-fact (key meta plan resource-consumed args? id ?p-id r ?r setup ?setup))
 =>
  ;(wm-fact (key scheduling event-location args? e ?plan-end)
    ;         (type SYMBOL) (value (node-name ?le ?le-side)))
   (bind ?r-id (formate-resource-name ?r))
   (assert (schedule-requirment (event-id ?e-id)
                                (resource-id ?r-id)
                                (resource-setup ?setup)
                                (resource-units -1)))
)

(defrule scheduling-plan-resources-at-end
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
 (plan (id ?p-id) (goal-id ?g-id))
 (schedule-event (id ?e-id) (entity ?p-id) (at END))
 (wm-fact (key meta plan resource-produced args? id ?p-id r ?r setup ?setup))
 =>
  ;(wm-fact (key scheduling event-location args? e ?plan-end)
    ;         (type SYMBOL) (value (node-name ?le ?le-side)))
   (bind ?r-id (formate-resource-name ?r))
   (assert (schedule-requirment (event-id ?e-id)
                                (resource-id ?r-id)
                                (resource-setup ?setup)
                                (resource-units 1)))
)

;Precedence across goals
(defrule scheduling-goal-plan-precedence
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
 (plan (id ?p-id) (goal-id ?g-id))
 (schedule-event (id ?goal-end) (entity ?g-id) (at END))
 (schedule-event (id ?plan-end) (entity ?p-id) (at END))
 =>
 (assert
  (wm-fact (key scheduling event-precedence args? e-a ?plan-end e-b ?goal-end)))
)

(defrule scheduling-in-plan-precedence
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode SELECTED))
 (plan (id ?p-id) (goal-id ?g-id))
 (schedule-event (id ?plan-start) (entity ?p-id) (at START))
 (schedule-event (id ?plan-end) (entity ?p-id) (at END))
=>
 (assert
  (wm-fact (key scheduling event-precedence args? e-a ?plan-start e-b ?plan-end)))
)

(defrule scheduling-precedence-across-goals
" From goal (1) plans end, to  goal (2) plans start"
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (wm-fact (key meta precedence goal-class args? a ?class-1 b ?class-2))
 (goal (id ?g1-id) (sub-type SCHEDULE-SUBGOALS) (class ?class-1))
 (goal (id ?g2-id) (sub-type SCHEDULE-SUBGOALS) (class ?class-2))
 (plan (id ?p1-id) (goal-id ?g1-id))
 (plan (id ?p2-id) (goal-id ?g2-id))
 (schedule-event (id ?p1-end) (entity ?p1-id) (at END))
 (schedule-event (id ?p2-start) (entity ?p2-id) (at START))
 (not (wm-fact (key scheduling event-precedence args? e-a ?p1-end e-b ?p2-start)))
 =>
 (assert (wm-fact (key scheduling event-precedence args? e-a ?p1-end e-b ?p2-start)))
)

;Resource setup times between 2 events
(defrule scheduling-resource-setup-duration
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule-resource (id ?r-id) (type ?r-type))
 (schedule-event (id ?producer) (entity ?entity-1))
 (schedule-requirment (event-id ?producer)
                      (resource-id ?r-id)
                      (resource-units ?v1&:(> ?v1 0))
                      (resource-setup ?setup-1))
 (schedule-event (id ?consumer) (entity ?entity-2))
 (schedule-requirment (event-id ?consumer)
                      (resource-id ?r-id)
                      (resource-units ?v2&:(< ?v2 0))
                      (resource-setup ?setup-2))
 (not (and (plan (id ?entity-1) (goal-id ?same-goal))
           (plan (id ?entity-2) (goal-id ?same-goal))))
 (not (wm-fact (key scheduling setup-duration args? r ?r e-a ?producer e-b ?consumer)))
 =>
 (bind ?duration 0)
 (if (eq  ?r-type ROBOT) then
      (bind ?duration (/ (nodes-distance ?setup-1 ?setup-2) ?*V*)))
 )
 (assert (wm-fact (key scheduling setup-duration args? r ?r e-a ?producer e-b ?consumer)
                  (type INT) (value ?duration)))
)


;; Scheduler Calls
(defrule scheduling-add-event-requirment
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule-event (id ?e-id))
 (schedule-resource (id ?r-id))
 (schedule-requirment (event-id ?e-d) (resource-id ?r-id) (resource-units ?req))
=>
 (scheduler-add-event-resource (sym-cat ?e-id) (sym-cat ?r-id) ?req)
)

(defrule scheduling-add-event-precedence
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule-event (id ?e1-id))
 (schedule-event (id ?e2-id))
 (wm-fact (key scheduling event-precedence args? e-a ?e1-id e-b ?e2-id))
 =>
 (scheduler-add-event-precedence (sym-cat ?e1-id) (sym-cat ?e2-id))
)

(defrule scheduling-add-goal-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS))
 (schedule-event (id ?e-id) (entity ?g-id))
 =>
 (scheduler-add-goal-event (sym-cat ?g-id) (sym-cat ?e-id))
)

(defrule scheduling-add-plan-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule-event (id ?e-id) (entity ?p-id) (duration ?d))
 (plan (id ?p-id) (goal-id ?g-id))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS))
 =>
 (scheduler-add-plan-event (sym-cat ?p-id) (sym-cat ?e-id))
 (scheduler-add-goal-event (sym-cat ?g-id) (sym-cat ?e-id))
 (scheduler-set-event-duration (sym-cat ?e-id) ?d)
)

(defrule scheduling-add-goal-plans
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (plan (id ?p-id) (goal-id ?g-id))
 (goal (id ?g-id) (sub-type SCHEDULED-SUBGOAL))
 =>
 (scheduler-add-goal-plan (sym-cat ?g-id) (sym-cat ?p-id))
)

(defrule scheduling-set-resource-setup-duration
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule-event (id ?producer))
 (schedule-event (id ?consumer))
 (wm-fact (key scheduling setup-duration args? r ?r e-a ?producer e-b ?consumer)
           (value ?setup))
 =>
 (scheduler-set-resource-setup-duration
      (sym-cat ?r) (sym-cat ?producer) (sym-cat ?consumer) ?setup )
 )


(defrule shceduling-call-scheduler
 (goal (class ORDER) (mode EXPANDED))
 =>
 (printout warn "Calling scheduler: Generating scheduling datasets" crlf)
 (scheduler-generate-model)
 (printout warn "datasets are generated" crlf)
)
