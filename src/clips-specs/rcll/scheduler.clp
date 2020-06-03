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
;
; Schedule Lifecyle:
; Schedules are direcly related to a goal-tree with sub-type SCHEDULE-SUBGOALS.
; Upon expansion of the top most goal with SCHEDULE-SUBGOALS subtype (i.e., the
; schedule root goal), a 'Schedule' fact is created tracking a single scheduling
; attempt (i.e, a single call to the scheduler). A Schedule has lifecyle
; similar to goal-lifecycle, tracked by its
; modes.
;
; FORMULATION [a schedule is created for the schedule root goal]
;  - all sub-goals in the 'SCHEDULE-SUBGOALS' subtree are added to the
;    schedule (goals)
;  - schedule-events and resource requirment are extracted from the tree
;    and the plan resource requirments
;  - the scheduler is called to build the datasets
;
; SELECTION [the scheduler is called to trigger the optimization]
;     - Process the results of the optimization into corresponding schedule-event
;
; EXPANSION [a schedule has been found]
;     - allows reasoning if this schedule should be COMMITTED for executing the
;        goal tree
; COMMITMENT [create setup goals needed for execution, at the right branch of
;              the scheduling goal-tree]
;
; DISPATCH [A schedule is ready to be executed, ground the starting-time]
;
; Upon dispatching of a schedule, the scheduling goal-subtree continue its
; lifecycle, according to the dispatched schedule]


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



(deftemplate scheduler-info
  (slot sched-id (type SYMBOL))
  (slot type (type SYMBOL) (allowed-values EVENT-TIME EVENT-SEQUENCE PLAN-SELECTION))
  (multislot descriptors (type SYMBOL))
  (slot value (type FLOAT))
)

(deftemplate schedule
  (slot id (type SYMBOL))
  (multislot goals (type SYMBOL))
  (slot mode (type SYMBOL))
  (slot duration (type INTEGER))
  (multislot dispatch-time (type INTEGER))
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
  (slot resource-id (type SYMBOL))
  (multislot events (type SYMBOL))
)


(deftemplate resource
  (slot id (type SYMBOL))
  (slot type (type SYMBOL))
  (slot entity (type SYMBOL))
  (slot consumable (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (slot producible (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (multislot states (type SYMBOL))
  (slot units (type INTEGER))
)

(deftemplate resource-setup
  (slot resource-id (type SYMBOL))
  (multislot from-state (type SYMBOL))
  (multislot to-state (type SYMBOL))
  (slot duration (type FLOAT))
)

(deftemplate schedule-requirment
  (slot sched-id (type SYMBOL))
  (slot event-id (type SYMBOL))
  (slot resource-id (type SYMBOL))
  (multislot resource-setup (type SYMBOL))
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
  return (sym-cat ?e)
)

(deffunction formate-resource-name (?n)
  (bind ?r (str-replace ?n "C-" ""))
  (bind ?r (str-replace ?r "M-" ""))
  (bind ?r (formate-event-name ?r))
  ;(if (eq (str-index "$" ?r) FALSE) then
  ;  (bind ?r (str-cat "$" ?r))
  ;)
  return (sym-cat ?r)
)

;; Schedule Lifecycle
(defrule scheduling-create-schedule-goal
"Formulate 'schedule' entity on the expantion of the top most goal of the
the sub-tree with SCHEDULE-SUBGOALS sub-type"
 (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (not (goal (id ?pg) (sub-type SCHEDULE-SUBGOALS)))
 (not (schedule (goals $? ?g-id $?)))
=>
 (assert (schedule (id (sym-cat sched_ (gensym*)))
                   (goals ?g-id)
                   (mode FORMULATED)))
)

(defrule scheduling-goals-to-schedule
"Include all sub-goals of the scheduling subtree in the schedule"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 ?sf <- (schedule (goals $?goals) (mode FORMULATED))
 (goal (id ?pg&:(member$ ?pg ?goals)) (mode EXPANDED))
 (goal (id ?g-id&:(not (member$ ?g-id ?goals))) (parent ?pg) (mode EXPANDED))
=>
 (modify ?sf (goals (create$ ?goals ?g-id)))
)

(defrule scheduling-select-schedule
 " After a 'schedule' events has been formulated and scheduler called to build
   sets, a 'schedule' model is selected and the optimization is triggered "
 ?sf <- (schedule (id ?s-id)(goals ?g-id $?) (mode FORMULATED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
=>
 (printout warn "Calling scheduler: Generating scheduling datasets" crlf)
 (scheduler-generate-model (sym-cat ?s-id))
 (printout warn "datasets are generated" crlf)
;TODO: check for scheduler feature and if usable (maybe by scheduling a simple model)
 (modify ?sf (mode SELECTED))
)

(defrule scheduling-check-optimization-results
"Periodically check if the optimization returned"
(time $?)
 ?sf <- (schedule (id ?s-id) (mode SELECTED))
 (not (scheduler-info))
=>
 (printout info "Calling scheduler: Checking progress" crlf)
 (scheduler-optimization-status (sym-cat ?s-id))
)


(defrule scheduling-expand-schedule
"Expand a schedule after the MIP scheduler has returned a scheduled event "
(declare (salience ?*SALIENCE-GOAL-EXPAND*))
 ?sf <- (schedule (id ?s-id) (goals ?g-id $?) (mode SELECTED))
 (schedule-event (sched-id ?s-id) (scheduled TRUE))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (not (scheduler-info))
=>
 (modify ?sf (mode EXPANDED))
)

(defrule scheduling-commit-schedule
"Commit after reasoning about the EXPANDED schedule"
 ?sf <- (schedule (id ?s-id) (goals ?g-id $?) (mode EXPANDED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 ;reasoning about schedule duration and events, happens here
=>
 (modify ?sf (mode COMMITTED))
)

(defrule scheduling-dispatch-schedule
"Dispatch to schedule by specifing its start-time"
 (time $?now)
 ?sf <- (schedule (id ?s-id) (goals $?goals) (mode COMMITTED))
=>
 (delayed-do-for-all-facts ((?g goal) (?e schedule-event)) (and (member$ ?g:id ?goals)
                                                        (eq ?e:entity ?g:id)
                                                        (eq ?e:at START)
                                                        (eq ?e:scheduled TRUE))
    (modify ?g (meta dispatch-time (+ (nth$ 1 ?now) ?e:scheduled-start 5))))
 (modify ?sf (mode DISPATCHED) (dispatch-time ?now))
)


;; General resource handling
(defrule scheduling-create-resource-source-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode FORMULATED))
 ;resource consumed by schedule
 (schedule-requirment (sched-id ?s-id)
                      (resource-id ?r-id)
                      (resource-units ?v&:(< ?v 0)))
 ;no source event in schedule
 (not (schedule-event (sched-id ?s-id) (entity ?r-id) (at START)))
 ;non producible resource
 (resource (id ?r-id) (producible FALSE) (entity ?entity) (units ?units))
 (or (test (eq ?entity UNKOWN))
     (wm-fact (key domain fact mps-type args? m ?entity t ?type)))
=>
 (bind ?source-id  (sym-cat ?r-id @start))
 (assert
   (schedule-event (sched-id ?s-id) (id ?source-id) (entity ?r-id) (at START))
   (schedule-requirment (sched-id ?s-id)
                        (event-id ?source-id)
                        (resource-id ?r-id)
                        (resource-units ?units)))
)

(defrule scheduling-create-resource-sink-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode FORMULATED))
 ;resourece produced by schedule
 (schedule-requirment (sched-id ?s-id)
                      (resource-id ?r-id)
                      (resource-units ?v&:(> ?v 0)))
 ;no sink event in schedule
 (not  (schedule-event (sched-id ?s-id) (entity ?r-id) (at END)))
 ;non consumable resource
 (resource (id ?r-id) (consumable FALSE) (entity ?entity) (units ?units))
 (or (test (eq ?entity UNKOWN))
     (wm-fact (key domain fact mps-type args? m ?entity t ?type)))
=>
 (bind ?sink-id  (sym-cat ?r-id @end))
 (assert
   (schedule-event (sched-id ?s-id) (id ?sink-id) (entity ?r-id) (at END))
   (schedule-requirment (sched-id ?s-id)
                        (event-id ?sink-id)
                        (resource-id ?r-id)
                        (resource-units (* -1 ?units))))
)

(defrule scheduling-create-resource-source--robot
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode FORMULATED))
 ;resource consumed by schedule
 (schedule-requirment (sched-id ?s-id)
                      (resource-id ?r-id)
                      (resource-units ?v&:(< ?v 0)))
 ;no source event in schedule
 (not (schedule-event (sched-id ?s-id) (entity ?r-id) (at START)))
 ;non producible resource
 (resource (id ?r-id) (producible FALSE) (entity ?robot) (units ?units))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key refbox team-color) (value ?team-color))
=>
 (bind ?source-id  (sym-cat ?r-id @start))

 (bind ?setup (create$ ?curr-location ?curr-side))
 (assert
   (schedule-event (sched-id ?s-id) (id ?source-id) (entity ?r-id) (at START))
   (schedule-requirment (sched-id ?s-id)
                        (event-id ?source-id)
                        (resource-id ?r-id)
                        (resource-setup ?setup)
                        (resource-units ?units)))
)

(defrule scheduling-create-resource-sink--robot
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode FORMULATED))
 ;produced by a schedule event
 (schedule-requirment (sched-id ?s-id)
                      (resource-id ?r-id)
                      (resource-units ?v&:(> ?v 0)))
 ;no sink event for resource in schedule
 (not  (schedule-event (sched-id ?s-id) (entity ?r-id) (at END)))
 (resource (id ?r-id) (consumable FALSE) (entity ?robot)(units ?units))
 ;;TODO: replace with IDLING location of a defaule state ANY
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key refbox team-color) (value ?team-color))
=>
 (bind ?sink-id  (sym-cat ?r-id @end))

 (bind ?setup (create$ ?curr-location ?curr-side))
 (if (eq ?curr-location START) then
   (if (eq ?team-color CYAN) then (bind ?setup "C-ins-in") else (bind ?setup "M-ins-in")))

 (assert
   (schedule-event (sched-id ?s-id) (id ?sink-id) (entity ?r-id) (at END))
   (schedule-requirment (sched-id ?s-id)
                        (event-id ?sink-id)
                        (resource-id ?r-id)
                        (resource-setup ?setup)
                        (resource-units (* -1 ?units))))
)

(defrule scheduling-create-resource-from-req
"Create a reproducable resource by default from requirement "
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-requirment (sched-id ?s-id) (event-id ?e-id) (resource-id ?r-id))
 (not (resource (id ?r-id)))
=>
 (assert (resource (id ?r-id)
                   (units 1)
                   (type UNKOWN)
                   (entity UNKOWN)
                   (consumable FALSE)
                   (producible FALSE)))
)

(defrule scheduling-create-schedule-resource-from-req
"Create schedule-resource to track scheduled sequence of events on each
 resource "
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-requirment (sched-id ?s-id) (event-id ?e-id) (resource-id ?r-id))
 (not (schedule-resource (sched-id ?r-id) (resource-id ?r-id)))
=>
 (assert (schedule-resource (sched-id ?s-id) (resource-id ?r-id)))
)

;We start off by defining the Events that happen at time 0
; (Ex: resource producing events)
(defrule scheduling-init-resources-machines
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 (wm-fact (key refbox team-color) (value ?team-color))
 (wm-fact (key domain fact mps-type args? m ?mps t ?type))
 (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
=>
 (bind ?r (formate-resource-name ?mps))
 (assert (resource (id ?r)
                   (units 1)
                   (type ?type)
                   (entity ?mps)
                   (consumable FALSE)
                   (producible FALSE)))
)

(defrule scheduling-init-resources-robots
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 (wm-fact (key domain fact at args? r ?robot m ? side ?))
 (not (resource (entity ?robot)))
=>
 (bind ?r (formate-resource-name ?robot))
 (assert (resource (id ?r)
                   (units 1)
                   (type ROBOT)
                   (entity ?robot)
                   (consumable FALSE)
                   (producible FALSE)))
)


;;TODO: Error on a requirment's unites more than resource units
;;TODO: Error on requirment with no resource
;;TODO: Error on event requires both -ve and +ve
;;TODO: deduce presedance from tree structure
;;TODO: add sched-id to scheduler-info

;; Building The Scheduling Model
(defrule scheduling-create-goal-event
"Create schedule-events for goals that have a child plan"
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED|COMMITTED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (goal-id ?g-id))
 (not (schedule-event (sched-id ?s-id) (entity ?g-id)))
 =>
 (bind ?goal-start (sym-cat (formate-event-name ?g-id) @start))
 (bind ?goal-end (sym-cat (formate-event-name ?g-id) @end))
 (assert (schedule-event (sched-id ?s-id)
                         (entity ?g-id)
                         (id ?goal-start)
                         (at START)))
 (assert (schedule-event (sched-id ?s-id)
                         (entity ?g-id)
                         (id ?goal-end)
                         (at END)))
)

(defrule scheduling-plan-events
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED|COMMITTED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id))
 (not (schedule-event (sched-id ?s-id) (entity ?p-id)))
 =>
 (bind ?plan-start (sym-cat  (formate-event-name ?p-id) @start))
 (bind ?plan-end   (sym-cat  (formate-event-name ?p-id) @end))
 (bind ?duration (plan-duration ?p-id))
 (assert
  (schedule-event (sched-id ?s-id)
                  (id ?plan-start)
                  (duration ?duration)
                  (entity ?p-id)
                  (at START))

  (schedule-event (sched-id ?s-id)
                  (id ?plan-end)
                  (entity ?p-id)
                  (at END))
 )
)

(defrule scheduling-plan-resources-at-start
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED|COMMITTED))
 (schedule-event (sched-id ?s-id) (id ?e-id) (entity ?p-id) (at START))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id))
 ?pf <- (wm-fact (key meta plan required-resource args? id ?p-id r ?r setup [ $?setup ] ))
 =>
 (bind ?r-id (formate-resource-name ?r))
 (assert (schedule-requirment (sched-id ?s-id)
                              (event-id ?e-id)
                              (resource-id ?r-id)
                              (resource-setup ?setup)
                              (resource-units -1)))
 (retract ?pf)
)

(defrule scheduling-plan-resources-at-end
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED|COMMITTED))
 (schedule-event (sched-id ?s-id) (id ?e-id) (entity ?p-id) (at END))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id))
 ?pf <- (wm-fact (key meta plan released-resource args? id ?p-id r ?r setup [ $?setup ] ))
 =>
 (bind ?r-id (formate-resource-name ?r))
 (assert (schedule-requirment (sched-id ?s-id)
                              (event-id ?e-id)
                              (resource-id ?r-id)
                              (resource-setup ?setup)
                              (resource-units 1)))
 (retract ?pf)
)

;;Schedule Formulate Precedence
(defrule scheduling-create-precedence--goal-plan-start
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?goal-start) (entity ?g-id) (at START))
 (schedule-event (sched-id ?s-id) (id ?plan-start) (entity ?p-id) (at START))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id))
=>
 (assert
  (wm-fact (key scheduling event-precedence args? e-a ?goal-start e-b ?plan-start)))
)

(defrule scheduling-create-precedence--goal-plan-end
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?goal-end) (entity ?g-id) (at END))
 (schedule-event (sched-id ?s-id) (id ?plan-end) (entity ?p-id) (at END))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id))
=>
 (assert
  (wm-fact (key scheduling event-precedence args? e-a ?plan-end e-b ?goal-end)))
)

(defrule scheduling-create-precedence--in-plan
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?plan-start) (entity ?p-id) (at START))
 (schedule-event (sched-id ?s-id) (id ?plan-end) (entity ?p-id) (at END))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id))
=>
 (assert
  (wm-fact (key scheduling event-precedence args? e-a ?plan-start e-b ?plan-end)))
)

(defrule scheduling-create-precedence--across-goals
" From goal (1) end, to  goal (2) start"
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $?goals) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?g1-end) (entity ?g1-id) (at END))
 (schedule-event (sched-id ?s-id) (id ?g2-start) (entity ?g2-id) (at START))

 (goal (id ?g1-id&:(member$ ?g1-id ?goals)) (sub-type SCHEDULE-SUBGOALS)
       (parent ?g2-id))
 (goal (id ?g2-id&:(member$ ?g2-id ?goals)) (sub-type SCHEDULE-SUBGOALS))
 (not (wm-fact (key scheduling event-precedence args? e-a ?g1-end e-b ?g2-start)))
 =>
 (assert (wm-fact (key scheduling event-precedence args? e-a ?g1-end e-b ?g2-start)))
)

(defrule scheduling-resource-setup-duration
 "Calculate Setup duration estimates for resources"
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (resource (id ?r-id) (type ?r-type))
 (schedule-event (sched-id ?s-id) (id ?producer) (entity ?entity-1))
 (schedule-requirment (sched-id ?s-id)
                      (event-id ?producer)
                      (resource-id ?r-id)
                      (resource-units ?v1&:(> ?v1 0))
                      (resource-setup $?setup-1))
 (schedule-event (sched-id ?s-id) (id ?consumer) (entity ?entity-2))
 (schedule-requirment (sched-id ?s-id)
                      (event-id ?consumer)
                      (resource-id ?r-id)
                      (resource-units ?v2&:(< ?v2 0))
                      (resource-setup $?setup-2))
 (not (and (plan (id ?entity-1) (goal-id ?same-goal))
           (plan (id ?entity-2) (goal-id ?same-goal))))
 (not (resource-setup (resource-id ?r-id)
                      (from-state $?setup-1)
                      (to-state $?setup-2)))
 =>
 (bind ?duration 0)
 (if (eq  ?r-type ROBOT) then
     (bind ?duration (estimate-action-duration "move"
                                               (create$ r from from-side to to-side)
                                               (create$ ANY (nth$ 1 ?setup-1) (nth$ 2 ?setup-1)
                                                            (nth$ 1 ?setup-2) (nth$ 2 ?setup-2)))))

 (assert (resource-setup (resource-id ?r-id)
                         (from-state ?setup-1)
                         (to-state ?setup-2)
                         (duration ?duration)))
)


;; Call scheduler to build data sets
(defrule scheduling-add-event-requirment
 (declare (salience ?*SALIENCE-GOAL-SELECT*))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?e-id))
 (schedule-requirment (sched-id ?s-id) (event-id ?e-id)
                      (resource-id ?r-id) (resource-units ?req))
 (resource (id ?r-id))
=>
 (scheduler-add-event-resource (sym-cat ?e-id) (sym-cat ?r-id) ?req)
)

(defrule scheduling-add-event-precedence
 (declare (salience ?*SALIENCE-GOAL-SELECT*))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?e1-id))
 (schedule-event (sched-id ?s-id) (id ?e2-id))
 (wm-fact (key scheduling event-precedence args? e-a ?e1-id e-b ?e2-id))
 =>
 (scheduler-add-event-precedence (sym-cat ?e1-id) (sym-cat ?e2-id))
)

(defrule scheduling-add-goal-event
 (declare (salience ?*SALIENCE-GOAL-SELECT*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?e-id) (entity ?g-id))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 =>
 (scheduler-add-goal-event (sym-cat ?g-id) (sym-cat ?e-id))
)

(defrule scheduling-add-plan-event
 (declare (salience ?*SALIENCE-GOAL-SELECT*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?e-id) (entity ?p-id) (duration ?d))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id))
 =>
 (scheduler-add-plan-event (sym-cat ?p-id) (sym-cat ?e-id))
 ;(scheduler-add-goal-event (sym-cat ?g-id) (sym-cat ?e-id))
 (scheduler-set-event-duration (sym-cat ?e-id) ?d)
)

(defrule scheduling-add-goal-plans
 (declare (salience ?*SALIENCE-GOAL-SELECT*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS))
 (plan (id ?p-id) (goal-id ?g-id))
 =>
 (scheduler-add-goal-plan (sym-cat ?g-id) (sym-cat ?p-id))
)

(defrule scheduling-set-resource-setup-duration
 (declare (salience ?*SALIENCE-GOAL-SELECT*))
 (schedule (id ?s-id) (mode FORMULATED))
 (resource (id ?r-id) (type ?r-type))
 (schedule-event (sched-id ?s-id) (id ?producer) (entity ?entity-1))
 (schedule-requirment (sched-id ?s-id)
                      (event-id ?producer)
                      (resource-id ?r-id)
                      (resource-units ?v1&:(> ?v1 0))
                      (resource-setup $?setup-1))
 (schedule-event (sched-id ?s-id) (id ?consumer) (entity ?entity-2))
 (schedule-requirment (sched-id ?s-id)
                      (event-id ?consumer)
                      (resource-id ?r-id)
                      (resource-units ?v2&:(< ?v2 0))
                      (resource-setup $?setup-2))
 (not (and (plan (id ?entity-1) (goal-id ?same-goal))
           (plan (id ?entity-2) (goal-id ?same-goal))))
 (resource-setup (resource-id ?r-id)
                 (from-state $?setup-1)
                 (to-state $?setup-2)
                 (duration ?duration))
 =>
 (scheduler-set-resource-setup-duration
      (sym-cat ?r-id) (sym-cat ?producer) (sym-cat ?consumer)  ?duration)
 )


;; Process scheduler results
(defrule scheduling-process-scheduled-plans
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (schedule (id ?s-id) (mode SELECTED))
  ?if <- (scheduler-info (type PLAN-SELECTION) (descriptors ?plan-id) (value ?v))
  ?ef <- (schedule-event (sched-id ?s-id) (entity ?plan-id))
  (plan (id ?plan-id) (goal-id ?goal-id))
=>
  (if (> ?v 0) then
    (modify ?ef (scheduled TRUE)))

 (retract ?if)
)

(defrule scheduling-process-scheduled-goals
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (schedule (id ?s-id) (goals $? ?g-id $?) (mode SELECTED))
  (schedule-event (sched-id ?s-id) (entity ?p-id) (scheduled TRUE) (at START))
  ?ef <- (schedule-event (sched-id ?s-id) (entity ?g-id) (scheduled FALSE))
  (goal (id ?g-id))
  (plan (id ?p-id) (goal-id ?g-id))
=>
  (modify ?ef (scheduled TRUE))
)


(defrule scheduling-process-events-start
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode SELECTED))
 ?if <- (scheduler-info (type EVENT-TIME)
                        (descriptors ?e-id)
                        (value ?scheduled-start))
 ?ef <- (schedule-event (sched-id ?s-id) (id ?e-id))
 =>
 (modify ?ef (scheduled-start ?scheduled-start))
 (retract ?if)
)

;TODO: check in consistency in schedule
;  - more that one possible sequence of events for some resource
(defrule scheduling-process-scheduled-edges
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode SELECTED))
 ?if <- (scheduler-info (type EVENT-SEQUENCE)
                        (descriptors ?r-id ?e1-id ?e2-id)
                        (value ?scheduled&:(> ?scheduled 0)))
 (resource (id ?r-id))
 ?ef1 <- (schedule-event (sched-id ?s-id) (id ?e1-id) (entity ?entity1) (at ?at1))
 ?ef2 <- (schedule-event (sched-id ?s-id) (id ?e2-id) (entity ?entity2) (at ?at2))
 (schedule-requirment (sched-id ?s-id) (event-id ?e1-id) (resource-id ?r-id))
 (schedule-requirment (sched-id ?s-id) (event-id ?e2-id) (resource-id ?r-id))
 ?rf <- (schedule-resource (sched-id ?s-id) (resource-id ?r-id)
                           (events $?resource-schedule))
 ;Where e1-id is the first scheduled event on that resource, or already
 ; added to the resource schedule
 (or
  (test (and (eq ?resource-schedule (create$))
             (eq ?entity1 ?r-id)
             (eq ?at1 START)))

  (and (test (eq ?resource-schedule (create$)))
       (not (and (schedule-event (sched-id ?s-id) (entity ?entity1)
                                 (id ?e-id&:(neq ?e1-id ?e-id)))
                 (schedule-requirment (sched-id ?s-id)
                                      (event-id ?e-id)
                                      (resource-id ?r-id)
                                      (resource-units ?consumption&:(< ?consumption 0))))))

  (and (schedule-event (sched-id ?s-id)
                       (entity ?entity1)
                       (id ?e-id&:(and (neq ?e1-id ?e-id)
                                       (member$ ?e-id ?resource-schedule))))
       (schedule-requirment (sched-id ?s-id)
                            (event-id ?e-id)
                            (resource-id ?r-id)
                            (resource-units ?consumed&:(< ?consumed 0))))
 )
=>
  (bind ?resource-schedule (create$ ?resource-schedule ?e1-id ?e2-id))

  (modify ?rf (events ?resource-schedule))
  (modify ?ef1 (scheduled TRUE))
  (modify ?ef2 (scheduled TRUE))

 (retract ?if)
)

(defrule scheduling-process-clean-nonscheduled-sequences
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode SELECTED))
 ?if <- (scheduler-info (type EVENT-SEQUENCE)
                        (descriptors ?r-id ?e1-id ?e2-id)
                        (value ?v&:(<= ?v 0)))
 (resource (id ?r-id))
 (schedule-event (sched-id ?s-id) (id ?e1-id))
 (schedule-event (sched-id ?s-id) (id ?e2-id))
 (schedule-requirment (sched-id ?s-id) (event-id ?e1-id) (resource-id ?r-id))
 (schedule-requirment (sched-id ?s-id) (event-id ?e2-id) (resource-id ?r-id))
=>
 (retract ?if)
)

;; Post processing on COMMITTED schedules
(defrule scheduling-post-processing--create-setup-goals
 "Create a setup goal for each producer-consumer sequence, which requires
  different setup-states for the resource"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 ?sf <- (schedule (id ?s-id) (goals $?goals) (mode COMMITTED))
 ;Producer -> consumer sequence
 (schedule-resource (sched-id ?s-id) (resource-id ?r-id) (events $? ?e1 ?e2 $?))
 ;Scheduled producer CEs
 (schedule-event (id ?e1) (sched-id ?s-id) (scheduled TRUE) (entity ?e1-entity)
                 (duration ?e1-dur) (scheduled-start ?e1-start))
 (schedule-requirment (event-id ?e1)
                      (sched-id ?s-id)
                      (resource-id ?r-id)
                      (resource-setup $?e1-setup)
                      (resource-units ?produced&:(> ?produced 0)))
 ;Scheduled consumer CEs
 (schedule-event (id ?e2) (sched-id ?s-id) (scheduled TRUE) (entity ?e2-entity))
 (schedule-requirment (event-id ?e2)
                      (sched-id ?s-id)
                      (resource-id ?r-id)
                      (resource-setup $?e2-setup&:(neq ?e1-setup ?e2-setup))
                      (resource-units ?consumed&:(< ?consumed 0)))
 ;resource CEs
 (resource-setup (resource-id ?r-id) (duration ?setup-duration)
                 (from-state $?e1-setup) (to-state $?e2-setup))

 ;Insert the setup-goal as a sibiling of the plan that needs the setup
 (plan (id ?e2-entity) (goal-id ?e2-goal))
 ;(goal (id ?e2-goal) (parent ?parent-goal))
 (not (goal (parent ?e2-goal)
            (id ?g-id&:(eq ?g-id (sym-cat SETUP_ ?r-id _ ?e1 _ ?e2)))))
 =>
 (bind ?g-id (sym-cat SETUP_ ?r-id  _ ?e1 _ ?e2))
 (assert (goal (id ?g-id) (parent ?e2-goal)
               (class SETUP) (mode SELECTED)
               (sub-type SCHEDULE-SUBGOALS)
               (params r ?r-id setup1 ?e1-setup setup2 ?e2-setup)
               (meta scheduled-start (+ ?e1-start ?e1-dur))))

(modify ?sf (goals (create$ ?goals ?g-id)))
)

(defrule scheduling-post-processing--schedule-setup-plan-events
 "Schedule setup plan events"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
 ?gsf <- (schedule-event (sched-id ?s-id) (entity ?g-id) (at START)
                         (scheduled FALSE))
 ?gef <- (schedule-event (sched-id ?s-id) (entity ?g-id) (at END)
                         (scheduled FALSE))
 ?pef <- (schedule-event (sched-id ?s-id) (entity ?p-id) (at END)
                         (scheduled FALSE) (duration ?pend-duration))
 ?psf <- (schedule-event (sched-id ?s-id) (entity ?p-id) (at START)
                         (scheduled FALSE) (duration ?pstart-duration))
 (plan (id ?p-id) (goal-id ?g-id))
 (goal (id ?g-id) (class SETUP) (meta scheduled-start ?goal-start))
 =>
 (modify ?gsf (scheduled TRUE)
              (scheduled-start ?goal-start))
 (modify ?psf (scheduled TRUE)
              (scheduled-start ?goal-start))
 (modify ?pef (scheduled TRUE)
              (scheduled-start (+ ?goal-start ?pstart-duration)))
 (modify ?gef (scheduled TRUE)
              (scheduled-start (+ ?goal-start ?pstart-duration ?pend-duration)))
)

(defrule scheduling-post-processing--propogate-goal-START-events
 "Bottom up propagation of start-time events for each goal in the
  scheduling goal tree"
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS))
 (not (schedule-event (sched-id ?s-id) (entity ?g-id) (at START)))

 ;Child with the smallest scheduled start
 ;(or
     (plan (id ?child-id) (goal-id ?g-id))
 ;   (goal (id ?child-id) (parent ?g-id)))
 (schedule-event (sched-id ?s-id) (entity ?child-id) (at START) (scheduled TRUE)
                 (scheduled-start ?child-start))
 ;(forall (goal (id ?sub-goal&:(neq ?child-id ?sub-goal)) (parent ?g-id))
 ;        (schedule-event (sched-id ?s-id) (entity ?sub-goal) (at START)
 ;                        (scheduled TRUE) (scheduled-start ?gt&:(<= ?child-start ?gt))))
 (forall (plan (id ?sub-plan&:(neq ?child-id ?sub-plan)) (goal-id ?g-id))
         (schedule-event (sched-id ?s-id) (entity ?sub-plan) (at START)
                         (scheduled TRUE) (scheduled-start ?pt&:(<= ?child-start ?pt))))
=>
 (assert (schedule-event (id (sym-cat ?g-id @start))
                         (sched-id ?s-id)
                         (entity ?g-id)
                         (at START)
                         (scheduled TRUE)
                         (scheduled-start ?child-start)))
)

(defrule scheduling-post-processing---ground-goal-resource
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
  (schedule-event (sched-id ?s-id) (id ?e-id) (entity ?p-id) (at START)
                  (scheduled TRUE))
  (schedule-requirment (sched-id ?s-id) (event-id ?e-id) (resource-id ?r-id)
                      (resource-units ?u&:(< ?u 0)))
  (plan (id ?p-id) (goal-id ?g-id))
  ?gf <- (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED)
               (required-resources $?req&:(not (member$ ?r-id ?req))))
  =>
  (modify ?gf (required-resources (create$ ?req ?r-id)))
)

(defrule scheduling-post-processing---reject-non-scheduled-plans
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
  (schedule-event (sched-id ?s-id) (id ?e-start) (entity ?p-id) (at START)
                  (scheduled FALSE))
  (plan (id ?p-id) (goal-id ?g-id))
  (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
  =>
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:id ?p-id)
   (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
       (retract ?a))
   (retract ?p))
)

(defrule scheduling-post-processing---ground-free-vars-WP
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  (schedule (id ?s-id) (goals $?goals) (mode COMMITTED))
  ; Binding
  (wm-fact (key domain fact wp-spawned-for args? wp ?bound-wp r ?bound-r))
  (wm-fact (key domain fact at args? r ?bound-r m ? side ?))
  (resource (id ?bound-resource) (entity ?bound-r))
  (goal (required-resources $? ?bound-resource $?)
        (id ?g&:(member$ ?g ?goals)) (class MOUNT-CAP|MOUNT-RING1))
  ; Free
  ?gf <- (wm-fact (key meta grounding wp-spawned-for args? wp ?fvar-wp r ?xvar_r))
  ?rf <- (resource (id ?r-id&:(eq ?r-id (formate-resource-name ?fvar-wp))))
  =>
  (delayed-do-for-all-facts ((?g goal)) (and (member$ ?g:id ?goals)
                                             (member$ ?r-id ?g:required-resources))
    (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?g:id)
      (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
        (modify ?a (param-values (replace-member$ ?a:param-values ?bound-wp ?fvar-wp))))))

  (modify ?rf (entity ?bound-wp))
  (retract ?gf)
)
