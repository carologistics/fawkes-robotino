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
  (slot value (type INTEGER))
)

(deftemplate schedule
  (slot id (type SYMBOL))
  (multislot goals (type SYMBOL))
  (multislot resources (type SYMBOL))
  (slot mode (type SYMBOL))
  (slot duration (type INTEGER))
  (multislot dispatch-time (type INTEGER))
  (slot scheduler-status (type SYMBOL))
)

(deftemplate schedule-event
  (slot sched-id (type SYMBOL))
  (slot id (type SYMBOL))
  (slot entity (type SYMBOL))
  (slot at (type SYMBOL) (allowed-values START END))
  (slot scheduled (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  (slot scheduled-start (type INTEGER) (default 0))
  (slot lbound (type FLOAT) (default 0.0))
  (slot ubound (type FLOAT) (default 1500.0))
  (slot duration (type INTEGER) (default 0))
)

(deftemplate schedule-resource
  (slot sched-id (type SYMBOL))
  (slot resource-id (type SYMBOL))
  (multislot events (type SYMBOL))
  (slot type (type SYMBOL))
  (slot entity (type SYMBOL))
)

(deftemplate schedule-requirment
  (slot sched-id (type SYMBOL))
  (slot event-id (type SYMBOL))
  (slot resource-type (type SYMBOL))
  (slot resource-entity (type SYMBOL))
  (slot resource-units (type INTEGER))
  (multislot resource-state (type SYMBOL))
  (multislot resource-setup (type SYMBOL))
)

(deftemplate schedule-setup
  (slot sched-id (type SYMBOL))
  (slot resource-id (type SYMBOL))
  (slot from-event (type SYMBOL))
  (slot to-event (type SYMBOL))
  (slot duration (type INTEGER))
  (multislot selection-groupds (type SYMBOL))
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
  ;(bind ?e (str-replace ?e "_" ""))
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


(deffunction create-schedule-resource (?sched-id ?entity ?type)
  (bind ?r-id (formate-resource-name ?entity))
  (if (not (any-factp ((?srf schedule-resource))
                      (eq ?srf:resource-id ?r-id)))
      then
      (assert (schedule-resource (sched-id ?sched-id)
                                 (entity ?entity)
                                 (type ?type))))
)
+
;; Schedule Lifecycle
(defrule scheduling-create-schedule-goal
"Formulate 'schedule' entity on the expantion of the top most goal of the
the sub-tree with SCHEDULE-SUBGOALS sub-type"
 (goal (id ?g-id) (parent ?pg) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (wm-fact (key refbox team-color) (value ?team-color))
 (not (goal (id ?pg) (sub-type SCHEDULE-SUBGOALS)))
 (not (schedule (goals $? ?g-id $?)))
=>
(bind ?resources (create$))
  ;Add the robots
  (do-for-all-facts ((?wm wm-fact)) (eq (wm-key-prefix ?wm:key) (create$ domain fact at))
    (bind ?resources (append$ ?resources (wm-key-arg ?wm:key r)))
  )
  ;Add machines
  (do-for-all-facts ((?wm wm-fact)) (and (eq (wm-key-prefix ?wm:key) (create$ domain fact mps-team))
                                         (eq (wm-key-arg ?wm:key col) ?team-color))
    (bind ?resources (append$ ?resources (wm-key-arg ?wm:key m)))
  )

 (assert (schedule (id (sym-cat sched_ (gensym*)))
                   (goals ?g-id)
                   (resources ?resources)
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
 ?sf <- (schedule (id ?s-id) (mode SELECTED) (scheduler-status ?status&~OPTIMAL|INFEASABL|INF_OR_UNBD))
=>
 (printout info "Calling scheduler: Checking progress" crlf)
 (bind ?new-status (sym-cat (scheduler-optimization-status (sym-cat ?s-id))))
 (if (neq ?status ?new-status) then
     (modify ?sf (scheduler-status ?new-status))
 )
)


(defrule scheduling-expand-schedule
"Expand a schedule after the MIP scheduler has returned a scheduled event "
(declare (salience ?*SALIENCE-GOAL-EXPAND*))
 ?sf <- (schedule (id ?s-id) (goals ?g-id $?) (mode SELECTED)
                  (scheduler-status OPTIMAL))
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
 (delayed-do-for-all-facts ((?g goal) (?es schedule-event) (?ee schedule-event))
                           (and (member$ ?g:id ?goals)
                                (eq ?es:entity ?g:id)
                                (eq ?ee:entity ?g:id)
                                (eq ?es:at START)
                                (eq ?ee:at END)
                                (eq ?es:scheduled TRUE)
                                (eq ?ee:scheduled TRUE))
    (modify ?g (meta scheduled-start (+ (nth$ 1 ?now) ?es:scheduled-start 5)
                     scheduled-finish (+ (nth$ 1 ?now) ?ee:scheduled-start 5))))
 (modify ?sf (mode DISPATCHED) (dispatch-time ?now))
)


;; General resource handling
;We start off by defining the Events that happen at time 0
; (Ex: resource producing events)
(defrule scheduling-init-resources
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 (schedule (id ?s-id) (resources ?$ ?r-entity $?)(mode FORMULATED))
 (domain-object (name ?r-entity) (type ?r-type))
 (not (schedule-resource (sched-id ?s-id) (entity ?r-entity)))
=>
 (bind ?r-id (formate-resource-name ?r-entity))
 (assert (schedule-resource (sched-id ?s-id)
                            (resource-id ?r-id)
                            (type ?r-type)
                            (entity ?r-entity)))
)

(defrule scheduling-create-resource-source-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode FORMULATED))
 ;resource consumed by schedule
 (schedule-requirment (sched-id ?s-id)
                      (resource-entity ?r-entity)
                      (resource-units ?v&:(< ?v 0)))
 ;no source event in schedule
 (not (schedule-event (sched-id ?s-id) (entity ?r-entity) (at START)))
 ;non producible resource
 (schedule-resource (sched-id ?s-id) (resource-id ?r-id) (type ?r-type) (entity ?r-entity))
 (resource-info (type ?r-type) (producible FALSE) (state-preds $?state-preds) (setup-preds $?setup-preds))
=>
 (bind ?source-id  (sym-cat ?r-id @start))
 (bind ?units 1)
 (assert
   (schedule-event (sched-id ?s-id) (id ?source-id) (entity ?r-entity) (at START) (scheduled TRUE)))

 (bind ?resource-state (create$))
 (bind ?resource-setup (create$))
 (do-for-all-facts ((?wd domain-fact))
                   (and (member$ ?wd:name (create$ ?state-preds ?setup-preds))
                        (member$ ?r-entity ?wd:param-values))
   (bind ?statement (create$ [ ?wd:name (delete-member$ ?wd:param-values ?r-entity) ] ))
   (if (member$ ?wd:name ?state-preds) then
       (bind ?resource-state (append$ ?resource-state  ?statement))
   )
   (if (member$ ?wd:name ?setup-preds) then
       (bind ?resource-setup (append$ ?resource-setup  ?statement))
   )
 )

 (assert (schedule-requirment (sched-id ?s-id)
                              (event-id ?source-id)
                              (resource-entity ?r-entity)
                              (resource-type ?r-type)
                              (resource-units ?units)
                              (resource-state ?resource-state)
                              (resource-setup ?resource-setup))
 )
)
(defrule scheduling-create-resource-sink-event
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode FORMULATED))
 ;resourece produced by schedule
 (schedule-requirment (sched-id ?s-id)
                      (resource-entity ?r-entity)
                      (resource-units ?v&:(> ?v 0)))
 ;no sink event in schedule
 (not  (schedule-event (sched-id ?s-id) (entity ?r-entity) (at END)))
 ;non consumable resource
 (schedule-resource (sched-id ?s-id) (resource-id ?r-id) (type ?r-type) (entity ?r-entity))
 (resource-info (type ?r-type) (consumable FALSE) (state-preds $?state-preds) (setup-preds $?setup-preds))
=>
 (bind ?sink-id  (sym-cat ?r-id @end))
 (bind ?units 1)
 (assert
   (schedule-event (sched-id ?s-id) (id ?sink-id) (entity ?r-entity) (at END) (scheduled TRUE)))

 ;Todo, maybe u dont really need a state for the sink and the assumption
 ; should be that that all states consumed needs to had been produced, not the
 ; other way
 (assert  (schedule-requirment (sched-id ?s-id)
                               (event-id ?sink-id)
                               (resource-entity ?r-entity)
                               (resource-type ?r-type)
                               (resource-units (* -1 ?units))))
)

;;TODO: Error on a requirment's unites more than resource units
;;TODO: Error on requirment with no resource
;;TODO: Error on event requires both -ve and +ve
;;TODO: deduce presedance from tree structure
;;TODO: add sched-id to scheduler-info

;; Building The Scheduling Model
(defrule scheduling-create-goal-start-event
"Create schedule-events for goals that have a child plan"
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED|COMMITTED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (goal-id ?g-id))
 (not (schedule-event (sched-id ?s-id) (entity ?g-id) (at START)))
 ;All subgoal has schedule-event end
 (not (and (goal (id ?sub-id) (parent ?g-id))
           (not (schedule-event (entity ?sub-id) (at END)))))
 =>
 (bind ?goal-start (sym-cat (formate-event-name ?g-id) @start))
 (bind ?goal-end (sym-cat (formate-event-name ?g-id) @end))

 (bind ?lbound 0)
 (bind ?dur 0)
 ;lower bound of goal start is the biggest lbound of its sub-goals
 (do-for-all-facts ((?gf goal) (?sef schedule-event)) (and (eq ?sef:entity ?gf:id)
                                                           (eq ?sef:at END)
                                                           (eq ?gf:parent ?g-id))
                   (if (> ?sef:lbound ?lbound)
                       then
                       (bind ?lbound ?sef:lbound)
                       (bind ?dur ?sef:duration)
                       ))

 (assert (schedule-event (sched-id ?s-id)
                         (entity ?g-id)
                         (id ?goal-start)
                         (lbound (+ ?lbound ?dur))
                         (scheduled TRUE)
                         (at START)))
)

(defrule scheduling-plan-events
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED|COMMITTED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id))
 (schedule-event (sched-id ?s-id) (entity ?g-id) (duration ?g-dur) (at START)
                 (lbound ?g-lb))
 (not (schedule-event (sched-id ?s-id) (entity ?p-id)))
 =>
 (bind ?plan-start (sym-cat  (formate-event-name ?p-id)))
 ;(bind ?plan-start (sym-cat  (formate-event-name ?p-id) @start))
 ;(bind ?plan-end   (sym-cat  (formate-event-name ?p-id) @end))
 (bind ?duration (plan-duration ?p-id))
 (assert
  (schedule-event (sched-id ?s-id)
                  (id ?plan-start)
                  (duration ?duration)
                  (lbound (+ ?g-lb ?g-dur))
                  (entity ?p-id)
                  (at START))

 ; (schedule-event (sched-id ?s-id)
 ;                (id ?plan-end)
 ;                 (lbound (+ ?g-lb ?g-dur ?duration))
 ;                 (entity ?p-id)
 ;                 (at END))
 )
)

(defrule scheduling-create-goal-end-event
"Create schedule-events for goals that have a child plan"
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED|COMMITTED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (not (schedule-event (sched-id ?s-id) (entity ?g-id) (at END)))
 ;all sub-plans have schedule-events
 (not (and (plan (id ?p2-id) (goal-id ?g-id))
           (not (schedule-event (entity ?p2-id) (at START)))))
 ;          (not (schedule-event (entity ?p2-id) (at END)))))
 =>
 (bind ?goal-end (sym-cat (formate-event-name ?g-id) @end))

 (bind ?lbound 0)
 ;lbound of goal end is the least of its plans (shortest plan in best case)
 (do-for-all-facts ((?pf plan) (?sef schedule-event)) (and (eq ?sef:entity ?pf:id)
                                                           (eq ?sef:at START)
                                                           (eq ?pf:goal-id ?g-id))
                   (if (or (eq ?lbound 0) (> ?lbound (+ ?sef:lbound ?sef:duration)))
                       then
                       (bind ?lbound (+ ?sef:lbound ?sef:duration))
                       ))

 (assert (schedule-event (sched-id ?s-id)
                         (entity ?g-id)
                         (lbound  ?lbound)
                         (id ?goal-end)
                         (scheduled TRUE)
                         (at END)))
)

(defrule scheduling-plan-resources-requriment
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED|COMMITTED))
 (schedule-event (sched-id ?s-id) (id ?e-id) (entity ?p-id) (at START))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED))
 (plan (id ?p-id) (goal-id ?g-id) (required-resources $? ?r-entity $?))
 (domain-object (name ?r-entity) (type ?r-type))
 (resource-info (type ?r-type) (state-preds $?state-preds) (setup-preds $?setup-preds))
 (wm-fact (key meta plan-resource ?at args? p ?p-id r ?r-entity $?))
 (not (schedule-requirment (sched-id ?s-id) (event-id ?e-id) (resource-entity ?r-entity)
                           (resource-units ?u&:(or (and (eq ?at at-start) (< ?u 0))
                                                   (and (eq ?at at-end) (> ?u 0)))))
 )
 =>
 (bind ?units 1)
 (if (eq ?at at-start) then (bind ?units (* -1 ?units)))
 (bind ?resource-state (create$))
 (bind ?resource-setup (create$))

 (do-for-all-facts ((?wm wm-fact))
                   (and (wm-key-prefix ?wm:key (create$ meta plan-resource ?at args? p ?p-id r ?r-entity))
                        (member$ (first$ (wm-key-arg ?wm:key pred)) ?setup-preds)
                        (eq ?wm:value POSITIVE))
   (bind ?statement (create$ [ (wm-key-arg ?wm:key pred) ] ))
   (bind ?resource-setup (append$ ?resource-setup  ?statement))
 )
 (do-for-all-facts ((?wm wm-fact))
                   (and (wm-key-prefix ?wm:key (create$ meta plan-resource ?at args? p ?p-id r ?r-entity))
                        (member$ (first$ (wm-key-arg ?wm:key pred)) ?state-preds)
                        (eq ?wm:value POSITIVE))
   (bind ?statement (create$ [ (wm-key-arg ?wm:key pred) ] ))
   (bind ?resource-state (append$ ?resource-state  ?statement))
 )

 (assert (schedule-requirment (sched-id ?s-id)
                              (event-id ?e-id)
                              (resource-entity ?r-entity)
                              (resource-type ?r-type)
                              (resource-units ?units)
                              (resource-state ?resource-state)
                              (resource-setup ?resource-setup))
 )

 (if (not (any-factp ((?sr schedule-resource)) (eq ?sr:entity ?r-entity))) then
   (bind ?r-id (formate-resource-name ?r-entity))
   (assert (schedule-resource (sched-id ?s-id) (resource-id ?r-id) (type ?r-type) (entity ?r-entity)))
 )
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
; (schedule-event (sched-id ?s-id) (id ?plan-end) (entity ?p-id) (at END))
 (schedule-event (sched-id ?s-id) (id ?plan-end) (entity ?p-id) (at START))
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

(defrule scheduling-events-setup-duration
 "Calculate Setup duration estimates for resources"
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (resource-info (type ?r-type))
 (schedule-resource (sched-id ?s-id) (type ?r-type) (entity ?r-entity) (resource-id ?r-id))
 (schedule-event (sched-id ?s-id) (id ?producer) (entity ?entity-1))
 (schedule-requirment (sched-id ?s-id)
                      (event-id ?producer)
                      (resource-entity ?r-entity)
                      (resource-type ?r-type)
                      (resource-state $?resource-state-1)
                      (resource-setup $?resource-setup-1)
                      (resource-units ?v1&:(> ?v1 0)))
 (schedule-event (sched-id ?s-id) (id ?consumer) (entity ?entity-2))
 (schedule-requirment (sched-id ?s-id)
                      (event-id ?consumer)
                      (resource-entity ?r-entity)
                      (resource-type ?r-type)
                      (resource-state $?resource-state-2)
                      (resource-setup $?resource-setup-2)
                      (resource-units ?v2&:(< ?v2 0)))
 (not (schedule-setup (sched-id ?s-id) (resource-id ?r-id)
                      (from-event ?producer ) (to-event ?consumer)))
 ;Plans do not to the same goal
 (not (and (plan (id ?entity-1) (goal-id ?same-goal))
           (plan (id ?entity-2) (goal-id ?same-goal))))
 ;Same none-setup state
; (not (and (wm-fact (key sched fact consumption-requirment args? r ?r-id e ?cosumer  pred [ $?pred1 ]))
;           (not (wm-fact (key sched fact production-requirment args? r ?r-id e ?producer pred [ $?pred1])))))

; (not (and (not (wm-fact (key sched fact consumption-requirment args? r ?r-id e ?cosumer  pred [ $?pred2 ])))
;           (wm-fact (key sched fact production-requirment args? r ?r-id e ?producer pred [ $?pred2 ]))))
(test (subsetp ?resource-state-2 ?resource-state-1))
=>
 (bind ?setup-duration (estimate-setup-duration ?r-type ?resource-setup-1 ?resource-setup-2))

 (assert (schedule-setup (sched-id ?s-id)
                         (resource-id ?r-id)
                         (from-event ?producer)
                         (to-event ?consumer)
                         (duration ?setup-duration)))
)

(defrule scheduling-update-resource-states-with-previous-plans
 "Resource-states for an existing "
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (resource-info (type ?r-type) (state-preds $?state-preds))
 (schedule-resource (sched-id ?s-id) (type ?r-type) (entity ?r-entity) (resource-id ?r-id))
 (schedule-event (sched-id ?s-id) (id ?producer) (entity ?entity-1))
 (schedule-requirment (sched-id ?s-id)
                      (event-id ?producer)
                      (resource-entity ?r-entity)
                      (resource-type ?r-type)
                      (resource-state $?
                                      [
                                      $?producer-state&:(and (not (member$ [ ?producer-state ))
                                                             (not (member$ ] ?producer-state )))
                                      ]
                                      $?)
                      (resource-units ?v1&:(> ?v1 0)))
 (schedule-event (sched-id ?s-id) (id ?consumer) (entity ?entity-2))
 ?srf <- (schedule-requirment (sched-id ?s-id)
                      (event-id ?consumer)
                      (resource-entity ?r-entity)
                      (resource-type ?r-type)
                      (resource-state $?consumer-states&:(not (subsetp ?producer-state ?consumer-states)))
                      (resource-units ?v2&:(> ?v2 0)))
 (schedule-setup (sched-id ?s-id) (resource-id ?r-id)
                      (from-event ?producer ) (to-event ?consumer))
 (not (wm-fact (key meta plan-resource at-end args? p ?entity-2 r ?r-entity pred [ $?producer-state ])))
=>
 (modify ?srf (resource-state (append$ ?consumer-states [ ?producer-state ] )) )
)


;; Call scheduler to build data sets
(defrule scheduling-create-event
 (declare (salience ?*SALIENCE-GOAL-SELECT*))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?e-id) (duration ?d) (lbound ?lb) (ubound ?ub))
 =>
 (scheduler-add-atomic-event (sym-cat ?e-id) ?d ?lb ?ub)
)

(defrule scheduling-add-event-selector
 (declare (salience ?*SALIENCE-GOAL-SELECT*))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?e-id) (entity ?selector))
 =>
 (scheduler-set-event-selector (sym-cat ?e-id) (sym-cat ?selector))
)

(defrule scheduling-set-selector-selected
 (declare (salience (- ?*SALIENCE-GOAL-SELECT* 1)))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (entity ?selector-id) (scheduled ?selected))
 =>
 (scheduler-set-selector-selected  (sym-cat ?selector-id) (sym-cat ?selected))
)

(defrule scheduling-add-selector-group-goal-plans
 (declare (salience (- ?*SALIENCE-GOAL-SELECT* 2)))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
 (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS))
 (plan (id ?p-id) (goal-id ?g-id))
 =>
 (scheduler-add-to-select-one-group (sym-cat ?p-id) (sym-cat ?g-id))
)

(defrule scheduling-add-event-requirment
 (declare (salience (-  ?*SALIENCE-GOAL-SELECT* 4)))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?e-id))
 (schedule-requirment (sched-id ?s-id) (event-id ?e-id)
                      (resource-entity ?r-entity) (resource-units ?req))
 (schedule-resource (sched-id ?s-id) (entity ?r-entity) (resource-id ?r-id))
=>
 (scheduler-add-event-resource (sym-cat ?e-id) (sym-cat ?r-id) ?req)
)

(defrule scheduling-add-event-precedence
 (declare (salience (- ?*SALIENCE-GOAL-SELECT* 3)))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-event (sched-id ?s-id) (id ?e1-id))
 (schedule-event (sched-id ?s-id) (id ?e2-id))
 (wm-fact (key scheduling event-precedence args? e-a ?e1-id e-b ?e2-id))
 =>
 (scheduler-add-event-precedence (sym-cat ?e1-id) (sym-cat ?e2-id))
)

(defrule scheduling-set-resource-setup-duration
 (declare (salience (- ?*SALIENCE-GOAL-SELECT* 5)))
 (schedule (id ?s-id) (mode FORMULATED))
 (schedule-setup (sched-id ?s-id) (resource-id ?r-id) (duration ?duration)
                 (from-event ?producer) (to-event ?consumer))
 =>
 (scheduler-set-edge-duration
      (sym-cat ?r-id) (sym-cat ?producer) (sym-cat ?consumer)  ?duration)
 )

;(defrule scheduling-add-edge-selection-producer
; (declare (salience (- ?*SALIENCE-GOAL-SELECT* 6)))
; (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
; (schedule-requirment (sched-id ?s-id)
;                      (event-id ?prod-id)
;                      (resource-entity ?r-entity)
;                      (resource-state $? [ $?prod-state&:(and (not (member$ [ ?prod-state))
;                                                              (not (member$ ] ?prod-state)))  ] $?)
;                      (resource-units ?prod-units&:(> ?prod-units 0)))
; (schedule-resource (sched-id ?s-id) (entity ?r-entity) (resource-id ?r-id))
; (schedule-setup (sched-id ?s-id) (resource-id ?r-id)
;                 (from-event ?prod-id) (to-event ?cons-id))
; =>
; (bind ?statement (create$))
; (progn$ (?p ?prod-state) (bind ?statemnet (sym-cat ?statement ?p)))
; (bind ?selector-name (sym-cat [ ?r-id ] [ ?prod-id ] [ ?statement ] ))
; (scheduler-add-edge-selector (sym-cat ?r-id) (sym-cat ?prod-id) (sym-cat ?cons-id) (sym-cat ?selector-name))
;)

;(defrule scheduling-add-edge-selection-consumer
; (declare (salience (- ?*SALIENCE-GOAL-SELECT* 6)))
; (schedule (id ?s-id) (goals $? ?g-id $?) (mode FORMULATED))
; (schedule-requirment (sched-id ?s-id)
;                      (event-id ?cons-id)
;                      (resource-entity ?r-entity)
;                      (resource-state $? [ $?cons-state&:(and (not (member$ [ ?cons-state))
;                                                              (not (member$ ] ?cons-state)))  ] $?)
;                      (resource-units ?cons-units&:(< ?cons-units 0)))
; (schedule-resource (sched-id ?s-id) (entity ?r-entity) (resource-id ?r-id))
; (schedule-setup (sched-id ?s-id) (resource-id ?r-id)
;                 (from-event ?prod-id) (to-event ?cons-id))
; =>
; (bind ?statement (create$))
; (progn$ (?p ?prod-state) (bind ?statemnet (sym-cat ?statement ?p)))
; (bind ?selector-name (sym-cat [ ?r-id ] [ ?prod-id ] [ ?statement ] ))
; (scheduler-add-edge-selector (sym-cat ?r-id) (sym-cat ?prod-id) (sym-cat ?cons-id) (sym-cat ?selector-name))
;)



;; Process scheduler results
(defrule scheduling-process-scheduled-plans
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (schedule (id ?s-id) (mode SELECTED))
  ?if <- (scheduler-info (type PLAN-SELECTION) (descriptors ?entity) (value ?v))
=>
  (if (> ?v 0)
      then
      (delayed-do-for-all-facts ((?sef schedule-event)) (eq ?sef:entity ?entity)
                        (modify ?sef (scheduled TRUE))))

 (retract ?if)
)

;(defrule scheduling-process-scheduled-goals
;  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
;  (schedule (id ?s-id) (goals $? ?g-id $?) (mode SELECTED))
;  (schedule-event (sched-id ?s-id) (entity ?p-id) (scheduled TRUE) (at START))
;  ?ef <- (schedule-event (sched-id ?s-id) (entity ?g-id) (scheduled FALSE))
;  (goal (id ?g-id))
;  (plan (id ?p-id) (goal-id ?g-id))
;=>
;  (modify ?ef (scheduled TRUE))
;)


(defrule scheduling-process-events-start
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode SELECTED))
 ?if <- (scheduler-info (type EVENT-TIME)
                        (descriptors ?e-id)
                        (value ?scheduled-start))
 ?ef <- (schedule-event (sched-id ?s-id) (id ?e-id))
 =>
 ;the comparison is needed to exclude the -0.0 values coming from the scheduler
 (if (> ?scheduled-start 0.0) then
     (modify ?ef (scheduled-start (abs ?scheduled-start))))
 (retract ?if)
)

;TODO: check in consistency in schedule
;  - more that one possible sequence of events for some resource
(defrule scheduling-process-scheduled-edges
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode SELECTED))
 (not (scheduler-info (type EVENT-TIME)))
 (not (scheduler-info (type PLAN-SELECTION)))
 ?if <- (scheduler-info (type EVENT-SEQUENCE) (value ?v&:(> ?v 0))
                        (descriptors ?r-id ?e1-id ?e2-id))
  ?e1f <- (schedule-event (sched-id ?s-id)
                          (id ?e1-id)
                          (scheduled ?e1-scheduled)
                          (scheduled-start ?e1-time))
 ?e2f <- (schedule-event (sched-id ?s-id)
                         (id ?e2-id)
                         (scheduled ?e2-scheduled)
                         (scheduled-start ?e2-time))
 (schedule-requirment (resource-entity ?r-entity) (sched-id ?s-id) (event-id ?e1-id))
 (schedule-requirment (resource-entity ?r-entity) (sched-id ?s-id) (event-id ?e2-id))
 ?rf <- (schedule-resource (entity ?r-entity) (resource-id ?r-id) (sched-id ?s-id) (events $?sched-events))
 ;(not (and  (scheduler-info (type EVENT-SEQUENCE) (value ?xv&:(> ?xv 0))
 ;                           (descriptors ?r-id ?ex1-id ?ex2-id))
 ;           (schedule-event (id ?ex1-id)
 ;                           (scheduled-start ?ex1-time&:(> ?e1-time ?ex1-time)))))
 (or (test (eq (length$ ?sched-events) 0))
     (test (member$ ?e1-id ?sched-events))
     (test (member$ ?e2-id ?sched-events)))
=>
 (if (not ?e1-scheduled) then
     (printout error "Event "  ?e1-id " appears in edge but was not scheduled" crlf)
     (modify ?e1f (scheduled TRUE)))
 (if (not ?e2-scheduled) then
     (printout error "Event "  ?e2-id " appears in edge but was not scheduled" crlf)
     (modify ?e2f (scheduled TRUE)))
 ;(if (not (member$ ?e1-id ?sched-events))
 ;    then (bind ?sched-events (append$ ?sched-events ?e1-id ))
 ;    (printout t "SCHEDULE [" ?r-id "] "  ?e1-id " " ?e1-time crlf)
 ;    )
 ;(if (not (member$ ?e2-id ?sched-events))
 ;    then (bind ?sched-events (append$ ?sched-events ?e2-id ))
 ;    (printout t "SCHEDULE [" ?r-id "] "?e2-id " " ?e2-time crlf)
;)
 (bind ?first-index (member$ ?e1-id ?sched-events))
 (bind ?second-index (member$ ?e2-id ?sched-events))
 (if (and (not ?first-index) (not ?second-index)) then
     (bind ?sched-events (create$ ?e1-id ?e2-id)))
 (if (and ?first-index (not ?second-index)) then
     (bind ?sched-events (insert$ ?sched-events (+ ?first-index 1) ?e2-id)))
 (if (and ?second-index (not ?first-index)) then
     (bind ?sched-events (insert$ ?sched-events ?second-index ?e1-id)))


 (modify ?rf (events ?sched-events))
 (retract ?if)
)

(defrule scheduling-process-clean-nonscheduled-sequences
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (schedule (id ?s-id) (mode SELECTED))
 ?if <- (scheduler-info (type EVENT-SEQUENCE)
                        (descriptors ?r-id ?e1-id ?e2-id)
                        (value ?v&:(<= ?v 0)))
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
 (schedule-resource (sched-id ?s-id) (type ?r-type) (entity ?r-entity)
                    (resource-id ?r-id) (events $? ?e1 ?e2 $?))
 ;Scheduled producer CEs
 (schedule-event (id ?e1) (sched-id ?s-id) (scheduled TRUE) (entity ?e1-entity)
                 (duration ?e1-dur) (scheduled-start ?e1-start))
 (schedule-requirment (event-id ?e1)
                      (sched-id ?s-id)
                      (resource-entity ?r-entity)
                      (resource-setup $?e1-setup)
                      (resource-units ?produced&:(> ?produced 0)))
 ;Scheduled consumer CEs
 (schedule-event (id ?e2) (sched-id ?s-id) (scheduled TRUE) (entity ?e2-entity))
 (schedule-requirment (event-id ?e2)
                      (sched-id ?s-id)
                      (resource-entity ?r-entity)
                      (resource-setup $?e2-setup&:(neq ?e1-setup ?e2-setup))
                      (resource-units ?consumed&:(< ?consumed 0)))
 ;Insert the setup-goal as a sibiling of the plan that needs the setup
 (plan (id ?e2-entity) (goal-id ?e2-goal))
 ;(goal (id ?e2-goal) (parent ?parent-goal))
 (not (goal (parent ?e2-goal)
            (id ?g-id&:(eq ?g-id (sym-cat SETUP [ ?r-id ] [ ?e1 ] [ ?e2 ] ))))
 )
 =>
 (if (member$ [ ?e2-setup) then
     (progn$ (?opening-i (create$ (member$ [ ?e2-setup)))
       (bind ?pred-name (nth$ (+ 1 ?opening-i) ?e2-setup))
       (bind ?setup-state-to (statement-by-name ?e2-setup ?pred-name ))
       (bind ?setup-state-from (statement-by-name ?e1-setup ?pred-name ))

       (bind ?g-id (sym-cat SETUP [ ?r-id ] [ ?e1 ] [ ?e2 ] ))
       (assert (goal (id ?g-id)
                     (parent ?e2-goal)
                     (class SETUP)
                     (mode SELECTED)
                     (sub-type SCHEDULE-SUBGOALS)
                     (params r ?r-entity
                             setup1 (rest$ ?setup-state-from)
                             setup2 (rest$ ?setup-state-to))
               (meta last-goal-end (+ ?e1-start ?e1-dur))))

       (modify ?sf (goals (create$ ?goals ?g-id)))
     )
    )
)

(defrule scheduling-post-processing--schedule-setup-plan-events
 "Schedule setup plan events"
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
 ?gsf <- (schedule-event (sched-id ?s-id) (entity ?g-id) (at START))
 ?gef <- (schedule-event (sched-id ?s-id) (entity ?g-id) (at END))
 ;?pef <- (schedule-event (sched-id ?s-id) (entity ?p-id) (at END)
 ;                        (scheduled FALSE) (duration ?pend-duration))
 ?psf <- (schedule-event (sched-id ?s-id) (entity ?p-id) (at START)
                         (scheduled FALSE) (duration ?pstart-duration))
 (plan (id ?p-id) (goal-id ?g-id))
 (goal (id ?g-id) (class SETUP) (meta last-goal-end ?goal-start))
 =>
 (modify ?gsf (scheduled-start ?goal-start))
 (modify ?psf (scheduled TRUE)
              (scheduled-start ?goal-start))
 ;(modify ?pef (scheduled TRUE)
 ;             (scheduled-start (+ ?goal-start ?pstart-duration)))
 ;(modify ?gef (scheduled TRUE)
 ;             (scheduled-start (+ ?goal-start ?pstart-duration ?pend-duration)))
 (modify ?gef (scheduled-start (+ ?goal-start ?pstart-duration)))
)

(defrule scheduling-post-processing---elevate-plan-resource
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
  (schedule-event (sched-id ?s-id) (id ?e-id) (entity ?p-id) (at START)
                  (scheduled TRUE))
  (schedule-requirment (sched-id ?s-id) (event-id ?e-id)
                       (resource-entity ?r-entity)
                       (resource-units ?u&:(< ?u 0)))
  (plan (id ?p-id) (goal-id ?g-id))
  ?gf <- (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS) (mode EXPANDED)
               (required-resources $?req&:(not (member$ ?r-entity ?req))))
  =>
  (modify ?gf (required-resources (create$ ?req ?r-entity)))
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

;(defrule scheduling-post-processing--propogate-goal-START-events
; "Bottom up propagation of start-time events for each goal in the
;  scheduling goal tree"
; (declare (salience ?*SALIENCE-GOAL-EXPAND*))
; (schedule (id ?s-id) (goals $? ?g-id $?) (mode COMMITTED))
; (goal (id ?g-id) (sub-type SCHEDULE-SUBGOALS))
; (not (schedule-event (sched-id ?s-id) (entity ?g-id) (at START)))
;
; ;Child with the smallest scheduled start
; ;(or
;     (plan (id ?child-id) (goal-id ?g-id))
; ;   (goal (id ?child-id) (parent ?g-id)))
; (schedule-event (sched-id ?s-id) (entity ?child-id) (at START) (scheduled TRUE)
;                 (scheduled-start ?child-start))
; ;(forall (goal (id ?sub-goal&:(neq ?child-id ?sub-goal)) (parent ?g-id))
; ;        (schedule-event (sched-id ?s-id) (entity ?sub-goal) (at START)
; ;                        (scheduled TRUE) (scheduled-start ?gt&:(<= ?child-start ?gt))))
; (forall (plan (id ?sub-plan&:(neq ?child-id ?sub-plan)) (goal-id ?g-id))
;         (schedule-event (sched-id ?s-id) (entity ?sub-plan) (at START)
;                         (scheduled TRUE) (scheduled-start ?pt&:(<= ?child-start ?pt))))
;=>
; (assert (schedule-event (id (sym-cat ?g-id @start))
;                         (sched-id ?s-id)
;                         (entity ?g-id)
;                         (at START)
;                         (scheduled TRUE)
;                         (scheduled-start ?child-start)))
;)


;(defrule scheduling-post-processing---ground-free-vars-WP
;  (declare (salience ?*SALIENCE-GOAL-SELECT*))
;  (schedule (id ?s-id) (goals $?goals) (mode COMMITTED))
;  ; Binding
;  (wm-fact (key domain fact wp-spawned-for args? wp ?bound-wp r ?bound-r))
;  (wm-fact (key domain fact at args? r ?bound-r m ? side ?))
;  (resource (id ?bound-resource) (entity ?bound-r))
;  (goal (required-resources $? ?bound-resource $?)
;        (id ?g&:(member$ ?g ?goals)) (class MOUNT-CAP|MOUNT-RING1))
;  ; Free
;  ?gf <- (wm-fact (key meta grounding wp-spawned-for args? wp ?fvar-wp r ?xvar_r))
;  ?rf <- (resource (id ?r-id&:(eq ?r-id (formate-resource-name ?fvar-wp))))
;  =>
;  (delayed-do-for-all-facts ((?g goal)) (and (member$ ?g:id ?goals)
;                                             (member$ ?r-id ?g:required-resources))
;    (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?g:id)
;      (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
;        (modify ?a (param-values (replace-member$ ?a:param-values ?bound-wp ?fvar-wp))))))
;
;  (modify ?rf (entity ?bound-wp))
;  (retract ?gf)
;)
