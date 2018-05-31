;---------------------------------------------------------------------------
;  goal-reasoner.clp - Goal reasoning for RCLL domain
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

(deftemplate goal-meta
  (slot goal-id (type SYMBOL))
  (slot num-tries (type INTEGER))
  (slot max-tries (type INTEGER))
  (multislot last-achieve (type INTEGER) (cardinality 2 2) (default 0 0))
)

(defglobal
  ?*GOAL-MAX-TRIES* = 3
  ?*SALIENCE-GOAL-FORMULATE* = 500
  ?*SALIENCE-GOAL-REJECT* = 400
  ?*SALIENCE-GOAL-EXPAND* = 300
  ?*SALIENCE-GOAL-SELECT* = 200
  ; common evaluate rules should have
  ;   lower salience than case specific ones
  ?*SALIENCE-GOAL-EVALUTATE-GENERIC* = -1
)

; #  Goal Selection
; We can choose one or more goals for expansion, e.g., calling
; a planner to determine the required steps.
(defrule goal-reasoner-select-goal
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (id ?goal-id) (mode FORMULATED))
   =>
  (modify ?g (mode SELECTED))
  (assert (goal-meta (goal-id ?goal-id) (max-tries ?*GOAL-MAX-TRIES*)))
)

;Select subgoal only when parent goal is expanded
(defrule goal-reasoner-select-subgoal
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?p <- (goal (id ?parent-id) (mode EXPANDED))
  ?g <- (goal (parent ?parent-id) (mode FORMULATED)
          (id ?subgoal-id) (priority ?priority))
  ;Select the formulated subgoal with the highest priority
  (not (goal (parent ?parent-id) (mode FORMULATED)
             (priority ?h-priority&:(> ?h-priority ?priority)))
  )
  ;No other subgoal being processed
  (not (goal (parent ?parent-id)
             (mode SELECTED|EXPANDED|
                   COMMITTED|DISPATCHED|
                   FINISHED|EVALUATED))
  )
=>
  ;(printout t "Goal " ?subgoal-id " selected!" crlf)
  (modify ?g (mode SELECTED))
  (assert (goal-meta (goal-id ?subgoal-id)))
)

; # Expand a parent goal
;Expanding a parent goal means it has some formulated goals
(defrule goal-reasoner-expand-parent-goal
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?p <- (goal (id ?parent-id) (mode SELECTED))
  ?g <- (goal (parent ?parent-id) (mode FORMULATED))
  =>
  (modify ?p (mode EXPANDED))
)

; #  Commit to goal (we "intend" it)
; Do not just commit to a goal, we first need to get all locks.
;(defrule goal-reasoner-commit
;  ?g <- (goal (mode EXPANDED))
;  =>
;  (modify ?g (mode COMMITTED))
;)

; #  Dispatch goal (action selection and execution now kick in)
; Trigger execution of a plan. We may commit to multiple plans
; (for different goals), e.g., one per robot, or for multiple
; orders. It is then up to action selection and execution to determine
; what to do when.
(defrule goal-reasoner-dispatch
  ?g <- (goal (mode COMMITTED))
  =>
  (modify ?g (mode DISPATCHED))
)

(defrule goal-reasoner-finish-parent-goal
  ?pg <- (goal (id ?pg-id) (mode DISPATCHED))
  ?sg <- (goal (id ?sg-id) (parent ?pg-id) (mode EVALUATED) (outcome ?outcome))
  (time $?now)
  =>
  (printout debug "Goal '" ?pg-id " finised and " ?outcome "cause " ?sg-id
    "' has been Evaluated" crlf)
  ;Finish the parent goal with the same outcome
  ;Could be extended later for custom behavior
  ;in case we want to try something else to achive that goal
  (modify ?pg (mode FINISHED) (outcome ?outcome))
)

(defrule goal-reasoner-fail-parent-goal-if-subgoals-rejected
  ?pg <- (goal (id ?pg-id) (mode DISPATCHED))
  (not (goal (id ?sg-id) (parent ?pg-id) (mode ~REJECTED)))
  (goal (parent ?pg-id))
  (time $?now)
  =>
;  (printout debug "Goal '" ?pg-id " failed because all subgoald been REJECTED" crlf)
  (modify ?pg (mode FINISHED) (outcome FAILED))
)

(defrule goal-reasoner-cleanup-rejected-goal
  ?g <- (goal (id ?goal-id) (mode REJECTED))
  =>
  (do-for-all-facts ((?plan plan)) (eq ?plan:goal-id ?goal-id)
    (do-for-all-facts
      ((?action plan-action))
      (and (eq ?action:goal-id ?goal-id) (eq ?action:plan-id ?plan:id))
      (retract ?action)
    )
    (retract ?plan)
  )
)

; #  Goal Monitoring
; ## Goal Evaluation
(defrule goal-reasoner-evaluate-subgoal-common
  (declare (salience ?*SALIENCE-GOAL-EVALUTATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id&~nil) (mode FINISHED) (outcome ?outcome))
  ?pg <- (goal (id ?parent-id))
  ?m <- (goal-meta (goal-id ?parent-id))
  (time $?now)
  =>
;  (printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
;    "') has been completed, Evaluating" crlf)
  (modify ?g (mode EVALUATED))
  (modify ?m (last-achieve ?now))
)

(defrule goal-reasoner-evaluate-clean-locks
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?p <- (plan (id ?plan-id) (goal-id ?goal-id))
  ?a <- (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
                     (action-name lock) (param-values ?name))
  (mutex (name ?name) (state LOCKED) (request NONE))
  =>
  (printout warn "Removing lock " ?name " of failed plan " ?plan-id
                 " of goal " ?goal-id crlf)
  (mutex-unlock-async ?name)
)

(defrule goal-reasoner-evaluate-clean-location-locks
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?p <- (plan (id ?plan-id) (goal-id ?goal-id))
  ?a <- (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
                     (action-name location-lock) (param-values ?loc ?side))
  (mutex (name ?name&:(eq ?name (sym-cat ?loc - ?side)))
         (state LOCKED) (request NONE))
  =>
  ; TODO only unlock if we are at a safe distance
  (printout warn "Removing location lock " ?name " without moving away!" crlf)
  (mutex-unlock-async ?name)
)

(defrule goal-reasoner-evaluate-common
  (declare (salience ?*SALIENCE-GOAL-EVALUTATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (parent nil) (mode FINISHED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
  =>
 ; (printout t "Goal '" ?goal-id "' has been " ?outcome ", evaluating" crlf)
  (if (eq ?outcome FAILED)
    then
    (bind ?num-tries (+ ?num-tries 1))
    (modify ?gm (num-tries ?num-tries))
  )

  (modify ?g (mode EVALUATED))
)

; # Goal Clean up
(defrule goal-reasoner-cleanup-common
  ?g <- (goal (id ?goal-id) (parent nil) (type ?goal-type)
          (mode EVALUATED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries) (max-tries ?max-tries))
  =>
 ; (printout t "Goal '" ?goal-id "' has been Evaluated, cleaning up" crlf)

  ;Flush plans of this goal
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (retract ?a)
    )
    (retract ?p)
  )

  (delayed-do-for-all-facts ((?sg goal)) (eq ?sg:parent ?goal-id)
    (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?sg:id)
     (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (retract ?a)
     )
     (retract ?p)
    )
  ;  (printout t "Goal '" ?sg:id "' (part of '" ?sg:parent
  ;     "') has, cleaning up" crlf)
    (retract ?sg)
  )

  (if (or (eq ?goal-type MAINTAIN)
          (and (eq ?outcome FAILED) (<= ?num-tries ?max-tries)))
    then
   ;   (printout t "Triggering re-expansion" crlf)
      (modify ?g (mode SELECTED) (outcome UNKNOWN))
    else
      (retract ?g ?gm)
    )
)
