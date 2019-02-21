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
  ; PRE-EVALUATE rules should do additional steps in EVALUATION but must not
  ; set the goal to EVALUATED
  ?*SALIENCE-GOAL-PRE-EVALUATE* = 1
  ; common evaluate rules should have
  ;   lower salience than case specific ones
  ?*SALIENCE-GOAL-EVALUTATE-GENERIC* = -1
)


; ============================================  Goal Selection ======================================


(defrule goal-reasoner-select-parent-goal
" We can choose one or more goals for expansion.
  Select all parent goals in order to expand them
"
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (id ?goal-id) (mode FORMULATED))
=>
  (modify ?g (mode SELECTED))
  (assert (goal-meta (goal-id ?goal-id) (max-tries ?*GOAL-MAX-TRIES*)))
)


(defrule goal-reasoner-expand-parent-goal
" Expand a parent goal if it has some formulated subgoals"
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?p <- (goal (id ?parent-id) (mode SELECTED))
  ?g <- (goal (parent ?parent-id) (mode FORMULATED))
=>
  (modify ?p (mode EXPANDED))
)


(defrule goal-reasoner-commit-parent-goal
" Commit an expanded parent goal, if it has a committed subgoal or no subgoal at all"
  ?g <- (goal (id ?goal-id) (mode EXPANDED) (parent nil))
  (not (goal (parent ?goal-id)))
  (plan (id ?plan-id) (goal-id ?goal-id))
=>
  (modify ?g (mode COMMITTED) (committed-to ?plan-id))
)


(defrule goal-reasoner-select-subgoal
" Select the subgoal with the highes priority, if the parent goal is expanded"
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?p <- (goal (id ?parent-id) (mode EXPANDED))
  ?g <- (goal (parent ?parent-id) (mode FORMULATED)
          (id ?subgoal-id) (class ?class) (priority ?priority))
  ;Select the formulated subgoal with the highest priority
  (not (goal (parent ?parent-id) (mode FORMULATED)
             (priority ?h-priority&:(> ?h-priority ?priority)))
  )
  ;No other subgoal being processed
  (not (goal (parent ?parent-id)
             (mode SELECTED|EXPANDED|
                   COMMITTED|DISPATCHED|
                   FINISHED|RETRACTED)))
  (not (goal (parent ?parent-id) (mode EVALUATED) (outcome ~REJECTED)))
=>
  (if (neq ?class BEACONACHIEVE) then
    (printout t "Goal " ?subgoal-id " selected!" crlf)
  )
  (modify ?g (mode SELECTED))
  (assert (goal-meta (goal-id ?subgoal-id)))
)


(defrule goal-reasoner-commit-subgoal
" Commit to a plan an expanded subgoal if the parent is expanded and the is no subgoal to this goal.
  Assumes, that there is only one plan per goal "
  ?g <- (goal (id ?goal-id) (class ?class) (mode EXPANDED) (parent ?parent-id))
  ?pg <- (goal (id ?parent-id) (mode EXPANDED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (or (not (goal (parent ?goal-id)))
      (goal (parent ?goal-id) (mode COMMITTED)))
=>
  (if (neq ?class BEACONACHIEVE) then
    (printout t "Goal " ?goal-id " committed!" crlf)
  )
  (modify ?g (mode COMMITTED) (committed-to ?plan-id))
  (modify ?pg (mode COMMITTED))
)


; ======================================== Goal Dispatching ============================
; Trigger execution of a plan. We may commit to multiple plans
; (for different goals), e.g., one per robot, or for multiple
; orders. It is then up to action selection and execution to determine
; what to do when.


(defrule goal-reasoner-dispatch-parent-goal
" Dispatch a parent goal if all resources where acquired and a subgoal is dispatched"
	?g <- (goal (mode COMMITTED)
          (id ?goal-id)
          (parent nil)
          (required-resources $?req)
          (acquired-resources $?acq&:(subsetp ?req ?acq)))
  (not (goal (parent ?goal-id)))
=>
	(modify ?g (mode DISPATCHED))
)


(defrule goal-reasoner-dispatch-subgoal
" Dispatch a subgoal if all resources where acquired and its parent goal is committed."
  ?g <- (goal (mode COMMITTED)
          (id ?goal-id)
          (class ?class)
          (parent ?parent-id)
          (required-resources $?req)
          (acquired-resources $?acq&:(subsetp ?req ?acq)))
  ?pg <- (goal (id ?parent-id) (mode COMMITTED))
  (or (not (goal (parent ?goal-id)))
      (goal (parent ?goal-id) (mode DISPATCHED)))
=>
  (if (neq ?class BEACONACHIEVE) then
    (printout t "Goal " ?goal-id " dispatched!" crlf)
  )
	(modify ?g (mode DISPATCHED))
	(modify ?pg (mode DISPATCHED))
)


(defrule goal-reasoner-reexpand-parent-goal
"The parent is committed or dispatched, but all subgoals are formulated and there is no active subgoal
 Reset the parent goal to the expanded state then.
"
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?p <- (goal (id ?parent-id) (mode COMMITTED|DISPATCHED))
  (goal (parent ?parent-id) (mode FORMULATED))
  (not (goal (parent ?parent-id) (mode ~FORMULATED) (outcome ~REJECTED)))
=>
  (modify ?p (mode EXPANDED))
)


(defrule goal-reasoner-finish-parent-goal
" If subgoal is evaluated, set the parent goal to finished."
  ?pg <- (goal (id ?pg-id) (mode DISPATCHED))
  ?sg <- (goal (id ?sg-id) (parent ?pg-id) (mode EVALUATED)
               (outcome ?outcome&~REJECTED))
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
" Fail a parent goal if all subgoals are rejected. "
  ?pg <- (goal (id ?pg-id) (mode ~FINISHED))
  (not (goal (parent ?pg-id) (outcome ~REJECTED)))
  (goal (parent ?pg-id))
=>
  ;(printout debug "Goal '" ?pg-id " failed because all subgoald been REJECTED" crlf)
  (modify ?pg (mode FINISHED) (outcome FAILED))
)


; ============================================ Goal Evaluation ==================================
; A finished goal has to be evaluated.
; In this step all necessary actions before removing the goal are executed, e.g unlock resources etc


(defrule goal-reasoner-evaluate-subgoal-common
" Finally set a finished goal to evaluated.
  All pre evaluation steps should have been executed, enforced by the higher priority
"
  (declare (salience ?*SALIENCE-GOAL-EVALUTATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id&~nil) (mode FINISHED) (outcome ?outcome))
  ?pg <- (goal (id ?parent-id))
  ?m <- (goal-meta (goal-id ?parent-id))
  (time $?now)
=>
  ;(printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
  ;  "') has been completed, Evaluating" crlf)
  (modify ?g (mode EVALUATED))
  (modify ?m (last-achieve ?now))
)


(defrule goal-reasoner-evaluate-clean-locks
" Unlock all remaining locks of a failed goal."
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (wm-fact (key cx identity) (value ?identity))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?p <- (plan (id ?plan-id) (goal-id ?goal-id))
  ?a <- (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
                     (action-name lock) (param-values ?name))
  (mutex (name ?name) (state LOCKED) (request NONE) (locked-by ?identity))
=>
  (printout warn "Removing lock " ?name " of failed plan " ?plan-id
                 " of goal " ?goal-id crlf)
  (assert (goal-reasoner-unlock-pending ?name))
  (mutex-unlock-async ?name)
)


(defrule goal-reasoner-evaluate-clean-location-locks
" Unlock all remaining location-locks of a failed goal."
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (wm-fact (key cx identity) (value ?identity))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?p <- (plan (id ?plan-id) (goal-id ?goal-id))
  ?a <- (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
                     (action-name location-lock) (param-values ?loc ?side))
  (mutex (name ?name&:(eq ?name (sym-cat ?loc - ?side)))
         (state LOCKED) (request NONE) (locked-by ?identity))
=>
  ; TODO only unlock if we are at a safe distance
  (printout warn "Removing location lock " ?name " without moving away!" crlf)
  (assert (goal-reasoner-unlock-pending ?name))
  (mutex-unlock-async ?name)
)


(defrule goal-reasoner-evaluate-unlock-done
" React to a successful unlock by removing the pending fact and resetting the mutex"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?p <- (goal-reasoner-unlock-pending ?lock)
  ?m <- (mutex (name ?lock) (request UNLOCK) (response UNLOCKED))
=>
  (modify ?m (request NONE) (response NONE))
  (retract ?p)
)

(defrule goal-reasoner-evaluate-common-parent-goal
" Evaluate a parent goal. If it was failed, increase the retry counter."
  (declare (salience ?*SALIENCE-GOAL-EVALUTATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (parent nil) (mode FINISHED) (outcome ?outcome))
  (not (goal-reasoner-unlock-pending ?))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
=>
  ;(printout t "Goal '" ?goal-id "' has been " ?outcome ", evaluating" crlf)
  (if (eq ?outcome FAILED)
    then
    (bind ?num-tries (+ ?num-tries 1))
    (modify ?gm (num-tries ?num-tries))
  )

  (modify ?g (mode EVALUATED))
)


; ================================= Goal Clean up =======================================


(defrule goal-reasoner-cleanup-rejected-goal
" If a goal is rejected, remove all belonging plans and plan-actions."
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome REJECTED))
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


(defrule goal-reasoner-cleanup-common-parent-goal
" Clean up all plans and plan-actions of an evaluated parent goal. 
  Retract all subgoals.
"
  ?g <- (goal (id ?goal-id) (parent nil) (type ?goal-type)
          (mode EVALUATED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries) (max-tries ?max-tries))
=>
  ;(printout t "Goal '" ?goal-id "' has been Evaluated, cleaning up" crlf)
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
  ; (printout t "Goal '" ?sg:id "' (part of '" ?sg:parent
  ;     "') has, cleaning up" crlf)
    (modify ?sg (mode RETRACTED))
  )
)


(defrule goal-reasoner-reselect-parent-goal
" Reselect a parent goal if it was failed and the maximum number of retries was not reached,
  or if it is a maintain goal.
"
  ?g <- (goal (id ?goal-id) (parent nil) (type ?goal-type)
          (mode EVALUATED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries) (max-tries ?max-tries))
  (not (goal (parent ?goal-id)))
=>
  (if (or (eq ?goal-type MAINTAIN)
          (and (eq ?outcome FAILED) (<= ?num-tries ?max-tries)))
    then
   ;   (printout t "Triggering re-expansion" crlf)
      (modify ?g (mode SELECTED) (outcome UNKNOWN))
    else
      (modify ?g (mode RETRACTED))
    )
)


(defrule goal-reasoner-remove-goalmeta-of-retracted-goals
" Remove retracted goals and the belonging goal-meta facts."
  ?g <- (goal (id ?goal-id) (mode RETRACTED) (acquired-resources))
=>
  (do-for-fact ((?gm goal-meta)) (eq ?gm:goal-id ?goal-id)
    (retract ?gm)
  )
  (retract ?g)
)
