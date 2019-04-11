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


; ========================== Goal-Tree-Functions ============================


(deffunction requires-subgoal (?goal-type)
  (return (or (eq ?goal-type TRY-ONE-OF-SUBGOALS)
              (eq ?goal-type TIMEOUT-SUBGOAL)
              (eq ?goal-type RUN-ONE-OF-SUBGOALS)
              (eq ?goal-type RETRY-SUBGOAL)
              (eq ?goal-type RUN-ENDLESS)))
)


(deffunction goal-tree-assert-run-endless (?class ?frequency $?fact-addresses)
        (bind ?id (sym-cat MAINTAIN- ?class - (gensym*)))
        (bind ?goal (assert (goal (id ?id) (class ?class) (type MAINTAIN)
                            (sub-type RUN-ENDLESS) (params frequency ?frequency)
                            (meta last-formulated (now)))))
        (foreach ?f ?fact-addresses
                (goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
        (return ?goal)
)


(deffunction goal-tree-assert-subtree (?id $?fact-addresses)
        (foreach ?f ?fact-addresses
                (goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
)


; ============================= Goal Selection ===============================


(defrule goal-reasoner-select-root
"  Select all root goals (having no parent) in order to expand them."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE|MAINTAIN) (sub-type ~nil) (id ?goal-id) (mode FORMULATED))
  (not (goal (parent ?goal-id)))
=>
  (modify ?g (mode SELECTED))
)



(defrule goal-reasoner-expand-goal-with-sub-type
" Expand a goal with sub-type, if it has a child."
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?p <- (goal (id ?parent-id) (type ACHIEVE|MAINTAIN)
              (sub-type ?sub-type&:(requires-subgoal ?sub-type)) (mode SELECTED))
  ?g <- (goal (parent ?parent-id) (mode FORMULATED))
=>
  (modify ?p (mode EXPANDED))
)


; ========================= Goal Dispatching =================================
; Trigger execution of a plan. We may commit to multiple plans
; (for different goals), e.g., one per robot, or for multiple
; orders. It is then up to action selection and execution to determine
; what to do when.


; ========================= Goal Evaluation ==================================
; A finished goal has to be evaluated.
; In this step all necessary actions before removing the goal are executed,
; such as unlocking resources or adapting the world model and strategy based on
; goal outcomes or plan and action status.


(defrule goal-reasoner-evaluate-common
" Finally set a finished goal to evaluated.
  All pre evaluation steps should have been executed, enforced by the higher priority
"
  (declare (salience ?*SALIENCE-GOAL-EVALUTATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome))
=>
  ;(printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
  ;  "') has been completed, Evaluating" crlf)
  (modify ?g (mode EVALUATED))
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


(defrule goal-reasoner-evaluate-location-unlock-done
" React to a successful unlock of an location by removing the corresponding location-locked domain-fact"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?p <- (goal-reasoner-unlock-pending ?lock)
  ?m <- (mutex (name ?lock) (request UNLOCK) (response UNLOCKED))
  ?df <- (domain-fact (name location-locked) (param-values ?mps ?side))
  (test (not (eq FALSE (str-index (str-cat ?mps) (str-cat ?lock)))))
  (test (not (eq FALSE (str-index (str-cat ?side) (str-cat ?lock)))))
  =>
  (modify ?m (request NONE) (response NONE))
  (retract ?df)
  (retract ?p)
)


(defrule goal-reasoner-evaluate-lock-unlock-done
" React to a successful unlock of a lock by removing the corresponding locked domain-fact"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?p <- (goal-reasoner-unlock-pending ?lock)
  ?m <- (mutex (name ?lock) (request UNLOCK) (response UNLOCKED))
  ?df <- (domain-fact (name locked) (param-values ?lock))
  =>
  (modify ?m (request NONE) (response NONE))
  (retract ?df)
  (retract ?p)
)


(defrule goal-reasoner-finish-sub-goals-of-finished-parent
" Evaluate any sub-goal of a parent such that they can be cleaned up.
  This allows to recursively evaluate the sub-tree of an evaluated goal
  which is necessary to ensure a proper clean up of any related plans.
"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome))
  (goal (id ?sub-goal) (parent ?goal-id) (mode ~FINISHED&~EVALUATED&~RETRACTED))
=>
  (delayed-do-for-all-facts ((?sg goal))
    (and (eq ?sg:parent ?goal-id) (neq ?sg:mode FINISHED)
                                  (neq ?sg:mode EVALUATED)
                                  (neq ?sg:mode RETRACTED))
  ; (printout t "Goal '" ?sg:id "' (part of '" ?sg:parent
  ;     "') has, cleaning up" crlf)
    (modify ?sg (mode FINISHED))
  )
)


; ================================= Goal Clean up ============================


(defrule goal-reasoner-retract-achieve
" Retract a goal if all sub-goals are retracted. Clean up any plans and plan
  actions attached to it.
"
  ?g <-(goal (id ?goal-id) (type ACHIEVE) (mode EVALUATED)
             (acquired-resources))
  (not (goal (parent ?goal-id) (mode ?mode&~RETRACTED)))
=>
  ;(printout t "Goal '" ?goal-id "' has been Evaluated, cleaning up" crlf)
  ;Flush plans of this goal
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
      (retract ?a)
    )
    (retract ?p)
  )
  (modify ?g (mode RETRACTED))
)

(defrule goal-reasoner-remove-retracted-goal
" Remove a retracted goal once all acquired resources are freed."
  (declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (mode RETRACTED) (acquired-resources))
=>
  (retract ?g)
)


