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
; Goal reasoner for goals with sub-types. Goals without sub-types are NOT
; handled.
;
; Basic functionalities:
;  - Select MAINTAIN goals
;  - Finish sub-goals of finished parent goals to ensure proper cleanup
;  - Expand goals that are inner nodes of a goal tree
;    (the other goals are expanded in the goal-expander)
;  - Automatically evaluate all goals with low priority
;    Special evaluation for specific goals with default priority
;  - Clean up and remove executed goals
;    Reject formulated inner production tree goals if no suitable sub-goal
;    could be formulated
;  - Reject formulated production tree goals once some leaf goal is dispatched
;
;
; The intended goal life-cycle of the production tree (assuming no goal gets
; rejected due to resource locks) can be summarized to:
;  - Formulate inner tree nodes to expand the root
;  - Formulate all currently achievable production goals
;  - Reject all inner tree nodes that have no sub-goal
;  - Recursively dispatch inner goals until a leaf goal is dispatched
;  - Reject all tree nodes that are not dispatched
;  - Once a leaf goal is finished and evaluated, the outcome is recursively
;    handed back to the root
;  - After the root is evaluated all other tree goals (by the time in mode
;    RETRACTED) are deleted
;  - The root gets reformulated and selected
;
; If a leaf goal has to be rejected, the parent goal dispatches another
; leaf goal instead. If this is not possible then the parent is rejected and
; recursively another goal is tried until either one leaf can be dispatched or
; all goals are rejected (this should never happen, since we have WAIT goals),
; leading the root to be rejected and reformulated.

(defglobal
  ?*SALIENCE-GOAL-FORMULATE* = 500
  ?*SALIENCE-GOAL-REJECT* = 400
  ?*SALIENCE-GOAL-EXPAND* = 300
  ?*SALIENCE-GOAL-SELECT* = 200
  ?*SALIENCE-GOAL-EVALUATE-GENERIC* = -1
)

(deffunction requires-subgoal (?goal-type)
  (return (or (eq ?goal-type TRY-ONE-OF-SUBGOALS)
              (eq ?goal-type TIMEOUT-SUBGOAL)
              (eq ?goal-type RUN-ONE-OF-SUBGOALS)
              (eq ?goal-type RETRY-SUBGOAL)
              (eq ?goal-type RUN-ENDLESS)
              (eq ?goal-type RUN-ALL-OF-SUBGOALS)))
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
  (printout error " i select a root " ?goal-id crlf)
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


; ----------------------- EVALUATE COMMON ------------------------------------

(defrule goal-reasoner-evaluate-common
" Finally set a finished goal to evaluated.
  All pre evaluation steps should have been executed, enforced by the higher priority
"
  (declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome))
=>
  ;(printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
  ;  "') has been completed, Evaluating" crlf)
  (modify ?g (mode EVALUATED))
)

; ----------------------- EVALUATE SPECIFIC GOALS ---------------------------


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
  (modify ?g (mode RETRACTED))
)


(defrule goal-reasoner-remove-retracted-goal-common
" Remove a retracted goal if it has no child (anymore).
  Goal trees are retracted recursively from bottom to top. This has to be done
  with low priority to avoid races with the sub-type goal lifecycle.
"
  (declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
  ?g <- (goal (id ?goal-id)
        (mode RETRACTED) (acquired-resources))
  (not (goal (parent ?goal-id)))
=>
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
      (retract ?a)
    )
    (retract ?p)
  )
  (retract ?g)
)

(defrule goal-reasoner-error-goal-without-sub-type-detected
" This goal reasoner only deals with goals that have a sub-type. Other goals
  are not supported.
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?goal) (class ?class) (sub-type nil))
=>
  (printout error ?goal " of class " ?class " has no sub-type" crlf)
)
