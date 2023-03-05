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
  ?*SALIENCE-GOAL-EXECUTABLE-CHECK* = 450
  ?*SALIENCE-GOAL-REJECT* = 400
  ?*SALIENCE-GOAL-EXPAND* = 300
  ?*SALIENCE-GOAL-PRE-SELECT* = 250
  ?*SALIENCE-GOAL-SELECT* = 200
  ?*SALIENCE-GOAL-EVALUATE-GENERIC* = -1
)


(deffunction log-debug ($?verbosity)
  (bind ?v (nth$ 1 ?verbosity))
  (switch ?v
    (case NOISY then (return t))
    (case DEFAULT then (return nil))
    (case QUIET then (return nil))
  )
  (return nil)
)

(deffunction log-info ($?verbosity)
  (bind ?v (nth$ 1 ?verbosity))
  (switch ?v
    (case NOISY then (return warn))
    (case DEFAULT then (return t))
    (case QUIET then (return nil))
  )
  (return t)
)

(deffunction set-robot-to-waiting (?robot)
" Sets a robot that was assigned in a goal meta to waiting.
  If no robot was assigned nothing happens.

  @param ?robot: robot1 robot2 robot3 central nil
"
  (if (neq ?robot nil) then
    (do-for-fact ((?r wm-fact))
      (and (wm-key-prefix ?r:key (create$ central agent robot))
           (eq ?robot (wm-key-arg ?r:key r)))
      (assert (wm-fact (key central agent robot-waiting
                        args? r (wm-key-arg ?r:key r))))
    )
  )
)

(deffunction remove-robot-assignment-from-goal-meta (?goal)
  (if (not (do-for-fact ((?f goal-meta))
      (eq ?f:goal-id (fact-slot-value ?goal id))
      (modify ?f (assigned-to nil))
      ))
   then
    (printout t "Cannot find a goal meta fact for the goal " ?goal crlf)
  )
)

;This must fire for enter field
(defrule goal-meta-reset-assigned
  ?m <- (goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
  (goal (id ?goal-id) (outcome COMPLETED))
  =>
  (modify ?m (assigned-to nil))
)

(defrule goal-reasoner-select-root
  "There is an exectuable simple goal assigned to central, propagate selection."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&~GOAL-ORDER-C0&~GOAL-ORDER-C1&~GOAL-ORDER-C2&~GOAL-ORDER-C3)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id))
  (goal (id ?id) (sub-type SIMPLE) (mode FORMULATED) (is-executable TRUE))
  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

; ============================== Goal Expander ===============================

; (defrule goal-reasoner-expand-goal-with-sub-type
; " Expand a goal with sub-type, if it has a child."
;  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
;  ?p <- (goal (id ?parent-id) (type ACHIEVE|MAINTAIN)
;              (sub-type ?sub-type&:(requires-subgoal ?sub-type)) (mode SELECTED)
;              (verbosity ?v))
;  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode FORMULATED))
;  =>
;  (printout (log-debug ?v) "Goal " ?goal-id " EXPANDED" crlf)
;  (modify ?p (mode EXPANDED))
; )


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
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome)
              (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))
=>
  (set-robot-to-waiting ?robot)
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED" crlf)
  (modify ?g (mode EVALUATED))
)

; ----------------------- EVALUATE SPECIFIC GOALS ---------------------------


; ================================= Goal Clean up ============================

(defrule goal-reasoner-retract-achieve
" Retract a goal if all sub-goals are retracted. Clean up any plans and plan
  actions attached to it.
"
  ?g <-(goal (id ?goal-id) (type ACHIEVE) (mode EVALUATED)
             (acquired-resources) (verbosity ?v))
  (not (goal (parent ?goal-id) (mode ?mode&~RETRACTED)))
=>
  (printout (log-debug ?v) "Goal " ?goal-id " RETRACTED" crlf)
  (modify ?g (mode RETRACTED))
)


(defrule goal-reasoner-remove-retracted-goal-common
" Remove a retracted goal if it has no child (anymore).
  Goal trees are retracted recursively from bottom to top. This has to be done
  with low priority to avoid races with the sub-type goal lifecycle.
"
  (declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (verbosity ?v)
        (mode RETRACTED) (acquired-resources) (parent ?parent))
  (not (goal (parent ?goal-id)))
  (goal (id ?parent) (type MAINTAIN))
=>
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
      (retract ?a)
    )
    (retract ?p)
  )
  (delayed-do-for-all-facts ((?f goal-meta)) (eq ?f:goal-id ?goal-id)
    (retract ?f)
  )
  (retract ?g)
  (printout (log-debug ?v) "Goal " ?goal-id " removed" crlf)
)
