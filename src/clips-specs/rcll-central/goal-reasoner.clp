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
  ?*SALIENCE-GOAL-PRE-EVALUATE* = 1
  ?*SALIENCE-GOAL-EVALUATE-GENERIC* = -1
)

; ========================== Goal-Tree-Functions ============================

(deffunction requires-subgoal (?goal-type)
  (return (or (eq ?goal-type TRY-ONE-OF-SUBGOALS)
              (eq ?goal-type TIMEOUT-SUBGOAL)
              (eq ?goal-type RUN-ONE-OF-SUBGOALS)
              (eq ?goal-type RETRY-SUBGOAL)
              (eq ?goal-type RUN-ENDLESS)
              (eq ?goal-type RUN-ALL-OF-SUBGOALS)))
)

(deffunction production-leaf-goal (?goal-class)
  (return (or (eq ?goal-class GET-BASE-TO-FILL-RS)
              (eq ?goal-class GET-SHELF-TO-FILL-RS)
              (eq ?goal-class FILL-RS)
              (eq ?goal-class FILL-CAP)
              (eq ?goal-class CLEAR-MPS)
              (eq ?goal-class DISCARD-UNKNOWN)
              (eq ?goal-class PRODUCE-C0)
              (eq ?goal-class PRODUCE-CX)
              (eq ?goal-class MOUNT-FIRST-RING)
              (eq ?goal-class MOUNT-NEXT-RING)
              (eq ?goal-class DELIVER)
              (eq ?goal-class RESET-MPS) 
              (eq ?goal-class WAIT)
              (eq ?goal-class GO-WAIT)
              (eq ?goal-class WAIT-FOR-MPS-PROCESS)))
)

(deffunction production-tree-goal (?goal-class)
  (return (or (eq ?goal-class PRODUCTION-SELECTOR)
              (eq ?goal-class URGENT)
              (eq ?goal-class FULFILL-ORDERS)
              (eq ?goal-class DELIVER-PRODUCTS)
              (eq ?goal-class INTERMEDEATE-STEPS)
              (eq ?goal-class CLEAR)
              (eq ?goal-class WAIT-FOR-PROCESS)
              (eq ?goal-class PREPARE-RESOURCES)
              (eq ?goal-class PREPARE-CAPS)
              (eq ?goal-class PREPARE-RINGS)
              (eq ?goal-class NO-PROGRESS)))
)

(deffunction production-goal (?goal-class)
  (return (or (production-tree-goal ?goal-class)
              (production-leaf-goal ?goal-class)))
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

(deffunction goal-reasoner-assign-production-tree-to-robot (?id ?robot)
  "Recursively add the robot as parameter to all sub-goals in the tree 
  rooted at the goal with the given ID.
  "
  (do-for-fact ((?g goal)) (eq ?g:id ?id)
    (delayed-do-for-all-facts ((?sub-goal goal)) (eq ?sub-goal:parent ?g:id)
      (goal-reasoner-assign-production-tree-to-robot ?sub-goal:id ?robot)
      (modify ?sub-goal (params $?sub-goal:params robot ?robot))
    )
  )
)

(defrule goal-reasoner-expand-production-tree
"  Populate the tree structure of the production tree. The priority of subgoals
   is determined by the order they are asserted. Sub-goals that are asserted
   earlier get a higher priority.
"
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (goal (id ?goal-id) (class PRODUCTION-MAINTAIN) 
        (params $?frequency robot ?robot $?params) (mode SELECTED))
  (not (goal (parent ?goal-id)))
=>
  (goal-tree-assert-subtree ?goal-id
    (goal-tree-assert-run-one PRODUCTION-SELECTOR
      (goal-tree-assert-run-one URGENT)
      (goal-tree-assert-run-one FULFILL-ORDERS
        (goal-tree-assert-run-one DELIVER-PRODUCTS)
        (goal-tree-assert-run-one INTERMEDEATE-STEPS))
      (goal-tree-assert-run-one PREPARE-RESOURCES
        (goal-tree-assert-run-one CLEAR)
        (goal-tree-assert-run-one PREPARE-CAPS)
        (goal-tree-assert-run-one WAIT-FOR-PROCESS)
        (goal-tree-assert-run-one PREPARE-RINGS))
      (goal-tree-assert-run-one NO-PROGRESS)))
  (goal-reasoner-assign-production-tree-to-robot ?goal-id ?robot)
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

; ------------------------- PRE EVALUATION -----------------------------------

(defrule goal-reasoner-pre-evaluate-location-unlock-done
" React to a successful unlock of an location by removing the corresponding location-locked domain-fact"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?p <- (goal-reasoner-unlock-pending ?lock)
  ?m <- (mutex (name ?lock) (request UNLOCK) (state OPEN))
  ?df <- (domain-fact (name location-locked) (param-values ?mps ?side))
  (test (not (eq FALSE (str-index (str-cat ?mps) (str-cat ?lock)))))
  (test (not (eq FALSE (str-index (str-cat ?side) (str-cat ?lock)))))
  =>
  (modify ?m (request NONE) (response NONE))
  (retract ?df)
  (retract ?p)
)


(defrule goal-reasoner-pre-evaluate-lock-unlock-done
" React to a successful unlock of a lock by removing the corresponding locked domain-fact"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?p <- (goal-reasoner-unlock-pending ?lock)
  ?m <- (mutex (name ?lock) (request UNLOCK) (state OPEN))
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

(defrule goal-reasoner-evaluate-visit-success
  "Evaluate VISIT-STATION goal. Assert fact that station was
  visited in case of success."
  ?g <- (goal (class VISIT-STATION) (mode FINISHED) (outcome ?outcome&COMPLETED)
    (params r ?robot station ?station side ?side))
  =>
  (assert (visited ?station))
  (printout t "Evaluating goal: Station " ?station " visited successfully" crlf)
  (modify ?g (mode EVALUATED))
)


(defrule goal-reasoner-evaluate-completed-produce-c0-and-mount-first-ring
" Bind a workpiece to the order it belongs to.

  Workpieces that got dispensed during PRODUCE-C0 and MOUNT-FIRST-RING get
  tied to their order independent of the goal outcome as long as they are
  still usable.
"
  ?g <- (goal (id ?goal-id) (class PRODUCE-C0|MOUNT-FIRST-RING)
              (parent ?parent-id)
              (mode FINISHED) (outcome ?outcome)
              (params $?params))
 (plan (goal-id ?goal-id) (id ?plan-id))
 (time $?now)
 (wm-fact (key domain fact wp-usable args? wp ?wp&:(eq ?wp (get-param-by-arg ?params wp))))
 (wm-fact (key order meta points-max
           args? ord ?order&:(eq ?order (get-param-by-arg ?params order)))
          (value ?max))
 =>
 (printout t "Goal '" ?goal-id "' has been completed, Evaluating" crlf)
 (assert (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order) (type BOOL) (value TRUE)))
 (printout t "Started producing order " ?order " which potentially yields "
             ?max " points" crlf)
 (modify ?g (mode EVALUATED))
)


(defrule goal-reasoner-evaluate-process-mps
  ?g <- (goal (class PROCESS-MPS) (id ?goal-id) (mode FINISHED) (outcome ?outcome) (params m ?mps))
  (plan-action (goal-id ?goal-id) (action-name ?prepare-action) (state FINAL))
  ?pre <- (wm-fact (key mps-handling prepare ?prepare-action ?mps args? $?prepare-params))
  ?pro <- (wm-fact (key mps-handling process ?process-action ?mps args? $?process-params))
  =>
  (retract ?pre ?pro)
  (modify ?g (mode EVALUATED))
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


(defrule goal-reasoner-reject-production-tree-goal-missing-subgoal
" Retract a formulated sub-goal of the production tree if it requires a
  sub-goal but there is none formulated.
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  ?g <- (goal (id ?goal) (parent ?parent) (type ACHIEVE)
              (sub-type ?sub-type&:(requires-subgoal ?sub-type))
              (class ?class&:(production-goal ?class)) (mode FORMULATED))
  (not (goal (parent ?goal) (mode FORMULATED)))
=>
  (modify ?g (mode RETRACTED) (outcome REJECTED))
)


(defrule goal-reasoner-reject-production-tree-goals-other-goal-dispatched
" Retract all formulated leaf-goals of a production tree once a production leaf
  goal of the tree is dispatched.
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?goal) (parent ?parent) (type ACHIEVE)
        (sub-type ?sub-type) (class ?class&:(production-goal ?class))
        (mode FORMULATED))
  (goal (id ?dispatched-leaf) (class ?some-class&:(production-goal ?some-class))
        (sub-type SIMPLE) (mode DISPATCHED) (parent ?parent-of-dispatched))
  (goal (id ?parent) (params $?p1 robot ?robot $?p2))
  (goal (id ?parent-of-dispatched) (params $?q1 robot ?robot $?q2))
=>
  (delayed-do-for-all-facts ((?g goal))
    (and (eq ?g:mode FORMULATED) (production-goal ?g:class) (eq ?g:parent ?parent))
    (modify ?g (mode RETRACTED) (outcome REJECTED))
  )
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
