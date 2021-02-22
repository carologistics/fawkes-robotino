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
  ?*SALIENCE-HIGHEST* = 1000 ; used for cleanup of failed goals
  ?*SALIENCE-GOAL-FORMULATE* = 500
  ?*SALIENCE-GOAL-REJECT* = 400
  ?*SALIENCE-GOAL-EXPAND* = 300
  ?*SALIENCE-GOAL-SELECT* = 200
  ?*SALIENCE-GOAL-PRE-EVALUATE* = 1
  ?*SALIENCE-GOAL-EVALUATE-GENERIC* = -1

  ?*MAX-RETRIES-PICK* = 2
  ?*MAX-RETRIES-PUT-SLIDE* = 2
  ?*GOAL-MAX-TRIES* = 3
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

(deffunction goal-needs-fresh-robot (?goal-class)
  (return (or (eq ?goal-class GET-BASE)
              (eq ?goal-class FILL-CS)
              (eq ?goal-class BUFFER-CS)
              (eq ?goal-class FILL-BASE-IN-RS)
              (eq ?goal-class PICKUP-WP)
              (eq ?goal-class CLEAR-OUTPUT)))
)

(deffunction goal-needs-robot-holding-wp (?goal-class)
  (return (or (eq ?goal-class MOUNT-CAP)
              (eq ?goal-class DELIVER)
              (eq ?goal-class MOUNT-RING)
              (eq ?goal-class FILL-RS)
              (eq ?goal-class DROP-WP)))
)

(deffunction production-root-goal (?goal-class)
  (return (or (eq ?goal-class PRODUCE-C0)
              (eq ?goal-class PRODUCE-CX)))
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

; ============================= Resource handling ===============================
; Since we don't need to consider multiple agents requesting the same resource
; and only use goals which require at most one resource (a mps) we use
; a simpler resource-distribution system. Whenever a goal requires a resource
; and it is free (not acquired by any other goal), the resource is assigned to that goal.
; After the goal is retracted, the resource is removed from the acquired-resources list
; and thus freed up again.

(defrule goal-reasoner-assign-resource
" Assign resources to goals that require exactly one resource"
  ?g <- (goal (id ?goal-id) (required-resources $?resources&:(eq (length$ $?resources) 1)) (mode COMMITTED)
              (meta $? global-priority ?gprio $?))
  (not (goal (acquired-resources $?not-available&:(member$ (nth$ 1 $?resources) $?not-available))))
  
  ; check that there isn't a goal with higher priority waiting
  (not (goal (mode COMMITTED) (meta $? global-priority ?ogprio&:(> ?ogprio ?gprio) $?)
            (required-resources $?oresources&:(member$ (nth$ 1 $?resources) ?oresources))))
  =>
  (modify ?g (acquired-resources $?resources))
  (printout t "Assigning resources " $?resources " to " ?goal-id crlf)
)

(defrule goal-reasoner-remove-resource
" Remove resource from a retracted goal
"
  (declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (acquired-resources $?acquired&:(> (length$ $?acquired) 0)) (mode RETRACTED))
  =>
  (modify ?g (acquired-resources))
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

(defrule goal-reasoner-create-retry-counter
" Make sure that every goal has a retry counter in the meta slot
"
  ; TODO: Only for production goals?
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (meta $?meta&:(not (member$ retries $?meta))) (mode FORMULATED))
  =>
  (modify ?g (meta $?meta retries 0))
)

(defrule goal-reasoner-inherit-global-priority
" Inherit global priority from parent
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?parent) (meta $? global-priority ?pprio $?))
  ?g <- (goal (meta $?meta&:(not (member$ global-priority $?meta))) (mode FORMULATED)
              (parent ?parent))
  =>
  (modify ?g (meta $?meta global-priority ?pprio))
)

(defrule goal-reasoner-create-global-priority
" Create global priority if parent doesn't have one
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (parent nil) (meta $?meta&:(not (member$ global-priority $?meta))) (mode FORMULATED))
  =>
  (modify ?g (meta $?meta global-priority 0))
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

;(defrule goal-reasoner-reset-machine-of-failed-goal
;" If a failed goal reserved a machine, make sure that the machine is
;  usable before releasing the resource. Resetting the mps should be done as a last
;  resort to avoid losing the machine for the rest of the game.
;"
;  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
;  (goal (id ?goal-id) (mode FINISHED) (outcome FAILED) (acquired-resources ?mps))
;  (domain-object (type mps) (name ?mps))
;  (wm-fact (key domain fact mps-state args? m ?mps s ~IDLE~BROKEN))
;  (not (wm-fact (key evaluated reset-mps args? m ?mps)))
;  =>
;  (printout warn "Resetting " ?mps " because " ?goal-id " failed" crlf)
;  (assert (wm-fact (key evaluated reset-mps args? m ?mps)))
;)

(defrule reset-retry-counter
" If a subgoal of a production finally succeeded, delete the failed subgoals
  and reset retry counter
"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?p <- (goal (id ?parent) (class ?class&:(production-root-goal ?class))
              (meta $?m1 retries ?retries $?m2))
  ?g <- (goal (parent ?parent) (class ?subgoal-class) (mode RETRACTED) (outcome FAILED))
  (goal (parent ?parent) (class ?subgoal-class) (mode FINISHED) (outcome COMPLETED))
  =>
  (retract ?g)
  (modify ?p (meta $?m1 retries (- ?retries 1) $?m2))
  (printout t "Retry of " ?parent " reset to " (- ?retries 1) crlf)  
)


(defrule goal-reasoner-drop-wp-of-failed-goal
" If a production root goal fails and some robot is still holding the wp
  it needs to drop the wp to avoid deadlocks. Discarding the wp should
  be done as a last resort to avoid losing a robot for the rest of the game.
"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (goal (id ?goal-id) (mode FINISHED) (outcome FAILED)
        (class ?class&:(production-root-goal ?class))
        (params $? wp ?wp $?)
        (meta $? retries ?retries&:(>= ?retries ?*GOAL-MAX-TRIES*) $?))
  (wm-fact (key domain fact holding args? r ?r wp ?wp))
  (not (wm-fact (key evaluated drop-wp args? r ?r wp ?wp)))
  =>
  (printout warn "Robot " ?r " should drop " ?wp " because " ?goal-id " failed " crlf)
  (assert (wm-fact (key evaluated drop-wp args? r ?r wp ?wp)))
)

; remove locks of failed goals
(defrule goal-reasoner-pre-evaluate-clean-locks
" Unlock all remaining locks of a failed goal."
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
                     (action-name lock) (param-values ?name)
                     ; only remove lock if the lock belongs to the plan
                     (state EXECUTION-SUCCEEDED|FINAL))
  ?df <- (domain-fact (name locked) (param-values ?mps))
=>
  (printout warn "Removing lock " ?mps " of failed plan " ?plan-id
                 " of goal " ?goal-id crlf)
  (retract ?df)
)


(defrule goal-reasoner-pre-evaluate-clean-location-locks
" Unlock all remaining location-locks of a failed goal."
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
                     (action-name location-lock) (param-values ?mps ?side)
                     ; only remove lock if the lock belongs to the plan
                     (state EXECUTION-SUCCEEDED|FINAL))
  (wm-fact (key refbox game-time) (values ?game-time $?))
  (domain-fact (name location-locked) (param-values ?mps ?side))
  (not (location-unlock-pending ?mps ?side ?))
=>
  (printout warn "Removing location lock " ?mps " on side " ?side crlf)
  (assert (location-unlock-pending ?mps ?side ?game-time))
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

(defrule goal-reasoner-remove-robot-from-finished
" Remove a robot from the parameter list of a production goal to free it up
  for another task
"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  ?g <- (goal (id ?goal-id) (mode RETRACTED) (params robot ?robot $?params))
  =>
  (printout t "Removing " ?robot " from " ?goal-id crlf)
  (modify ?g (params $?params))
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


; copied
(defrule goal-reasoner-evaluate-process-ds
  " Enhance the order-delivered fact of the order of a successful deliver goal,
    delete the mps-handling fact if the preparation took place.
  "
  ?g <- (goal (id ?goal-id) (class HANDLE-MPS) (mode FINISHED) (outcome ?outcome)
              (params ?mps))
  (wm-fact (key domain fact mps-type args? m ?mps t DS))
  (wm-fact (key refbox team-color) (value ?team-color))
  ; get order from plan action
  (plan-action (goal-id ?goal-id) (action-name prepare-ds) (param-values ?mps ?order))
  ?od <- (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color) (value ?val))  
  =>
  (if (eq ?outcome COMPLETED)
    then
      (printout t "Order " ?order " increased quantity by 1" crlf)
      (modify ?od (value (+ ?val 1)))
  )
  (modify ?g (mode EVALUATED))
)

; copied
(defrule goal-reasoner-evaluate-mps-reset-completed
  " Remove reset-mps flag after a successful reset-mps goal
  "
  ?g <- (goal (id ?id) (class RESET-MPS) (mode FINISHED)
                      (outcome COMPLETED) (params m ?mps))
  ?t <- (wm-fact (key evaluated reset-mps args? m ?mps))
  =>
  (retract ?t)
  (modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-evaluate-drop-wp-completed
  " Remove drop-wp flag after a successful drop-wp goal
  "
  ?g <- (goal (id ?id) (class DROP-WP) (mode FINISHED)
                      (outcome COMPLETED) (params r ?r wp ?wp))
  ?t <- (wm-fact (key evaluated drop-wp args? r ?r wp ?wp))
  =>
  (retract ?t)
  (modify ?g (mode EVALUATED))
)

; ================================= Goal Clean up ============================


(defrule goal-reasoner-reject-subgoals-of-failed-goal
  "Reject a subgoal if its parent has failed or is rejected"
  ?g <- (goal (id ?goal-id) (mode FORMULATED|SELECTED|EXPANDED) (parent ?parent))
  (goal (id ?parent) (mode EVALUATED) 
        (outcome ?parent-outcome&:(or (eq ?parent-outcome FAILED) (eq ?parent-outcome REJECTED))))
  =>
  (printout t ?goal-id " rejected because parent " ?parent " was " ?parent-outcome crlf)
  (modify ?g (mode RETRACTED) (outcome REJECTED))
)

(defrule goal-reasoner-retract-achieve
" Retract a goal if all sub-goals are retracted. Clean up any plans and plan
  actions attached to it.
"
  ; if goal has a parent, the parent will retract it
  ?g <-(goal (id ?goal-id) (type ACHIEVE) (mode EVALUATED) (parent nil))
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
  ?g <- (goal (id ?goal-id) (parent ?parent)
        (mode RETRACTED) (acquired-resources))
  (not (goal (parent ?goal-id)))
  ; keep subgoals until parent is finished, so that failed goals can affect the parent as well
  (not (goal (id ?parent) (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
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
