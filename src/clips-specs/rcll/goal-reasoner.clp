;---------------------------------------------------------------------------
;  goal-reasoner.clp - Goal reasoning for RCLL domain
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;             2020  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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
  ; Number of retrying enter-field
  ; until succeeding it manually
  ?*ENTER-FIELD-RETRIES* = 1
  ?*MAX-RETRIES-PICK* = 2
  ?*MAX-RETRIES-PUT-SLIDE* = 2
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
  ?*SALIENCE-GOAL-EVALUATE-GENERIC* = -1
)


; ========================== Goal-Tree-Functions ============================


(deffunction requires-subgoal (?goal-type)
  (return (or (eq ?goal-type TRY-ONE-OF-SUBGOALS)
              (eq ?goal-type TIMEOUT-SUBGOAL)
              (eq ?goal-type RUN-ONE-OF-SUBGOALS)
              (eq ?goal-type RUN-ALL-OF-SUBGOALS)
              (eq ?goal-type RETRY-SUBGOAL)
              (eq ?goal-type RUN-ENDLESS)))
)


(deffunction production-leaf-goal (?goal-class)
  (return (or (eq ?goal-class GET-BASE-TO-FILL-RS)
              (eq ?goal-class GET-SHELF-TO-FILL-RS)
              (eq ?goal-class FILL-RS)
              (eq ?goal-class FEED-CAP)
              (eq ?goal-class CLEAR-MPS)
              (eq ?goal-class DISCARD-UNKNOWN)
              (eq ?goal-class FEED-BASE-FIRST-RING)
              (eq ?goal-class FEED-BASE-NEXT-RING)
              (eq ?goal-class FEED-BASE-PRODUCE-C0)
              (eq ?goal-class FEED-PRODUCT-PRODUCE-CX)
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
              (eq ?goal-class FILL-CAP)
              (eq ?goal-class MOUNT-FIRST-RING)
              (eq ?goal-class MOUNT-NEXT-RING)
              (eq ?goal-class PRODUCE-C0)
              (eq ?goal-class PRODUCE-CX)
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
  (modify ?g (mode SELECTED))
)


(defrule goal-reasoner-expand-production-tree
"  Populate the tree structure of the production tree. The priority of subgoals
   is determined by the order they are asserted. Sub-goals that are asserted
   earlier get a higher priority.
"
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  (goal (id ?goal-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (parent ?goal-id)))
  (wm-fact (key config rcll enable-bot) (value ?enabled))
=>
  (if ?enabled
    then
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
    else
      (goal-tree-assert-subtree ?goal-id
        (goal-tree-assert-run-one PRODUCTION-SELECTOR
            (goal-tree-assert-run-one NO-PROGRESS)))
  )
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


(defrule goal-reasoner-pre-evaluate-clean-locks
" Unlock all remaining locks of a failed goal."
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (wm-fact (key cx identity) (value ?identity))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?p <- (plan (id ?plan-id) (goal-id ?goal-id))
  ?a <- (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
                     (action-name lock) (param-values ?name))
  (mutex (name ?name) (state LOCKED) (request ~UNLOCK) (locked-by ?identity)
         (pending-requests $?pending&:(not (member$ UNLOCK ?pending))))
=>
  (printout warn "Removing lock " ?name " of failed plan " ?plan-id
                 " of goal " ?goal-id crlf)
  (assert (goal-reasoner-unlock-pending ?name))
  (mutex-unlock-async ?name)
)


(defrule goal-reasoner-pre-evaluate-clean-location-locks
" Unlock all remaining location-locks of a failed goal."
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (wm-fact (key cx identity) (value ?identity))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?p <- (plan (id ?plan-id) (goal-id ?goal-id))
  ?a <- (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id ?plan-id)
                     (action-name location-lock) (param-values ?loc ?side))
  (mutex (name ?name&:(eq ?name (sym-cat ?loc - ?side)))
         (state LOCKED) (request ~UNLOCK) (locked-by ?identity)
         (pending-requests $?pending&:(not (member$ UNLOCK ?pending))))
=>
  ; TODO only unlock if we are at a safe distance
  (printout warn "Removing location lock " ?name " without moving away!" crlf)
  (assert (goal-reasoner-unlock-pending ?name))
  (mutex-unlock-async ?name)
)


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


(defrule goal-reasoner-pre-evaluate-failed-exog-actions
  " If an exogenous action failed, this means that something went totally wrong.
    However, it is unlikely that we are able to continue using the mps. Therefore
    we want to reset it
  "
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (plan-action (id ?id) (goal-id ?goal-id)
               (plan-id ?plan-id)
               (action-name bs-dispense|cs-retrieve-cap|cs-mount-cap|rs-mount-ring1|rs-mount-ring2|rs-mount-ring3)
               (param-values $? ?mps $?)
               (state FAILED))
  (domain-object (name ?mps) (type mps))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  (not (wm-fact (key evaluated reset-mps args? m ?mps)))
  =>
  (assert
    (wm-fact (key evaluated reset-mps args? m ?mps))
  )
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


(defrule goal-reasoner-evaluate-mps-reset-completed
  " Remove reset-mps flag after a successful reset-mps goal
  "
  ?g <- (goal (id ?id) (class RESET-MPS) (mode FINISHED)
                      (outcome COMPLETED) (params r ?self m ?mps))
  ?t <- (wm-fact (key evaluated reset-mps args? m ?mps))
  =>
  (retract ?t)
  (modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-evaluate-production-maintain
  "Clean up all rs-fill-priorities facts when the production maintenance goal
   fails."
  ?g <- (goal (id ?goal-id) (class PRODUCTION-MAINTAIN) (parent nil)
              (mode FINISHED) (outcome ?outcome))
  =>
  (printout t "Goal '" ?goal-id "' has been " ?outcome ", evaluating" crlf)
  (do-for-all-facts ((?prio wm-fact)) (wm-key-prefix ?prio:key (create$ evaluated fact rs-fill-priority))
   (retract ?prio))
  (modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-evaluate-completed-deliver
" Enhance the order-delivered fact of the order of a successful deliver goal
"
  ?g <- (goal (id ?goal-id) (class DELIVER) (mode FINISHED) (outcome COMPLETED)
              (params $? ?order $?))
  (wm-fact (id "/refbox/team-color") (value ?team-color&:(neq ?team-color nil)))
  ?od <- (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color) (value ?val))
  =>
  (modify ?od (value (+ ?val 1)))
  (modify ?g (mode EVALUATED))
)


(defrule goal-reasoner-evaluate-completed-produce-c0-and-mount-first-ring
" Bind a workpiece to the order it belongs to.

  Workpieces that got dispensed during PRODUCE-C0 and MOUNT-FIRST-RING get
  tied to their order independent of the goal outcome as long as they are
  still usable.
"
  ?g <- (goal (id ?goal-id) (class FEED-BASE-PRODUCE-C0|FEED-BASE-FIRST-RING)
              (parent ?parent-id)
              (mode FINISHED) (outcome ?outcome)
              (params $?params))
 (plan (goal-id ?goal-id) (id ?plan-id))
 (time $?now)
 (wm-fact (key domain fact wp-usable args? wp ?wp&:(eq ?wp (get-param-by-arg ?params wp))))
 (wm-fact (key domain fact self args? r ?robot))
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


(defrule goal-reasoner-evaluate-completed-subgoal-refill-shelf
  ?g <- (goal (class REFILL-SHELF) (mode FINISHED))
  =>
   (modify ?g (mode EVALUATED))
)


(defrule goal-reasoner-evaluate-failed-wp-put
" After a failed wp-put, check if the gripper interface indicates, that the workpiece is still in the gripper.
  If this is not the case, the workpiece is lost and the corresponding facts are marked for clean-up
"
  (declare (salience ?*SALIENCE-GOAL-PRE-EVALUATE*))
  (plan-action (id ?id) (goal-id ?goal-id)
	(plan-id ?plan-id) (action-name ?an&:(or (eq ?an wp-put) (eq ?an wp-put-slide-cc)))
	   (param-values ?r ?wp ?mps $?)
	   (state FAILED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?hold <- (wm-fact (key domain fact holding args? r ?r wp ?wp))
  (RobotinoSensorInterface (digital_in ?d1 ?d2 $?))
  =>
  (if (not (and (eq ?d1 FALSE) (eq ?d2 TRUE)))
  then
      (assert (wm-fact (key monitoring safety-discard)))
  )
  (printout t "Goal " ?goal-id " failed because of " ?an crlf)
)

(defrule goal-reasouner-evaluate-failed-exog-actions
  " If an exogenous action failed, this means that something went totally wrong.
    However, it is unlikely that we are able to continue using the mps. Therefore
    we want to reset it
  "
  (plan-action (id ?id) (goal-id ?goal-id)
               (plan-id ?plan-id)
               (action-name bs-dispense|cs-retrieve-cap|cs-mount-cap|rs-mount-ring1|rs-mount-ring2|rs-mount-ring3)
               (param-values $? ?mps $?)
               (state FAILED))
  (domain-object (name ?mps) (type mps))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  (not (wm-fact (key evaluated reset-mps args? m ?mps)))
  =>
  (assert
    (wm-fact (key evaluated reset-mps args? m ?mps))
  )
  (modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-evaluate-get-shelf-failed
" After a failed wp-get-shelf, assume that the workpiece is not there
  and mark the corresponding facts for cleanup.
  By this, the next time a different spot will be tried
"
  (plan-action (id ?id) (goal-id ?goal-id)
	   (plan-id ?plan-id)
	   (action-name wp-get-shelf)
	   (param-values ?r ?wp ?mps ?spot)
	   (state FAILED))
  (plan (id ?plan-id) (goal-id ?goal-id))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED))
  ?wp-s<- (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot))
  =>
  (printout t "Goal " ?goal-id " has been failed because of wp-get-shelf and is evaluated" crlf)
  (assert (wm-fact (key monitoring cleanup-wp args? wp ?wp)))
  (assert (wm-fact (key domain fact spot-free args? m ?mps spot ?spot)))
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
" Retract all formulated sub-goal of the production tree once a production leaf
  goal is dispatched.
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?goal) (parent ?parent) (type ACHIEVE)
        (sub-type ?sub-type) (class ?class&:(production-goal ?class))
        (mode FORMULATED))
  (goal (id ?some-leaf) (class ?some-class&:(production-goal ?some-class))
        (sub-type SIMPLE) (mode DISPATCHED))
=>
  (delayed-do-for-all-facts ((?g goal))
    (and (eq ?g:mode FORMULATED) (production-goal ?g:class))
    (modify ?g (mode RETRACTED) (outcome REJECTED))
  )
)


(defrule goal-reasoner-reject-production-goals-that-block-produce-cx
" Retract a formulated sub-goal of the production tree if it blocks a
  goal to produce high complexity products.
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (domain-obj-is-of-type ?mps mps)
  (goal (id ?goal) (parent ?parent) (type ACHIEVE)
        (sub-type SIMPLE) (class PRODUCE-CX
                                |MOUNT-NEXT-RING
                                |MOUNT-FIRST-RING)
        (required-resources $? ?mps $?) (mode FORMULATED))
  ?g <- (goal (id ?o-goal&~?goal) (class ~PRODUCE-CX
                                        &~MOUNT-NEXT-RING
                                        &~MOUNT-FIRST-RING)
              (sub-type SIMPLE) (parent ?o-parent) (mode FORMULATED)
              (required-resources $? ?mps $?))
  (goal (id ?o-parent) (class ~URGENT))
=>
  (printout t "Goal " ?o-goal " is rejected because it blocks " ?goal crlf)
  (modify ?g (mode RETRACTED) (outcome REJECTED))
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

(defrule goal-reasoner-reject-subgoal-if-parent-rejected
" Reject a goal if its parent is rejected. 
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?parent) (mode RETRACTED) (outcome REJECTED))
  ?g <- (goal (parent ?parent) (id ?id) (mode FORMULATED|SELECTED))
  =>
  (modify ?g (mode RETRACTED) (outcome REJECTED))
)

(defrule goal-reasoner-cut-goal-from-goaltree-if-only-async-children-left
  " Remove a production from the goal-tree if all the children of type SIMPLE are executed
    and there are still async children left.  
  "
  ?p <- (goal (id ?parent) (mode DISPATCHED) (sub-type RUN-ALL-OF-SUBGOALS) (parent ?grandparent&~nil))
  ?gp <- (goal (id ?grandparent) (mode DISPATCHED))
  (not (goal (parent ?parent) (sub-type SIMPLE) (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
  (not (goal (parent ?parent) (sub-type SIMPLE) (outcome FAILED)))
  (goal (parent ?parent) (sub-type SIMPLE-ASYNC) (mode ~DISPATCHED&~FINISHED&~EVALUATED&~RETRACTED))
  =>
  (modify ?p (parent nil))
  (modify ?gp (mode FINISHED) (outcome COMPLETED))
)

(defrule goal-reasoner-retract-run-all-of-subgoals
  " Retract an evaluated RUN-ALL goal, so that the acquired resources can be released.
  "
  ?g <- (goal (id ?parent) (mode EVALUATED) (sub-type RUN-ALL-OF-SUBGOALS))
  =>
  (modify ?g (mode RETRACTED))
)
