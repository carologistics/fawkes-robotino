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
  ?*SALIENCE-GOAL-SELECT* = 200
  ?*SALIENCE-GOAL-EVALUATE-GENERIC* = -1
)

(deffunction requires-subgoal (?goal-type)
  (return (or (eq ?goal-type TRY-ONE-OF-SUBGOALS)
              (eq ?goal-type TIMEOUT-SUBGOAL)
              (eq ?goal-type RUN-ONE-OF-SUBGOALS)
              (eq ?goal-type RETRY-SUBGOAL)
              (eq ?goal-type RUN-ENDLESS)))
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

(deffunction is-parent-of (?parent ?child)
  (bind ?propagate TRUE)
  (bind ?goal-id ?child)
  (while ?propagate
    (do-for-all-facts ((?goal goal)) (eq ?goal:id ?goal-id)
      (if (or (eq ?goal:parent nil) (eq ?goal:parent ?parent)) then
        (bind ?propagate FALSE)
      )
      (bind ?goal-id ?goal:parent)
    )
  )

  (return (eq ?goal-id ?parent))
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

(deffunction goal-tree-update-meta-run-all-order (?f ?ordering)
  (do-for-fact ((?goal-meta goal-meta)) (eq ?goal-meta:goal-id (fact-slot-value ?f id))
    (modify ?goal-meta (run-all-ordering ?ordering))
  )
)

(deffunction goal-reasoner-compute-order-conflicts-payments (?order1 ?order2)
  (bind ?conflict FALSE)
  (printout t crlf crlf ?order1 " " ?order2 crlf crlf)
  (do-for-all-facts ((?rs domain-fact)) (and (eq ?rs:name mps-type) (member$ RS ?rs:param-values))
    (bind ?rs-name (nth$ 1 ?rs:param-values))
    (if (> (+  (calculate-order-payments-sum ?order1 ?rs-name) (calculate-order-payments-sum ?order2 ?rs-name)) 3) then
      (bind ?conflict TRUE)
    )
  )
  (return ?conflict)
)

(deffunction goal-tree-assert-run-endless (?class ?frequency $?fact-addresses)
        (bind ?id (sym-cat MAINTAIN- ?class - (gensym*)))
        (bind ?goal (assert (goal (id ?id) (class ?class) (type MAINTAIN)
                            (sub-type RUN-ENDLESS) (params frequency ?frequency)
                            (meta last-formulated (now)) (meta-template goal-meta))))
        (foreach ?f ?fact-addresses
                (goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
        (return ?goal)
)


(deffunction goal-tree-assert-subtree (?id $?fact-addresses)
        (foreach ?f ?fact-addresses
                (goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
)

(deffunction goal-tree-assert-central-run-one (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-ONE- ?class - (gensym*)))
	(bind ?goal
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS)))
  )
  (assert (goal-meta (goal-id ?id)))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-central-run-all (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-ALL- ?class - (gensym*)))
	(bind ?goal
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)))
  )
  (assert (goal-meta (goal-id ?id)))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-central-run-all-sequence (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-ALL- ?class - (gensym*)))
	(bind ?goal
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
                  (meta sequence-mode)))
  )
  (assert (goal-meta (goal-id ?id)))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-central-run-all-prio (?class ?prio $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-ALL- ?class - (gensym*)))
	(bind ?goal
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)))
  )
  (assert (goal-meta (goal-id ?id)))
	(foreach ?f ?fact-addresses
    ;(goal-tree-update-meta-run-all-order ?f (+ 1 (- (length$ ?fact-addresses) ?f-index)))
		(goal-tree-update-child ?f ?id ?prio)
  )
	(return ?goal)
)

(deffunction goal-tree-assert-central-run-parallel (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-PARALLEL- ?class - (gensym*)))
	(bind ?goal
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)))
  )
  (assert (goal-meta (goal-id ?id)))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-central-run-parallel-flat (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-PARALLEL- ?class - (gensym*)))
	(bind ?goal
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)))
  )
  (assert (goal-meta (goal-id ?id)))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id 1)
  )
	(return ?goal)
)

(deffunction goal-tree-assert-central-run-parallel-prio (?class ?prio $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-PARALLEL- ?class - (gensym*)))
	(bind ?goal
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)))
  )
  (assert (goal-meta (goal-id ?id)))
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id ?prio)
  )
	(return ?goal)
)
; =========================== Goal Executability =============================

(defrule goal-reasoner-propagate-executability
  "There is an executable goal for a waiting robot or central, propagate until
  we hit the root or a goal that is not FORMULATED."
  (declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
  (or
    (and ?g <- (goal (id ?id) (sub-type SIMPLE) (mode FORMULATED) (is-executable TRUE)
                (parent ?pid))
         (goal-meta (goal-id ?id) (assigned-to central)))
    (and
      (wm-fact (key central agent robot-waiting args? r ?robot))
      ?g <- (goal (id ?goal-id) (sub-type SIMPLE) (mode FORMULATED) (is-executable TRUE)
                  (parent ?pid))
      (goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
    )
  )
  (test (neq ?pid nil))
  =>
  (bind ?propagate TRUE)
  (bind ?parent-id ?pid)
  (while (eq ?propagate TRUE)
    (do-for-all-facts ((?parent goal)) (eq ?parent:id ?parent-id)
      (if (eq ?parent:mode FORMULATED)
        then
        (modify ?parent (is-executable TRUE))
        (bind ?parent-id ?parent:parent)
      )

      (if (or (eq ?parent:parent nil) (neq ?parent:mode FORMULATED))
        then (bind ?propagate FALSE)
      )
    )
  )
)

; ============================= Goal Selection ===============================


(defrule goal-reasoner-select-root-maintain
"  Select all root maintain goals (having no parent) in order to expand them."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type MAINTAIN) (sub-type ~nil) (id ?goal-id)
        (mode FORMULATED) (verbosity ?v))
  (not (goal (parent ?goal-id)))
=>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-root
  "There is an exectuable simple goal assigned to central, propagate selection."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (root-for-order nil))

  (or
    ;either there is an executable sub-goal assigned to central
    ;this should use some reference to the root id such that only the right root
    ;is selected
    (and
      (goal (id ?id) (sub-type SIMPLE) (mode FORMULATED) (is-executable TRUE))
      (goal-meta (goal-id ?id) (assigned-to central))
    )
    ;or there is a robot that is waiting and not assigned to a subgoal that is waiting
    (and
      (wm-fact (key central agent robot-waiting args? r ?robot))

      (not (and (goal-meta (goal-id ?g-id) (assigned-to ?robot))
                (goal (id ?g-id) (mode ~FORMULATED))))

      (not (and (wm-fact (key central agent robot-waiting
                          args? r ?o-robot&:(> (str-compare ?robot ?o-robot) 0)))
          (goal-meta (assigned-to ?o-robot))))
    )
  )
  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-from-dispatched-children
  "Select the candidate child goal of highest priority amongst the run-parallel and
  run-all goals. Depending on the parent type enforce two different policies to find a candidate:
  - if the parent is a RUN-ALL goal, then there must be no other goal under the same parent
    that has a smaller ordering and has not been start yet.
  - if the parent is a RUN-PARALLEL goal, there the candidate must have the highest priority."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (id ?candidate-id) (parent ?candidate-parent) (is-executable TRUE) (mode FORMULATED) (priority ?candidate-priority))
  (goal-meta (goal-id ?candidate-id) (run-all-ordering ?candidate-ordering))
  (goal (id ?candidate-parent) (mode DISPATCHED) (sub-type ?candidate-parent-type&CENTRAL-RUN-ALL-OF-SUBGOALS|CENTRAL-RUN-SUBGOALS-IN-PARALLEL))

  ;it is the correct goal to choose within the subtree
  (or
    ;parent is a run-parallel goal (there is no formulated goal with a higher priority)
    (and
      (test (eq ?candidate-parent-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL))
      (not (goal (id ~?candidate-id) (parent ?candidate-parent) (is-executable TRUE) (mode FORMULATED) (priority ?other-priority&:(> ?other-priority ?candidate-priority))))
    )
    ;parent is a run-all goal (there is no formulated goal with a smaller ordering number)
    (and
      (test (eq ?candidate-parent-type CENTRAL-RUN-ALL-OF-SUBGOALS))
      (not
        (and
          (goal (id ?other-id&~?candidate-id) (parent ?candidate-parent) (mode FORMULATED))
          (goal-meta (goal-id ?other-id) (run-all-ordering ?other-ordering&:(> ?candidate-ordering ?other-ordering)))
        )
      )
    )
  )

  ;it is the correct goal within the entire tree to choose from (there is no goal that fulfills the same requirements with a higher priority)
  (not
    (and
      (goal (id ?alternative-id&~?candidate-id) (parent ?alternative-parent) (is-executable TRUE) (mode FORMULATED) (priority ?alternative-priority&:(> ?alternative-priority ?candidate-priority)))
      (goal-meta (goal-id ?alternative-id) (run-all-ordering ?alternative-ordering))
      (goal (id ?alternative-parent) (mode DISPATCHED) (sub-type ?alternative-parent-type&CENTRAL-RUN-ALL-OF-SUBGOALS|CENTRAL-RUN-SUBGOALS-IN-PARALLEL))

      (or
        (and
          (test (eq ?alternative-parent-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL))
          (not (goal (id ~?alternative-id&~?candidate-id) (parent ?alternative-parent) (is-executable TRUE) (mode FORMULATED) (priority ?other-priority&:(> ?other-priority ?alternative-priority))))
        )
        (and
          (test (eq ?alternative-parent-type CENTRAL-RUN-ALL-OF-SUBGOALS))
          (not
            (and
              (goal (id ?other-id&~?alternative-id&~?candidate-id) (parent ?alternative-parent) (mode FORMULATED))
              (goal-meta (goal-id ?other-id) (run-all-ordering ?other-ordering&:(> ?alternative-ordering ?other-ordering)))
            )
          )
        )
      )
    )
  )

  ;there is no other acheive goal currently selected, expanded, or committed
  (not (goal (mode SELECTED|EXPANDED|COMMITTED) (type ACHIEVE)))
  =>
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-root-for-order
  "Select the root of an order-production-tree if it has the highest priority
  and is not interfering with currently selected goals."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  (goal (parent nil) (type ACHIEVE) (sub-type ~nil)
      (id ?any-goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?any-goal-id) (root-for-order ?any-order))

  ;at most two orders are active
  (not (and
    (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (id ?goal-id2) (mode SELECTED|EXPANDED|DISPATCHED))
    (goal-meta (goal-id ?goal-id2) (root-for-order ~nil))
    (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (id ?goal-id3&:(neq ?goal-id3 ?goal-id2)) (mode SELECTED|EXPANDED|DISPATCHED))
    (goal-meta (goal-id ?goal-id3) (root-for-order ~nil))
  ))
  =>
  ;find a suitable order for parallel fulfillment 
  (bind ?target-priority 0)
  (bind ?target-goal nil)
  (do-for-all-facts ((?goal-fact goal) (?goal-meta-fact goal-meta))
      (and (eq ?goal-fact:id ?goal-meta-fact:goal-id)
           (neq ?goal-meta-fact:root-for-order nil)
           (eq ?goal-fact:mode FORMULATED)
      )
  
    ;check active order trees for conflicts
    (bind ?existing-order-conflict FALSE)
    (do-for-all-facts ((?existing-goal-fact goal) (?existing-goal-meta-fact goal-meta))
        (and (eq ?existing-goal-fact:id ?existing-goal-meta-fact:goal-id)
            (neq ?existing-goal-meta-fact:root-for-order nil)
            (neq ?existing-goal-fact:mode FORMULATED)
            (neq ?existing-goal-fact:mode FINISHED)
        )
        (if (goal-reasoner-compute-order-conflicts-payments ?existing-goal-meta-fact:root-for-order ?goal-meta-fact:root-for-order) then
          (bind ?existing-order-conflict TRUE)
        )
    )

    ;if the priority is higher than that of the current candidate and there is no 
    ;conflict save this goal as new candidate
    (if (and 
          (> ?goal-fact:priority ?target-priority)
          (not ?existing-order-conflict)
        )
      then
      (bind ?target-priority ?goal-fact:priority)
      (bind ?target-goal ?goal-fact)
    )
  )

  (if (neq ?target-goal nil) then
    (printout (log-debug ?v) "Goal " (fact-slot-value ?target-goal id) " SELECTED" crlf)
    (modify ?target-goal (mode SELECTED))
  )
)

(defrule goal-reasoner-balance-payment-goals
  "If there are multiple orders being fulfilled in parallel and one of them contains 
  a DISCARD-CC goal, try to replace it with a payment goal instead."
  (goal (parent nil) (type ACHIEVE) (sub-type ~nil)
      (id ?goal1) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED) (verbosity ?v1))
  (goal-meta (goal-id ?goal1) (root-for-order ?order1))
  (goal (parent nil) (type ACHIEVE) (sub-type ~nil)
      (id ?goal2) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED) (verbosity ?v2))
  (goal-meta (goal-id ?goal2) (root-for-order ?order2))
  (wm-fact (key domain fact order-complexity args? ord ?order1 comp C0))
  (wm-fact (key domain fact order-complexity args? ord ?order2 comp C2|C3))
  ?d <- (goal (id ?discard-goal) (class DISCARD) (mode FORMULATED) (params wp ?wp&~UNKNOWN wp-loc ?source-loc wp-side ?source-side))
  ?p1 <- (goal (id ?payment-goal) (class PAY-FOR-RINGS-WITH-BASE) (mode FORMULATED) (parent ?pay-base-parent) (params $? target-mps ?target-loc target-side ?target-side))
  ?p2 <- (goal (id ?payment-instruct) (class INSTRUCT-BS-DISPENSE-BASE) (mode FORMULATED) (parent ?pay-base-parent))
  ?p3 <- (goal (id ?pay-base-parent) (class PAY-FOR-RING-GOAL) (mode FORMULATED) (parent ?payment-parent))
  (test (is-parent-of ?goal1 ?discard-goal))
  (test (is-parent-of ?goal2 ?payment-goal))
  =>
  (retract ?d)
  (retract ?p1)
  (retract ?p2)
  (retract ?p3)
  (assert 
    (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
      (id (sym-cat PAY-FOR-RINGS-WITH-CAP-CARRIER- (gensym*))) (sub-type SIMPLE)
      (verbosity NOISY) (is-executable FALSE) (meta-template goal-meta) (parent ?payment-parent)
      (params  wp ?wp
                wp-loc ?source-loc
                wp-side ?source-side
                target-mps ?target-loc
                target-side ?target-side
      )
	  )
  )
)

; ============================== Goal Expander ===============================

(defrule goal-reasoner-expand-goal-with-sub-type
" Expand a goal with sub-type, if it has a child."
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?p <- (goal (id ?parent-id) (type ACHIEVE|MAINTAIN)
              (sub-type ?sub-type&:(requires-subgoal ?sub-type)) (mode SELECTED)
              (verbosity ?v))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode FORMULATED))
  =>
  (printout (log-debug ?v) "Goal " ?goal-id " EXPANDED" crlf)
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
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome)
	            (verbosity ?v))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot))
=>
	(set-robot-to-waiting ?robot)
	(printout (log-debug ?v) "Goal " ?goal-id " EVALUATED" crlf)
	(modify ?g (mode EVALUATED))
)

; ----------------------- EVALUATE SPECIFIC GOALS ---------------------------

(defrule goal-reasoner-evaluate-move-out-of-way
" Sets a finished move out of way goal independent of the outcome to formulated."
  ?g <- (goal (id ?goal-id) (class MOVE-OUT-OF-WAY) (mode FINISHED)
              (outcome ?outcome) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
=>
  (printout (log-debug ?v) "Evaluate move-out-of-way goal " ?goal-id crlf)
  (set-robot-to-waiting ?robot)
  (remove-robot-assignment-from-goal-meta ?g)

  ; delete plans of the goal
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action))
                              (and (eq ?a:plan-id ?p:id)
                                   (eq ?a:goal-id ?goal-id))
      (retract ?a))
    (retract ?p)
  )
  (modify ?g (mode FORMULATED) (outcome UNKNOWN) (is-executable FALSE))
  (printout (log-debug ?v) "Goal " ?goal-id " FORMULATED" crlf)
)

(defrule goal-reasoner-evaluate-failed-goto
" Re-formulate a failed goal if the workpiece it processes is still usable
"
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED) (meta $?meta)
	            (verbosity ?v))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(plan-action (action-name ?action&move|go-wait|wait-for-wp|wait-for-free-side)
	             (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot))
	=>
	(set-robot-to-waiting ?robot)
	(remove-robot-assignment-from-goal-meta ?g)
	(printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, reformulate as only a " ?action " action failed" crlf)
	(modify ?g (mode FORMULATED) (outcome UNKNOWN))

	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action)) (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
			(retract ?a)
		)
		(retract ?p)
	)
)

(defrule goal-reasoner-evaluate-failed-workpiece-usable
" Re-formulate a failed goal if the workpiece it processes is still usable
"
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED) (meta $?meta)
	            (verbosity ?v))
	(plan (id ?plan-id) (goal-id ?goal-id))
	(plan-action (action-name ?action&wp-get|wp-put|wp-put-slide-cc|wp-get-shelf)
	             (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED)
	             (param-values $? ?wp $?))
	(or (wm-fact (key domain fact wp-usable args? wp ?wp))
	    (wm-fact (key domain fact wp-on-shelf args? wp ?wp $?))
	)
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot))
	=>
	(set-robot-to-waiting ?robot)
	(remove-robot-assignment-from-goal-meta ?g)
	(printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, reformulate as workpiece is still usable after failed " ?action crlf)
	(modify ?g (mode FORMULATED) (outcome UNKNOWN))

	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action)) (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
			(retract ?a)
		)
		(retract ?p)
	)
)

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

(defrule goal-reasoner-error-goal-without-sub-type-detected
" This goal reasoner only deals with goals that have a sub-type. Other goals
  are not supported.
"
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  (goal (id ?goal) (class ?class) (sub-type nil))
=>
  (printout error ?goal " of class " ?class " has no sub-type" crlf)
)

(defrule goal-reasoner-clean-goals-separated-from-parent
	?g <- (goal (parent ?pid&~nil))
	(not (goal (id ?pid)))
	=>
	(retract ?g)
)

(deffunction is-goal-running (?mode)
	(return (or (eq ?mode SELECTED) (eq ?mode EXPANDED)
	            (eq ?mode COMMITTED) (eq ?mode DISPATCHED)))
)
