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

(deffunction set-robot-to-waiting (?meta)
" Sets a robot that was assigned in a goal meta to waiting.
  If no robot was assigned in the meta nothing happens.

  @param ?meta: goal meta
"
	(bind ?is-assigned (member$ assigned-to ?meta))
	(if ?is-assigned then
		(do-for-fact ((?r wm-fact))
			(and (wm-key-prefix ?r:key (create$ central agent robot))
			     (eq (nth$ (+ 1 ?is-assigned) ?meta) (wm-key-arg ?r:key r)))
			(assert (wm-fact (key central agent robot-waiting
			                  args? r (wm-key-arg ?r:key r))))
		)
	)
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

(deffunction goal-tree-assert-central-run-one (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-ONE- ?class - (gensym*)))
	(bind ?goal 
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-ONE-OF-SUBGOALS)))
  )
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-central-run-all (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-ALL- ?class - (gensym*)))
	(bind ?goal 
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)))
  )
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
)

(deffunction goal-tree-assert-central-run-parallel (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-PARALLEL- ?class - (gensym*)))
	(bind ?goal 
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)))
  )
	(foreach ?f ?fact-addresses
		(goal-tree-update-child ?f ?id (+ 1 (- (length$ ?fact-addresses) ?f-index))))
	(return ?goal)
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

(defrule goal-reasoner-select-root-robot-goal
"  Select all root goals assigned to the central."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type SIMPLE) (id ?goal-id)
              (mode FORMULATED) (meta $? assigned-to ?r&~central $?)
              (is-executable TRUE) (verbosity ?v))
  (not (goal (parent ?goal-id)))
  (not (goal (meta $? assigned-to ?r $?) (mode ~FORMULATED)))
=>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-root-central
"  Select all root goals assigned to the central."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type SIMPLE) (id ?goal-id)
              (mode FORMULATED) (meta $? assigned-to central $?)
              (is-executable TRUE) (verbosity ?v))
  (not (goal (parent ?goal-id)))
=>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

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


(defrule goal-reasoner-select-root-waiting-robot
  "Select all executable root goals in order to propagate selection."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE|MAINTAIN) (sub-type ~nil) 
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE))
  (not (goal (parent ?goal-id)))
  (domain-fact (name robot-waiting) (param-values ?robot))
  (not (and (wm-fact (key central agent robot-waiting
                      args? r ?o-robot&:(> (str-compare ?robot ?o-robot) 0)))
            (not (goal (meta $? assigned-to ?o-robot $?)))))
=>
  (printout error " i select a root " ?goal-id crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-root-central-executable-simple-goal
  "There is an exectuable simple goal assigned to central, propagate selection."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE|MAINTAIN) (sub-type ~nil) 
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE))
  (goal (sub-type SIMPLE) (mode FORMULATED) (is-executable TRUE) 
        (meta $? assigned-to central $?))
  =>
  (printout error " i select a root " ?goal-id crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-propagate-executability
  "There is an executable goal for a waiting robot or central, propagate."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  (or 
    ?g <- (goal (sub-type SIMPLE) (mode FORMULATED) (is-exectuable TRUE)
                (meta $? assigned-to central $?) (parent ?pid))
    (and 
      (domain-fact (name robot-waiting) (param-values ?robot))
      ?g <- (goal (sub-type SIMPLE) (mode FORMULATED) (is-exectuable TRUE)
                  (meta $?meta&:(not (member$ assigned-to ?meta))))
    )
  )
  =>
  ;fill missing parts
  (bind ?propagate TRUE)
  (bind ?parent-id ?pid)
  (while (eq ?propagate TRUE)
    (do-for-fact-all-facts ((?parent goal)) (and (eq ?parent:id ?pid))
      (modify ?parent (is-executable TRUE))
      (if (eq ?parent:parent nil)
        then (bind ?propagate FALSE)
      )
    )
  )
)

(defrule goal-reasoner-reset-selection-executability
  "If we selected a leaf node, reset executability flag and goal selection"
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (mode SELECTED) (sub-type ~SIMPLE))
  (goal (mode SELECTED) (sub-type SIMPLE))
  =>
  (modify ?g (mode FORMULATED) (is-executable FALSE));better do for fact
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
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome) (meta $?meta)
	            (verbosity ?v))
=>
	(set-robot-to-waiting ?meta)
	;(printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
	;  "') has been completed, Evaluating" crlf)
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
  (printout (log-debug) "Goal " ?goal-id " RETRACTED" crlf)
  (modify ?g (mode RETRACTED))
)


(defrule goal-reasoner-remove-retracted-goal-common
" Remove a retracted goal if it has no child (anymore).
  Goal trees are retracted recursively from bottom to top. This has to be done
  with low priority to avoid races with the sub-type goal lifecycle.
"
  (declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
  ?g <- (goal (id ?goal-id) (verbosity ?v)
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
