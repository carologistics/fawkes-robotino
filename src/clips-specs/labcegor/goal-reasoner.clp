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

(deffunction requires-subgoal (?goal-type)
  (return (or (eq ?goal-type TRY-ONE-OF-SUBGOALS)
              (eq ?goal-type TIMEOUT-SUBGOAL)
              (eq ?goal-type RUN-ONE-OF-SUBGOALS)
              (eq ?goal-type RETRY-SUBGOAL)
              (eq ?goal-type RUN-ENDLESS)
              (eq ?goal-type CENTRAL-RUN-ALL-OF-SUBGOALS)
              (eq ?goal-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)))
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

(deffunction goal-reasoner-nuke-subtree (?goal)
  "Remove an entire subtree."
  (do-for-all-facts ((?child goal)) (eq ?child:parent (fact-slot-value ?goal id))
    (goal-reasoner-nuke-subtree ?child)
  )
  (retract ?goal)
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
  "Detects a conflict between two order payments, i.e. when the sum of required payments
   by both orders is bigger than the max number of payments per machine, which is 3."
  (bind ?conflict FALSE)
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
      (if (or (eq ?parent:parent nil) (neq ?parent:mode FORMULATED))
        then (bind ?propagate FALSE)
      )
      (if (eq ?parent:mode FORMULATED)
        then
        (bind ?parent-id ?parent:parent)
        (modify ?parent (is-executable TRUE))
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
  (goal (id ?id) (sub-type SIMPLE) (mode FORMULATED) (is-executable TRUE))
  (goal-meta (goal-id ?id) (assigned-to central))
  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-init-selection-criteria
  (domain-loaded)
  (not (wm-fact (key goal selection criterion args? t ?)))
  =>
  (assert
    (wm-fact (key goal selection criterion args? t root) (type SYMBOL) (is-list TRUE) (values (create$)))
    (wm-fact (key goal selection criterion args? t run-all) (type SYMBOL) (is-list TRUE) (values (create$)))
    (wm-fact (key goal selection criterion args? t run-parallel) (type SYMBOL) (is-list TRUE) (values (create$)))
  )
)
(defrule goal-reasoner-select-root-for-order
  "Select the root of an order-production-tree if it has the highest priority
  and is not interfering with currently selected goals."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?target-goal <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil)
      (id ?any-goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?any-goal-id) (root-for-order ?any-order&~nil))
  =>
  (printout (log-debug ?v) "Goal " (fact-slot-value ?target-goal id) " SELECTED" crlf)
  (modify ?target-goal (mode SELECTED))
)


(defrule goal-reasoner-add-selectable-root-goal
  (declare (salience ?*SALIENCE-GOAL-PRE-SELECT*))
  (goal (type ACHIEVE) (id ?goal-id) (parent nil) (mode FORMULATED) (is-executable TRUE))
  ?selection <- (wm-fact (key goal selection criterion args? t root) (values $?values&:(not (member$ ?goal-id ?values))))
  =>
  (modify ?selection (values (append$ ?values ?goal-id)))
)

(defrule goal-reasoner-add-selectable-run-all-goal
  (declare (salience ?*SALIENCE-GOAL-PRE-SELECT*))
  (goal (type ACHIEVE) (id ?parent-id) (mode DISPATCHED) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS))
  (goal (type ACHIEVE) (id ?goal-id) (parent ?parent-id) (mode FORMULATED) (is-executable TRUE))
  (goal-meta (goal-id ?goal-id) (run-all-ordering ?ordering))
  (not (and
    (goal (id ?other-id&~?goal-id) (parent ?parent-id) (mode FORMULATED))
    (goal-meta (goal-id ?other-id) (run-all-ordering ?other-ordering&:(> ?ordering ?other-ordering)))
  ))
  ?selection <- (wm-fact (key goal selection criterion args? t run-all) (values $?values&:(not (member$ ?goal-id ?values))))
  =>
  (modify ?selection (values (append$ ?values ?goal-id)))
)

(defrule goal-reasoner-add-selectable-run-parallel-goal
  (declare (salience ?*SALIENCE-GOAL-PRE-SELECT*))
  (goal (type ACHIEVE) (id ?parent-id) (mode DISPATCHED) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL))
  (goal (type ACHIEVE) (id ?goal-id) (parent ?parent-id) (mode FORMULATED)
        (is-executable TRUE) (priority ?priority))
  (not (goal (id ~?goal-id) (parent ?parent-id) (is-executable TRUE) (mode FORMULATED) (priority ?other-priority&:(> ?other-priority ?priority))))
  ?selection <- (wm-fact (key goal selection criterion args? t run-parallel) (values $?values&:(not (member$ ?goal-id ?values))))
  =>
  (modify ?selection (values (append$ ?values ?goal-id)))
)

(defrule goal-reasoner-remove-non-executable-from-list
  (declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
  (goal (id ?id) (is-executable FALSE))
  ?selection <- (wm-fact (key goal selection criterion args? t ?) (values $?values&:(member$ ?id ?values)))
  =>
  (bind ?index (member$ ?id ?values))
  (modify ?selection (values (delete$ ?values ?index ?index)))
)


(defrule goal-reasoner-apply-selection-across-types
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  (wm-fact (key goal selection criterion args? t ?) (values ?some-goal-id $?))
  ?some-goal <- (goal (id ?some-goal-id) (priority ?some-prio))
  (not (goal (mode SELECTED|EXPANDED|COMMITTED) (type ACHIEVE)))
  =>
  (bind ?all-choices (create$))
  (delayed-do-for-all-facts ((?selection wm-fact))
    (wm-key-prefix ?selection:key (create$ goal selection criterion))
    (bind ?all-choices (append$ ?all-choices ?selection:values))
    (modify ?selection (values (create$)))
  )
  (bind ?highest-prio ?some-prio)
  (bind ?highest-prio-goal-fact ?some-goal)
  (do-for-all-facts ((?g goal))
    (member$ ?g:id ?all-choices)
    (if (> ?g:priority ?highest-prio)
     then
      (bind ?highest-prio ?g:priority)
      (bind ?highest-prio-goal-fact ?g)
    )
  )
  (bind ?robot nil)
  ; get assigned robot
  (do-for-fact ((?highest-prio-gm goal-meta))
               (eq ?highest-prio-gm:goal-id (fact-slot-value ?highest-prio-goal-fact id))
                 (bind ?robot (fact-slot-value ?highest-prio-gm assigned-to))
  )
  (modify ?highest-prio-goal-fact (mode SELECTED))
  ; flush executability
	(delayed-do-for-all-facts ((?g goal))
		(and (eq ?g:is-executable TRUE) (neq ?g:class SEND-BEACON))
		(modify ?g (is-executable FALSE))
	)
  ; if it is actually a robot, remove all other assignments and the waiting status
	(if (and (neq ?robot central) (neq ?robot nil))
		then
		(delayed-do-for-all-facts ((?g goal))
			(and (eq ?g:mode FORMULATED) (not (eq ?g:type MAINTAIN))
			     (any-factp ((?gm goal-meta))
			                (and (eq ?gm:goal-id ?g:id)
			                     (eq ?gm:assigned-to ?robot))))
			(remove-robot-assignment-from-goal-meta ?g)
		)
		(do-for-fact ((?waiting wm-fact))
			(and (wm-key-prefix ?waiting:key (create$ central agent robot-waiting))
			     (eq (wm-key-arg ?waiting:key r) ?robot))
			(retract ?waiting)
		)
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
  (wm-fact (key domain fact order-complexity args? ord ?order1 com C0))
  (wm-fact (key domain fact order-complexity args? ord ?order2 com C2|C3))
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


(defrule goal-reasoner-evaluate-mount-or-payment
" Sets a finished mount or payment goal to evaluated"
  ?g <- (goal (id ?goal-id) (class MOUNT-CAP|PAY-FOR-RINGS-WITH-BASE|PAY-FOR-RINGS-WITH-CAP-CARRIER|PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF) (mode FINISHED) (outcome COMPLETED)
              (verbosity ?v) (params $? ?mn $? ?rc))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot)(order-id ?order-id))
=>
  (set-robot-to-waiting ?robot)
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED" ?mn  crlf)
  (modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-evaluate-mount-goal-mps-workload
" Evaluate a MOUNT-RING goal and reduce mps workload counter based on completed plan-actions"
 ?g <- (goal (id ?goal-id) (class MOUNT-RING) (mode FINISHED) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot) (order-id ?order-id))
  (plan-action (action-name rs-mount-ring1|rs-mount-ring2|rs-mount-ring3) (goal-id ?goal-id)
               (param-values ?rs ?wp $?) (state FINAL))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order-id))
  ?wmf-order <- (wm-fact (key mps workload order args? m ?rs ord ?order-id))
  ?update-fact <- (wm-fact (key mps workload needs-update) (value ?value))
  =>
  (modify ?wmf-order (value (- (fact-slot-value ?wmf-order value) 1)))
  (if (eq ?value FALSE) then
    (modify ?update-fact (value TRUE))
  )
  (set-robot-to-waiting ?robot)
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED" crlf)
  (modify ?g (mode EVALUATED))
)

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

(defrule goal-reasoner-evaluate-failed-discard
" Re-formulate a failed discard goal"
	?g <- (goal (id ?goal-id) (class DISCARD) (mode FINISHED) (outcome FAILED)
	            (verbosity ?v))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot))
	=>
	(set-robot-to-waiting ?robot)
	(remove-robot-assignment-from-goal-meta ?g)
	(printout (log-debug ?v) "Goal " ?goal-id " EVALUATED and reformulated as only a discard failed" crlf)
	(modify ?g (mode FORMULATED) (outcome UNKNOWN))
	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action))
		   (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
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
(defrule goal-reasoner-evaluate-failed-preparation-goal
  "A carrier was lost for a preparation goal, reformulate the goal."
  ?g <- (goal (id ?goal-id) (class BUFFER-CAP|DISCARD) (mode FINISHED) (outcome FAILED))
  ?p <- (plan (goal-id ?goal-id) (id ?plan-id))
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (action-name wp-get-shelf|wp-get|wp-put) (state FAILED) (param-values ? ? ?mps $?))
  =>
  ;remove the plan
  (delayed-do-for-all-facts ((?pa plan-action)) (and (eq ?pa:goal-id ?goal-id) (eq ?pa:plan-id ?plan-id))
    (retract ?pa)
  )
  (retract ?p)

  ;reassert the goal
  (modify ?g (mode FORMULATED) (outcome UNKNOWN))
)

(defrule goal-reasoner-evaluate-failed-payment-goal
  "A ring payment was lost, reformulate the goal"
  ?g <- (goal (id ?goal-id) (class ?class&PAY-FOR-RINGS-WITH-CAP-CARRIER|PAY-FOR-RINGS-WITH-BASE) (mode FINISHED) (outcome FAILED))
  ?p <- (plan (goal-id ?goal-id) (id ?plan-id))
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (action-name wp-get-shelf|wp-get|wp-put-slide-cc) (state FAILED) (param-values ? ? ?mps $?))
  =>
  ;remove the plan
  (delayed-do-for-all-facts ((?pa plan-action)) (and (eq ?pa:goal-id ?goal-id) (eq ?pa:plan-id ?plan-id))
    (retract ?pa)
  )
  (retract ?p)

  ;reassert the goal
  (modify ?g (mode FORMULATED) (outcome UNKNOWN))
  (if (eq ?class PAY-FOR-RINGS-WITH-CAP-CARRIER) then
    (modify ?g (class PAY-FOR-RINGS-WITH-BASE))
  )
)

(defrule goal-reasoner-evaluate-failed-production-goal
  "The goal failed and was not reformulated because the workpiece was lost. Escalate the error."
  ?g <- (goal (id ?goal-id) (class ?goal-class&DELIVER|MOUNT-RING|MOUNT-CAP) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?goal-id) (order-id ?order-id))
  (plan (goal-id ?goal-id) (id ?plan-id))
  (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (action-name wp-get|wp-put) (state FAILED) (param-values ? ? ?mps $?))
  =>
  ;fail the entire tree
  (delayed-do-for-all-facts ((?tree-goal goal) (?tree-goal-meta goal-meta))
                            (and (eq ?tree-goal:id ?tree-goal-meta:goal-id)
                                 (eq ?tree-goal-meta:order-id ?order-id)
                                 (neq ?tree-goal:mode RETRACTED))
      (modify ?tree-goal (mode FINISHED) (outcome FAILED))

      (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?tree-goal:id)
        (delayed-do-for-all-facts ((?pa plan-action)) (and (eq ?pa:goal-id ?tree-goal:id)  (eq ?pa:plan-id ?plan-id))
          (retract ?pa)
        )
        (retract ?p)
    )
  )
)

(defrule goal-reasoner-evaluate-failed-order-tree
  "If the root of an order tree fails, take care of open and fulfilled payments/buffered caps and
  clean up the tree."
  ?root <- (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id))

  ;wait until no requests for the order are
  (not (wm-fact (key request $? args? ord ?order-id $?) (value ~ACTIVE)))

  ?instruct-root <- (goal (class INSTRUCT-ORDER) (id ?instruct-root-id))
  (goal (parent ?instruct-root-id) (id ?instruct-root-child))
  (goal-meta (order-id ?order-id) (goal-id ?instruct-root-child))

  ;goals to handle
  (goal (id ?buffer-goal) (class BUFFER-CAP) (mode ?buffer-goal-mode) (outcome ?buffer-goal-outcome) (params target-mps ?cs $?))
  (goal-meta (goal-id ?buffer-goal) (order-id ?order-id))
  (goal (id ?discard-goal) (class  DISCARD) (mode ?discard-goal-mode) (outcome ?discard-goal-outcome))
  (goal-meta (goal-id ?discard-goal) (order-id ?order-id))

  (wm-fact (key order meta wp-for-order args? wp ?wp-for-order ord ?order-id))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp-for-order col ?cap-color $?))
  (wm-fact (key domain fact order-cap-color args? ord ?order-id col ?order-cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?cs op ?cs-op))
  =>
  ;nuke the production tree
  (goal-reasoner-nuke-subtree ?root)
  ;nuke the instruction tree
  (goal-reasoner-nuke-subtree ?instruct-root)
)

(defrule goal-reasoner-remove-wp-facts-on-removed-order-parent
  "When the root of an order is removed, remove the facts describing the wp for the order"
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order-id))
  (not (goal-meta (root-for-order ?order-id)))
  =>
	(assert (wm-fact (key monitoring cleanup-wp args? wp ?wp)))
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
