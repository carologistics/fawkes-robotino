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

; (deffunction goal-reasoner-nuke-subtree (?goal)
;   "Remove an entire subtree."
;   (do-for-all-facts ((?child goal)) (eq ?child:parent (fact-slot-value ?goal id))
;     (goal-reasoner-nuke-subtree ?child)
;   )
;   (retract ?goal)
; )

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

; (deffunction remove-robot-assignment-from-goal-meta (?goal)
;   (if (not (do-for-fact ((?f goal-meta))
;       (eq ?f:goal-id (fact-slot-value ?goal id))
;       (modify ?f (assigned-to nil))
;       ))
;    then
;     (printout t "Cannot find a goal meta fact for the goal " ?goal crlf)
;   )
; )

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
  (not (reset-game (stage STAGE-0|STAGE-1|STAGE-2|STAGE-3)))
  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-init-selection-criteria
  (domain-loaded)
  (not (wm-fact (key goal selection criterion args? t ?)))
  (not (reset-game (stage STAGE-0|STAGE-1|STAGE-2|STAGE-3)))
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
  (not (reset-game (stage STAGE-0|STAGE-1|STAGE-2|STAGE-3)))
  =>
  ;(printout (log-debug ?v) "Goal " (fact-slot-value ?target-goal id) " SELECTED" crlf)
  ;(modify ?target-goal (mode SELECTED))
  (printout (log-debug ?v) "Goal " (fact-slot-value ?target-goal id) " is root for order - asserted rl-waiting fact" crlf crlf)
  (assert (rl-waiting))
)


(defrule goal-reasoner-add-selectable-root-goal
  (declare (salience ?*SALIENCE-GOAL-PRE-SELECT*))
  (not (reset-game (stage STAGE-0|STAGE-1|STAGE-2|STAGE-3)))
  (goal (type ACHIEVE) (id ?goal-id) (parent nil) (mode FORMULATED) (is-executable TRUE))
  (goal-meta (goal-id ?goal-id) (assigned-to central|nil))
  ?selection <- (wm-fact (key goal selection criterion args? t root) (values $?values&:(not (member$ ?goal-id ?values))))
  =>
  (modify ?selection (values (append$ ?values ?goal-id)))
)

(defrule goal-reasoner-add-selectable-run-all-goal
  (declare (salience ?*SALIENCE-GOAL-PRE-SELECT*))
  (not (reset-game (stage STAGE-0|STAGE-1|STAGE-2|STAGE-3)))
  (goal (type ACHIEVE) (id ?parent-id) (mode DISPATCHED) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS))
  (goal (type ACHIEVE) (id ?goal-id) (parent ?parent-id) (mode FORMULATED) (is-executable TRUE))
  (goal-meta (goal-id ?goal-id) (run-all-ordering ?ordering) (assigned-to central|nil))
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
  (not (reset-game (stage STAGE-0|STAGE-1|STAGE-2|STAGE-3)))
  (goal (type ACHIEVE) (id ?parent-id) (mode DISPATCHED) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL))
  (goal (type ACHIEVE) (id ?goal-id) (parent ?parent-id) (mode FORMULATED)
        (is-executable TRUE) (priority ?priority))
  (goal-meta (goal-id ?goal-id) (assigned-to central|nil))
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
  (not (reset-game (stage STAGE-0|STAGE-1|STAGE-2|STAGE-3)))
  (wm-fact (key goal selection criterion args? t ?) (values ?some-goal-id $?))
  ?some-goal <- (goal (id ?some-goal-id) (class ~ENTER-FIELD) (priority ?some-prio))
  (not (goal (mode SELECTED|EXPANDED|COMMITTED) (type ACHIEVE)))
  (goal-meta (goal-id ?some-goal-id) (assigned-to central|nil))
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
  (printout error (fact-slot-value ?highest-prio-goal-fact id) crlf)
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


; ----------------------- EVALUATE GOALS ---------------------------

(deffunction goal-reasoner-retract-plan-action (?goal-id)
  (delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
    (delayed-do-for-all-facts ((?a plan-action)) (and (eq ?a:plan-id ?p:id) (eq ?a:goal-id ?goal-id))
      (retract ?a)
    )
    (retract ?p)
  )
)

(defrule goal-reasoner-remove-maintain-refill-shelf
  "Remove the maintain refill shelf goal if it is evaluated and has no children
   to enable reformulation."
  ?g <- (goal (id ?root-id)
              (class MAINTAIN-REFILL-SHELF)
              (mode EVALUATED)
              (outcome COMPLETED)
              (verbosity ?v))
  (not (goal (parent ?root-id)))
  =>
  (printout (log-debug ?v) "Goal " ?root-id ", remove goal as it was completed" crlf)
  (retract ?g)
)

(defrule goal-reasoner-remove-refill-shelf
  "Remove the refill shelf goal if it is retracted for clean-up."
  ?g <- (goal (id ?goal-id)
              (class REFILL-SHELF)
              (mode RETRACTED)
              (outcome COMPLETED)
              (verbosity ?v))
  =>
  (printout (log-debug ?v) "Goal " ?goal-id ", remove goal as it was completed" crlf)
  (retract ?g)
)

(defrule goal-reasoner-evaluate-clean-up-failed-order-root
  "Once all requests have been removed, a failed order tree root can be safely
  cleaned up, thus freeing capacity for starting new orders."
  (declare (salience ?*MONITORING-SALIENCE*))
  (goal (id ?root-id) (outcome FAILED) (mode FINISHED|EVALUATED|RETRACTED))
  ?gm <- (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))
  (not (wm-fact (key request ? args? ord ?order-id $?)))
  =>
  (modify ?gm (root-for-order nil))
)

(defrule goal-reasoner-evaluate-production-and-maintenance-wp-still-usable
  "If a production or maintenance goal failed but the WP is still usable "
  (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))
  ?g <- (goal (class ?class)
              (error $?errors&:(not (or (member$ WP-LOST ?errors) (member$ BROKEN-MPS ?errors) (member$ INTERACTED-WITH-BROKEN-MPS ?errors))));exclude special cases
              (id ?goal-id)
              (mode FINISHED|EVALUATED|RETRACTED)
              (outcome FAILED)
              (verbosity ?v))
  ?gm <- (goal-meta (goal-id ?goal-id) (assigned-to ?robot)
             (category ?category&PRODUCTION|MAINTENANCE|PRODUCTION-INSTRUCT|MAINTENANCE-INSTRUCT) (retries ?retries))
  =>
  (if (not
        (or
          (eq ?category PRODUCTION-INSTRUCT)
          (eq ?category MAINTENANCE-INSTRUCT)
        )
      )
      then
        (set-robot-to-waiting ?robot)
        (remove-robot-assignment-from-goal-meta ?g)
  )
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, reformulate as workpiece is still usable after fail" crlf)
  (modify ?g (mode FORMULATED) (outcome UNKNOWN))
  (modify ?gm (retries (+ 1 ?retries)))

  (goal-reasoner-retract-plan-action ?goal-id)
)

(defrule goal-reasoner-evaluate-production-goal-failed-wp-lost
  "If a production goal was failed because the WP was lost,
  clean-up the goal tree and requests
  and let the production selector re-decide which order to pursue."
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (class ?class)
        (id ?goal-id)
        (error WP-LOST)
        (mode FINISHED)
        (outcome FAILED)
        (verbosity ?v)
  )
  (goal-meta (goal-id ?goal-id) (order-id ?order-id) (assigned-to ?robot)
             (category ?category&PRODUCTION|PRODUCTION-INSTRUCT))
  ?order-root <- (goal (id ?root-id) (outcome ~FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id))
  =>
  (if (not
          (eq ?category PRODUCTION-INSTRUCT)
      )
      then
        (set-robot-to-waiting ?robot)
        (remove-robot-assignment-from-goal-meta ?g)
  )
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, aborting order as the WP was lost." crlf)
  ;first fail the order root and the production/production-instruct goals
  ;this operation should be save, since only one production goal is gonna
  ;run at at time. Failing the order root will also trigger a clean-up of requests
  ;and thus of maintenance goals and their instructs.
  (modify ?order-root (mode FINISHED) (outcome FAILED) (error WP-LOST))
  (do-for-all-facts ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:id ?goal-meta:goal-id)
      (eq ?goal-meta:order-id ?order-id)
      (or
        (eq ?category PRODUCTION)
        (eq ?category PRODUCTION-INSTRUCT)
      )
      (member$ ?goal:mode (create$ FORMULATED SELECTED EXPANDED))
    )
    (modify ?goal (mode FINISHED) (outcome FAILED) (error WP-LOST))
  )
  ;next gracefully stop all dispatched production and production-instruct goals
  ;by marking them for failure which will cause them to fail once no action
  ;of their plan is running
  (do-for-all-facts ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:id ?goal-meta:goal-id)
      (eq ?goal-meta:order-id ?order-id)
      (or
        (eq ?category PRODUCTION)
        (eq ?category PRODUCTION-INSTRUCT)
      )
      (member$ ?goal:mode (create$ DISPACTHED))
      (eq ?goal:outcome UNKNOWN)
    )
	  (assert (wm-fact (key monitoring fail-goal args? g ?goal:id)))
  )

  (goal-reasoner-retract-plan-action ?goal-id)
)

(defrule goal-reasoner-evaluate-production-goal-failed-broken-retry
  "If a production goal was failed because it interacted with a broken mps,
  and we are late in the production process, we reformulate the goal to not lose
  our progress."
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (class ?class)
        (id ?goal-id)
        (error BROKEN-MPS)
        (mode FINISHED)
        (outcome FAILED)
        (verbosity ?v)
  )
  ?gm <- (goal-meta (goal-id ?goal-id) (order-id ?order-id) (assigned-to ?robot)
             (category ?category&PRODUCTION|PRODUCTION-INSTRUCT) (retries ?retries))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order-id))
  (wm-fact (key wp meta next-step args? wp ?wp) (value ?step))
  (test
    (or
      (and
        (eq ?comp C0)
        (eq ?step DELIVER)
      )
      (and
        (eq ?comp C1)
        (eq ?step DELIVER)
      )
      (and
        (eq ?comp C2)
        (or
          (eq ?step CAP)
          (eq ?step DELIVER)
        )
      )
      (and
        (eq ?comp C3)
        (or
          (eq ?step RING3)
          (eq ?step CAP)
          (eq ?step DELIVER)
        )
      )
    )
  )
  =>
  (if (not
          (eq ?category PRODUCTION-INSTRUCT)
      )
      then
        (set-robot-to-waiting ?robot)
        (remove-robot-assignment-from-goal-meta ?g)
  )
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, failed due to broken mps. Reformulate as we already progressed far." crlf)
  (goal-reasoner-retract-plan-action ?goal-id)
  (modify ?g (mode FORMULATED) (outcome UNKNOWN))
  (modify ?gm (retries (+ 1 ?retries)))
)

(defrule goal-reasoner-evaluate-production-goal-failed-broken-abort
  "If a production goal was failed because it interacted with a broken mps,
  and gently abort the order, as we probably lost the WP."
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (class ?class)
        (id ?goal-id)
        (error ?error&:(or (eq ?error INTERACTED-WITH-BROKEN-MPS) (eq ?error BROKEN-MPS)))
        (mode FINISHED)
        (outcome FAILED)
        (verbosity ?v)
  )
  (goal-meta (goal-id ?goal-id) (order-id ?order-id) (assigned-to ?robot)
             (category ?category&PRODUCTION|PRODUCTION-INSTRUCT))
  ?order-root <- (goal (id ?root-id) (outcome ~FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id))
  =>
  (if (not
          (eq ?category PRODUCTION-INSTRUCT)
      )
      then
        (set-robot-to-waiting ?robot)
        (remove-robot-assignment-from-goal-meta ?g)
  )
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, aborting order as we are early in the production process." crlf)
  ;first fail the order root and the production/production-instruct goals
  ;this operation should be save, since only one production goal is gonna
  ;run at at time. Failing the order root will also trigger a clean-up of requests
  ;and thus of maintenance goals and their instructs.
  (modify ?order-root (mode FINISHED) (outcome FAILED) (error ?error))
  (do-for-all-facts ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:id ?goal-meta:goal-id)
      (eq ?goal-meta:order-id ?order-id)
      (or
        (eq ?category PRODUCTION)
        (eq ?category PRODUCTION-INSTRUCT)
      )
      (member$ ?goal:mode (create$ FORMULATED SELECTED EXPANDED COMMITTED))
    )
    (modify ?goal (mode FINISHED) (outcome FAILED) (error WP-LOST))
  )
  ;next gracefully stop all dispatched production and production-instruct goals
  ;by marking them for failure which will cause them to fail once no action
  ;of their plan is running
  (do-for-all-facts ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:id ?goal-meta:goal-id)
      (eq ?goal-meta:order-id ?order-id)
      (or
        (eq ?category PRODUCTION)
        (eq ?category PRODUCTION-INSTRUCT)
      )
      (member$ ?goal:mode (create$ DISPACTHED))
      (eq ?goal:outcome UNKNOWN)
    )
	  (assert (wm-fact (key monitoring fail-goal args? g ?goal:id)))
  )
  (goal-reasoner-retract-plan-action ?goal-id)
)

(defrule goal-reasoner-evaluate-maintenance-goal-failed-wp-lost-retry
  "If a maintenance goal was failed because the WP was lost, a cap-carrier, base,
  or similar was lost. Reformulate the goal."
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (class ?class)
        (id ?goal-id)
        (error ?error&:(eq ?error WP-LOST))
        (mode FINISHED)
        (outcome FAILED)
        (verbosity ?v)
  )
  ?gm <- (goal-meta (goal-id ?goal-id) (order-id ?order-id) (assigned-to ?robot)
             (category ?category&MAINTENANCE|MAINTENANCE-INSTRUCT) (retries ?retries))
  (test (neq ?class PAY-FOR-RINGS-WITH-CAP-CARRIER))
  =>
  (modify ?gm (retries (+ 1 ?retries)))
  (if (not
          (eq ?category MAINTENANCE-INSTRUCT)
      )
      then
        (set-robot-to-waiting ?robot)
        (remove-robot-assignment-from-goal-meta ?g)
  )
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, reformulate as the support WP was lost" crlf)
  (modify ?g (mode FORMULATED) (outcome UNKNOWN))

  (goal-reasoner-retract-plan-action ?goal-id)
)

(defrule goal-reasoner-evaluate-maintenance-goal-discard-failed
  "If a discard goal fails, reformulate it as formulated."
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (id ?goal-id) (class DISCARD) (mode FINISHED) (outcome FAILED)
              (verbosity ?v) (params wp ?wp $?))
  ?gm <- (goal-meta (goal-id ?goal-id) (assigned-to ?robot) (retries ?retries))
  =>
  (set-robot-to-waiting ?robot)
  (modify ?gm (retries (+ 1 ?retries)))
  (remove-robot-assignment-from-goal-meta ?g)
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, reformulate and dispatch with classical drop discard" crlf)
  (modify ?g (mode FORMULATED) (outcome UNKNOWN))
  (goal-reasoner-retract-plan-action ?goal-id)
)

(defrule goal-reasoner-evaluate-failed-goto
  "Re-formulate a failed goal if the reason was a failed go-to action as the
  best course of action is to just retry from the agent's perspective."
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (id ?goal-id) (mode FINISHED) (outcome FAILED) (meta $?meta)
              (verbosity ?v))
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (action-name ?action&move|go-wait|wait-for-wp|wait-for-free-side)
               (goal-id ?goal-id) (plan-id ?plan-id) (state FAILED))
  ?gm <- (goal-meta (goal-id ?goal-id) (assigned-to ?robot) (retries ?retries))
  =>
  (set-robot-to-waiting ?robot)
  (modify ?gm (retries (+ 1 ?retries)))
  (remove-robot-assignment-from-goal-meta ?g)
  (printout (log-debug ?v) "Goal " ?goal-id " EVALUATED, reformulate as only a " ?action " action failed" crlf)
  (modify ?g (mode FORMULATED) (outcome UNKNOWN))
  (goal-reasoner-retract-plan-action ?goal-id)
)

(defrule goal-reasoner-evaluate-move-out-of-way-cleanup-wp
" Sets a finished move-out-of-way or empty discard goal to formulated."
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (id ?goal-id) (class ?class&MOVE-OUT-OF-WAY|CLEANUP-WP) (mode FINISHED)
              (outcome ?outcome) (verbosity ?v))
  ?gm <- (goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (retries ?retries))
  =>
  (printout (log-debug ?v) "Evaluate " ?class " goal " ?goal-id crlf)
  (set-robot-to-waiting ?robot)
  (modify ?gm (retries (+ 1 ?retries)))
  (remove-robot-assignment-from-goal-meta ?g)

  ; delete plans of the goal
  (goal-reasoner-retract-plan-action ?goal-id)
  (modify ?g (mode FORMULATED) (outcome UNKNOWN) (is-executable FALSE))
  (printout (log-debug ?v) "Goal " ?goal-id " FORMULATED" crlf)
)

(defrule goal-reasoner-evaluate-buffer-cap-wrong-slot
" Sets a finished move-out-of-way or empty discard goal to formulated."
  (declare (salience ?*MONITORING-SALIENCE*))
  ?g <- (goal (id ?goal-id) (class BUFFER-CAP) (mode FINISHED)
              (outcome FAILED) (verbosity ?v))
  ?gm <- (goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (retries ?retries))
  (plan-action (goal-id ?goal-id) (state FAILED) (error-msg "Unsatisfied precondition") (action-name wp-get-shelf) (param-values ?robot ? ?cs ?spot))
  ?wp-on-shelf-fact <- (wm-fact  (key domain fact wp-on-shelf args? wp ? m ?cs spot ?spot))
  (wm-fact  (key domain fact wp-on-shelf args? wp ? m ?cs spot ?other-spot&:(neq ?other-spot ?spot)))
  =>
  (printout (log-debug ?v) "Evaluate buffer-cap goal " ?goal-id ". Retrying as we can just try a different slot." crlf)
  (set-robot-to-waiting ?robot)
  (remove-robot-assignment-from-goal-meta ?g)

  ; delete plans of the goal
  (goal-reasoner-retract-plan-action ?goal-id)
  (modify ?g (mode FORMULATED) (outcome UNKNOWN) (is-executable FALSE))
  (modify ?gm (retries (+ 1 ?retries)))
  (printout (log-debug ?v) "Goal " ?goal-id " FORMULATED" crlf)
  (retract ?wp-on-shelf-fact)
  (printout (log-debug ?v) "Removed WP from shelf on " ?cs " slot " ?spot crlf)
)

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

; ================================= Goal Clean up ============================

(defrule goal-reasoner-retract-achieve
" Retract a goal if all sub-goals are retracted.
"
  ?g <-(goal (id ?goal-id) (type ACHIEVE) (mode EVALUATED)
             (acquired-resources) (verbosity ?v))
  (not (goal (parent ?goal-id) (mode ?mode&~RETRACTED)))
=>
  (printout (log-debug ?v) "Goal " ?goal-id " RETRACTED" crlf)
  (modify ?g (mode RETRACTED))
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
  ?g <- (goal (id ?gid) (class ?class) (parent ?pid&~nil))
  (not (goal (id ?pid)))
  =>
  (retract ?g)
  (printout error ?gid " of class " ?class " has a non-existing parent and was removed" crlf)
)

(deffunction is-goal-running (?mode)
  (return (or (eq ?mode SELECTED) (eq ?mode EXPANDED)
              (eq ?mode COMMITTED) (eq ?mode DISPATCHED)))
)
