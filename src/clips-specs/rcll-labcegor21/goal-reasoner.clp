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

(deffunction goal-tree-assert-central-run-all-sequence (?class $?fact-addresses)
	(bind ?id (sym-cat CENTRAL-RUN-ALL- ?class - (gensym*)))
	(bind ?goal
    (assert (goal (id ?id) (class ?class) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
                  (meta sequence-mode)))
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
  ; (not (goal (parent ?goal-id)))
=>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-order-helper-goals
  "Select buffer/pay/discard goal if nothing else is executable."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (class ~PRODUCE-ORDER) (mode FORMULATED) (is-executable TRUE))
  (goal (id ?parent-id) (class PRODUCTION-ROOT))
  ?assignment <- (goal-meta (goal-id ?goal-id) (assigned-to nil))

   ; We have a robot that isn't doing anything.
	(wm-fact (key central agent robot args? r ?robot))
	(not (and (goal (id ?other-goal) (is-executable TRUE)
					        (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED))
		        (goal-meta (goal-id ?other-goal) (assigned-to ?robot))))

  ; And we don't have an executable production goal.
  (not (and (goal (parent ?other-parent-id) (mode FORMULATED) (is-executable TRUE))
            (goal (id ?other-parent-id) (class PRODUCE-ORDER))))
  =>
  (printout t "Production helper goal " ?goal-id " SELECTED and assigned to " ?robot crlf)
  (modify ?assignment (assigned-to ?robot))
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-produce-order-goal
  "Select goals needed to produce an order."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode FORMULATED) (is-executable TRUE))
  (goal (id ?parent-id) (class PRODUCE-ORDER) (params order ?order-id))
  ?assignment <- (goal-meta (goal-id ?goal-id) (assigned-to nil))

  ; We have a robot that isn't doing anything.
	(wm-fact (key central agent robot args? r ?robot))
	(not (and (goal (id ?other-goal) (is-executable TRUE)
					        (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED))
		        (goal-meta (goal-id ?other-goal) (assigned-to ?robot))))

  ; We don't want to handle it if there is another order with earlier delivery end.
  (wm-fact (key refbox order ?order-id delivery-end) (value ?order-end))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?order-com))
  (not (and (goal (id ?other-goal-id) (parent ?other-parent-id) (mode FORMULATED) (is-executable TRUE))
            (goal (id ?other-parent-id) (class PRODUCE-ORDER) (params order ?other-order-id))
            (not (goal-meta (goal-id ?other-goal-id) (assigned-to central)))
            (wm-fact (key refbox order ?other-order-id delivery-end) (value ?other-order-end))
            (wm-fact (key domain fact order-complexity args? ord ?other-order-id com ?other-order-com))
            (or
              (and (test (= (str-compare ?other-order-com ?order-com) 0))
                   (test (< ?other-order-end ?order-end))
              ) 
              (test (> (str-compare ?other-order-com ?order-com) 0))
            )
            
  ))
  =>
  (printout t "Production goal " ?goal-id " for order " ?order-id " SELECTED and assigned to " ?robot crlf)
  (do-for-all-facts ((?og goal)) (and (eq ?og:is-executable TRUE) (eq ?og:mode FORMULATED) (neq ?og:id ?goal-id))
    (do-for-all-facts ((?op goal)) (and (eq ?op:id ?og:parent) (eq ?op:class PRODUCE-ORDER))
      (printout t "Did not select " ?og:id " " ?op:params crlf)
    )
	)
  (modify ?assignment (assigned-to ?robot))
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-central-proudction-goal
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode FORMULATED) (is-executable TRUE))
  (goal (id ?parent-id) (class PRODUCE-ORDER|PRODUCTION-ROOT))
  (goal-meta (goal-id ?goal-id) (assigned-to central))
  =>
  (printout t "Production goal " ?goal-id " SELECTED for central" crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-simple-waiting-robot
  "Select all executable simple goals in order to propagate selection."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (type ACHIEVE) (sub-type SIMPLE)
              (id ?goal-id) (parent ?parent-id)
              (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  ; We must not select children of RUN-ENDLESS.
  (not (goal (id ?parent-id) (sub-type RUN-ENDLESS)))
  ; We want to handle goals of our production nodes separately.
  (not (goal (id ?parent-id) (class PRODUCTION-ROOT|PRODUCE-ORDER)))

  (wm-fact (key central agent robot-waiting args? r ?robot))

  (not (and (goal-meta (goal-id ?g-id) (assigned-to ?robot))
            (goal (id ?g-id) (mode ~FORMULATED))))

  ;(not (and (wm-fact (key central agent robot-waiting
  ;                    args? r ?o-robot&:(> (str-compare ?robot ?o-robot) 0)))
  ;     (goal-meta (assigned-to ?o-robot))))
  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)

(defrule goal-reasoner-select-root-central-executable-simple-goal
  "There is an executable simple goal assigned to central, propagate selection."
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal (id ?id) (sub-type SIMPLE) (mode FORMULATED) (is-executable TRUE))
  (goal-meta (goal-id ?id) (assigned-to central))
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

(defrule goal-reasoner-select-run-in-parallel
  "Select the goal of highest priority of a run parallel if it is dispatched and
  is executable"
  (declare (salience ?*SALIENCE-GOAL-SELECT*))
  (goal (id ?parent1) (mode DISPATCHED) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL) (priority ?p1))
  ?g <- (goal (id ?id) (parent ?parent1) (is-executable TRUE) (mode FORMULATED) (priority ?pc1))
  =>
  (modify ?g (mode SELECTED))
)

; ----------------------- EVALUATE COMMON ------------------------------------

(defrule goal-reasoner-evaluate-common
" Finally set a finished goal to evaluated.
  All pre evaluation steps should have been executed, enforced by the higher priority
"
	(declare (salience ?*SALIENCE-GOAL-EVALUATE-GENERIC*))
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome)
	            (verbosity ?v))
	(goal-meta (goal-id ?goal-id))
=>
	(printout (log-debug ?v) "Goal " ?goal-id " EVALUATED" crlf)
	(modify ?g (mode EVALUATED))
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

;(deffunction is-goal-running (?mode)
;	(return (or (eq ?mode SELECTED) (eq ?mode EXPANDED)
;	            (eq ?mode COMMITTED) (eq ?mode DISPATCHED)))
;)
