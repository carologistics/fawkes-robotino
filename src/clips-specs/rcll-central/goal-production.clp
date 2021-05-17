;---------------------------------------------------------------------------
;  goal-maintain-production.clp - Generate production goals of RCLL
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

(deffunction assign-robot-to-goal (?meta ?robot)
	(if (eq ?meta nil) then (return (create$ assigned-to ?robot)))
	(bind ?pos (member$ assigned-to ?meta))
	(if ?pos
	 then
		(return (replace$ ?meta ?pos (+ ?pos 1) (create$ assigned-to ?robot)))
	 else
		(return (create$ ?meta assigned-to ?robot))
	)
)

(deffunction remove-robot-assignment-from-goal (?meta ?robot)
	(bind ?pos (member$ assigned-to ?meta))
	(bind ?pos2 (member$ ?robot ?meta))
	(if (and ?pos ?pos2 (eq ?pos2 (+ ?pos 1)))
	 then
		(return (delete$ ?meta ?pos ?pos2))
	 else
		(return ?meta)
	)
)

(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (final TRUE))
  (not (NavGraphWithMPSGeneratorInterface (final ~TRUE)))
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
)


(defrule goal-production-create-beacon-maintain
" The parent goal for beacon signals. Allows formulation of
  goals that periodically communicate with the refbox.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class BEACON-MAINTAIN)))
  (or (domain-facts-loaded)
      (wm-fact (key refbox phase) (value ~SETUP&~PRE_GAME)))
  =>
  (bind ?goal (goal-tree-assert-run-endless BEACON-MAINTAIN 1))
  (modify ?goal (verbosity QUIET) (params frequency 1))
)


(defrule goal-production-create-beacon-achieve
" Send a beacon signal whenever at least one second has elapsed since it
  last one got sent.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (time $?now)
  ?g <- (goal (id ?maintain-id) (class BEACON-MAINTAIN) (mode SELECTED))
  ; TODO: make interval a constant
  =>
  (assert (goal (id (sym-cat SEND-BEACON- (gensym*))) (sub-type SIMPLE)
                (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET)))
)

(defrule goal-production-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
  (not (mutex (name ?n&:(eq ?n (resource-to-mutex refill-shelf))) (state LOCKED)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  =>
  (bind ?goal (goal-tree-assert-run-endless REFILL-SHELF-MAINTAIN 1))
  (modify ?goal (required-resources refill-shelf)
                (params frequency 1 retract-on-REJECTED)
                (verbosity QUIET))
)


(defrule goal-production-create-refill-shelf-achieve
  "Refill a shelf whenever it is empty."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?maintain-id) (class REFILL-SHELF-MAINTAIN) (mode SELECTED))
  (not (goal (class REFILL-SHELF)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  =>
  (assert (goal (id (sym-cat REFILL-SHELF- (gensym*)))
                (class REFILL-SHELF) (sub-type SIMPLE)
                (parent ?maintain-id) (verbosity QUIET)
                (params mps ?mps)))
)


(defrule goal-production-assign-robot-to-simple-goals
" Before checking SIMPLE goals for their executability, pick a waiting robot
  that should get a new goal assigned to it next. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	(goal (sub-type SIMPLE) (mode FORMULATED)
	      (meta $?meta&:(not (member$ assigned-to ?meta)))
	      (is-executable FALSE))
	(wm-fact (key central agent robot args? r ?robot))
	(not (goal (meta $? assigned-to ?robot $?)))
	(wm-fact (key central agent robot-waiting args? r ?robot))
	(not (and (wm-fact (key central agent robot-waiting
	                    args? r ?o-robot&:(> (str-compare ?robot ?o-robot) 0)))
	          (not (goal (meta $? assigned-to ?o-robot $?)))))
	=>
	(delayed-do-for-all-facts ((?g goal))
		(and (not (member$ assigned-to ?g:meta)) (eq ?g:is-executable FALSE)
		     (eq ?g:sub-type SIMPLE) (eq ?g:mode FORMULATED))
		(modify ?g (meta (assign-robot-to-goal ?meta ?robot)))
	)
)

(defrule goal-production-unassign-robot-from-simple-goals
" A waiting robot got a new goal, clear executability and robot assignment from other goals. "
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (sub-type SIMPLE) (mode SELECTED)
	      (meta $? assigned-to ?robot $?)
	      (is-executable TRUE))
	(wm-fact (key central agent robot args? r ?robot))
	(goal (sub-type SIMPLE) (mode FORMULATED)
	      (meta $? assigned-to ?robot $?))
	?waiting <- (wm-fact (key central agent robot-waiting args? r ?robot))
	=>
	(delayed-do-for-all-facts ((?g goal))
		(and (subsetp (create$ assigned-to ?robot) ?g:meta)
		     (eq ?g:mode FORMULATED))
		(modify ?g (meta (remove-robot-assignment-from-goal ?g:meta ?robot))
		           (is-executable FALSE))
	)
	(retract ?waiting)
)

(defrule goal-production-enter-field-executable
 " ENTER-FIELD is executable for a robot if it has not entered the field yet."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class ENTER-FIELD) (sub-type SIMPLE) (mode FORMULATED)
	      (params team-color ?team-color) (meta $? assigned-to ?robot $?)
	      (is-executable FALSE))

	(wm-fact (key refbox state) (value RUNNING))
	(wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
	(wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE))
	(wm-fact (key refbox team-color) (value ?team-color))
	; (NavGraphGeneratorInterface (final TRUE))
	(not (wm-fact (key domain fact entered-field
	               args? r ?robot team-color ?team-color)))
	=>
	(printout t "Goal ENTER-FIELD executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-hack-failed-enter-field
  "HACK: Stop trying to enter the field when it failed a few times."
  ; TODO-GM: this was after 3 tries, now its instantly
  ?g <- (goal (id ?gid) (class ENTER-FIELD)
               (mode FINISHED) (outcome FAILED))
  ?pa <- (plan-action (goal-id ?gid) (state FAILED) (action-name enter-field))
  =>
  (printout t "Goal '" ?gid "' has failed, evaluating" crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
  (modify ?g (mode DISPATCHED) (outcome UNKNOWN))
)
