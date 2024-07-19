;---------------------------------------------------------------------------
;  goal-executability.clp - Check executability of production goals
;
;  Created: Sat 30 Apr 2022 18:44:00 CET
;  Copyright  2021  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
;             2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;             2021  Sonja Ginter <sonja.ginter@rwth-aachen.de>
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
  ?*PRODUCE-C0-AHEAD-TIME* = 150
  ?*PRODUCE-C1-AHEAD-TIME* = 250
  ?*PRODUCE-C2-AHEAD-TIME* = 350
  ?*PRODUCE-C3-AHEAD-TIME* = 450
  ?*DELIVER-AHEAD-TIME* = 60
)

(deffunction is-free (?target-pos)
	(if (any-factp ((?at wm-fact))
	        (and (wm-key-prefix ?at:key (create$ domain fact at))
	             (eq (wm-key-arg ?at:key m) ?target-pos)))
	 then
		(return FALSE)
	 else
		(return TRUE)
	)
)

(defrule goal-executability-ground-precondition
  " Create grounded pddl formulas for the precondition of a goal that is assigned to a robot.
  "
  (declare (salience ?*SALIENCE-DOMAIN-GROUND*))
  (goal (id ?id) (class ?class) (params $?params) (outcome UNKNOWN) (sub-type SIMPLE))
  ?gm <- (goal-meta (goal-id ?id) (assigned-to ?robot&~nil) (precondition nil))
  (domain-operator (name ?operator-id&:(eq ?operator-id (sym-cat goal- (lowcase ?class)))) (param-names $?op-param-names&:(= (length$ ?op-param-names) (+ 1 (/ (length$ $?params) 2)))))
  (pddl-formula (part-of ?operator-id))
  =>
  (bind ?param-names (insert$ (names-from-name-value-list ?params) 1 r))
  (bind ?param-values (insert$ (values-from-name-value-list ?params) 1 ?robot))
  (bind ?grounding (ground-pddl-formula ?operator-id ?param-names ?param-values nil))
  (printout debug "ground precondition for " ?id crlf)
  (modify ?gm (precondition ?grounding))
)

(defrule goal-executability-check-if-goal-precondition-is-satisfied
  "Check if there is a referenced precondition formula that is satisfied,
  if yes make the goal executable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?g <- (goal (id ?id) (class ?class) (params $?params) (sub-type SIMPLE) (is-executable FALSE))
  (goal-meta (goal-id ?id) (assigned-to ?robot&~nil) (precondition ?grounding-id))
  (pddl-formula (part-of ?operator&:(eq ?operator (sym-cat goal- (lowcase ?class)))) (id ?formula-id))
  (grounded-pddl-formula (is-satisfied TRUE) (formula-id ?formula-id) (grounding ?grounding-id))
  (pddl-grounding (id ?grounding-id))
  =>
  (modify ?g (is-executable TRUE))
  (printout t "Goal " ?id " is executable" crlf)
  (printout debug "based on grounding " ?grounding-id " of formula " ?formula-id crlf)
)

(defrule goal-executability-check-if-goal-precondition-is-unsatisfied
  "Check if all referenced precondition formulas are not satisfied,
  if yes make the action not executable."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?g <- (goal (id ?id) (class ?class) (params $?params) (sub-type SIMPLE) (is-executable TRUE))
  (goal-meta (goal-id ?id) (assigned-to ?robot&~nil) (precondition ?grounding-id))
  (pddl-formula (part-of ?operator&:(eq ?operator (sym-cat goal- (lowcase ?class)))) (id ?formula-id))
  (not (grounded-pddl-formula (is-satisfied TRUE) (formula-id ?formula-id) (grounding ?grounding-id)))
  (pddl-grounding (id ?grounding-id))
  =>
  (modify ?g (is-executable FALSE))
  (printout debug "Goal " ?id " is no longer executable" crlf)
)

(defrule goal-executability-retract-grounding-for-goal-if-precondition-mismatch
  " Remove the grounding as soon as a goal changes it params or assigned robot..
  "
  (declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
  ?pg <- (pddl-grounding (param-values $?param-values) (id ?grounding))
  ?g <- (goal (id ?id) (class ?class) (params $?goal-params) (sub-type SIMPLE) (is-executable ?is-executable))
  ?gm <- (goal-meta (goal-id ?id) (assigned-to ?robot) (precondition ?grounding))
  (test (neq $?param-values (create$ ?robot (values-from-name-value-list $?goal-params))))
  =>
  (retract ?pg)
  (printout debug "Goal " ?id " changed, remove grounding" crlf)
  (modify ?gm (precondition nil))
  (if ?is-executable then (modify ?g (is-executable FALSE)))
)

(defrule goal-executability-goal-done
  "After the effects of an action have been applied, change it to FINAL."
  (declare (salience ?*SALIENCE-DOMAIN-APPLY*))
  ?g <- (goal (id ?id) (class ?class) (params $?goal-params) (sub-type SIMPLE) (outcome ~UNKNOWN) (is-executable ?is-executable))
  ?gm <- (goal-meta (goal-id ?id) (assigned-to ?robot) (precondition ?grounding&:(neq ?grounding nil)))
  =>
  (printout debug "Goal " ?id " done, remove grounding" crlf)
  (modify ?gm (precondition nil))
  (domain-retract-grounding ?grounding)
  (if ?is-executable then (modify ?g (is-executable FALSE)))
)

(defrule goal-executability-grounding-retract-no-action
  (domain-operator (name ?op-name&:(not (str-index goal- ?op-name))) (param-names $?param-names))
  ?pg <- (pddl-grounding (id ?id&:(str-index (sym-cat "grounding-" ?op-name "1-gen") ?id))
                                               (param-names $?param-names)
                                               (param-values $?param-values))
  (not (plan-action
          (action-name ?op-name)
          (param-values $?param-values)
        )
  )
  =>
  (retract ?pg)
)

(defrule goal-executability-grounding-retract-no-goal
  (domain-operator (name ?op-name&:(str-index goal- ?op-name)) (param-names $?param-names))
  ?pg <- (pddl-grounding (id ?id&:(str-index (sym-cat "grounding-" ?op-name "1-gen") ?id))
                                               (param-names $?param-names)
                                               (param-values $?param-values))
  (not (goal (class ?class&:(eq (sym-cat goal- (lowcase ?class)) ?op-name))))
  =>
  (retract ?pg)
)


; ----------------------- Production GOALS -------------------------------

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

(defrule goal-production-cleanup-wp-executable
	(declare (salience (- ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (id ?id) (class CLEANUP-WP) (sub-type SIMPLE)
				(mode FORMULATED) (is-executable FALSE)

	)
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact holding args? r ?robot wp ?wp))
	(not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?any-order)))
	(not (goal (params $? ?wp $?)))
	=>
	(printout t "Goal CLEANUP-WP executable for " ?robot " and WP " ?wp  crlf)
 	(modify ?g (is-executable TRUE))
)

(defrule goal-production-pick-and-place-executable
"Check executability for pick and place
 Picks a wp from the output of the given mps
  and feeds it into the input of the same mps"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?id) (class PICK-AND-PLACE) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-mps ?mps )
	            (is-executable FALSE))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact mps-side-free args? m ?mps side INPUT))
	(or (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	(wm-fact (key domain fact maps args? m ?mps r ?robot))
	(domain-fact (name zone-content) (param-values ?zz ?mps))
	=>
	(printout t "Goal PICK-AND-PLACE executable for " ?robot " at " ?mps crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-move-robot-to-output-executable
"Check executability to move to the output of the given mps. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?id) (class MOVE) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-mps ?mps )
	            (is-executable FALSE))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact maps args? m ?mps r ?robot))
	=>
	(printout t "Goal MOVE executable for " ?robot " at " ?mps crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-wait-nothing-executable-executable
	(declare (salience ?*SALIENCE-GOAL-REJECT*))
	?g <- (goal (id ?oid) (class WAIT-NOTHING-EXECUTABLE)
	            (mode FORMULATED) (is-executable FALSE))
	(not (goal (class ~WAIT-NOTHING-EXECUTABLE)
	            (mode FORMULATED) (is-executable TRUE)))
	(goal-meta (goal-id ?oid) (assigned-to ~nil&~central))
	=>
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-remove-retracted-wait-nothing-executable
  "When a wait-nothing-executable goal is retracted, remove it to prevent spam"
  ?g <- (goal (class WAIT-NOTHING-EXECUTABLE) (mode RETRACTED))
  =>
  (retract ?g)
)
