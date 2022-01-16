;---------------------------------------------------------------------------
;  goal-maintain-production.clp - Generate production goals of RCLL
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;             2021  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
;             2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

(deffunction goal-meta-assert (?goal ?robot)
"Creates the goal-meta fact and assign the goal to the robot"
	(if (neq ?robot nil) then
		(assert (goal-meta (goal-id (fact-slot-value ?goal id))
		                   (assigned-to ?robot)))
	)
	(return ?goal)
)

(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (id "/navgraph-generator-mps") (final TRUE))
   (forall
     (wm-fact (key central agent robot args? r ?robot))
     (NavGraphWithMPSGeneratorInterface (id ?id&:(eq ?id (remote-if-id ?robot "navgraph-generator-mps"))) (final TRUE))
   )
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
  (delayed-do-for-all-facts ((?wm wm-fact)) (wm-key-prefix ?wm:key (create$ central agent robot))
    (assert (wm-fact (key central agent robot-waiting args? r (wm-key-arg ?wm:key r))))
  )
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
	(not (goal (parent ?maintain-id)))
	(wm-fact (key central agent robot args? r ?r))
	=>
	(bind ?goal (assert (goal (id (sym-cat SEND-BEACON- (gensym*))) (sub-type SIMPLE)
	              (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET)
	              (meta-template goal-meta)
	              (is-executable TRUE))))
	(goal-meta-assert ?goal central)
)

(defrule goal-production-enter-field-executable
 " ENTER-FIELD is executable for a robot if it has not entered the field yet."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?id) (class ENTER-FIELD) (sub-type SIMPLE) (mode FORMULATED)
	      (params team-color ?team-color)
	      (is-executable FALSE))
	(goal-meta (goal-id ?id) (assigned-to ?robot))
	(wm-fact (key refbox state) (value RUNNING))
	(wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
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
	?g <- (goal (id ?gid) (class ENTER-FIELD)
	            (mode FINISHED) (outcome FAILED))
	?pa <- (plan-action (goal-id ?gid) (state FAILED) (action-name enter-field))
	=>
	(printout t "Goal '" ?gid "' has failed, evaluating" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(modify ?g (mode DISPATCHED) (outcome UNKNOWN))
)

(defrule goal-production-buffer-cap-executable
" Bring a cap-carrier from a cap stations shelf to the corresponding mps input
  to buffer its cap. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?id) (class BUFFER-CAP) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-mps ?mps
	                    cap-color ?cap-color
	            )
	            (is-executable FALSE))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(wm-fact (key refbox team-color) (value ?team-color))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
	(not (wm-fact (key domain fact cs-buffered args? m ?mps col ?any-cap-color)))
	(not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side INPUT)))
	; Capcarrier CEs
	(or (and
	        (not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
	        (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))
	        (not (plan-action (action-name wp-get-shelf) (param-values $? ?wp $?)))
	        (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	    )
	    (and
	        (wm-fact (key domain fact holding args? r ?robot wp ?cc))
	        (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	        (domain-object (name ?cc) (type cap-carrier))
	    )
	)
	=>
	(printout t "Goal BUFFER-CAP executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

; ----------------------- MPS Instruction GOALS -------------------------------

(defrule goal-production-instruct-cs-buffer-cap-executable
" Instruct cap station to buffer a cap. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class INSTRUCT-CS-BUFFER-CAP) (sub-type SIMPLE)
	             (mode FORMULATED)
	            (params target-mps ?mps
	                    cap-color ?cap-color
	             )
	             (is-executable FALSE))
	(not (goal (class INSTRUCT-CS-BUFFER-CAP) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
	(wm-fact (key refbox team-color) (value ?team-color))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
	(not (wm-fact (key domain fact cs-buffered args? m ?mps col ?any-cap-color)))
	; WP CEs
	(wm-fact (key domain fact wp-at args? wp ?cc m ?mps side INPUT))
	(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side OUTPUT)))
	=>
	(printout t "Goal INSTRUCT-CS-BUFFER-CAP executable" crlf)
	(modify ?g (is-executable TRUE))
)

(deffunction goal-production-assert-buffer-cap
	(?mps ?cap-color)

	(bind ?goal (assert (goal (class BUFFER-CAP)
	      (id (sym-cat BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta-template goal-meta)
	      (params target-mps ?mps
	              cap-color ?cap-color)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-instruct-cs-buffer-cap
	(?mps ?cap-color)

	(bind ?goal (assert (goal (class INSTRUCT-CS-BUFFER-CAP)
	      (id (sym-cat INSTRUCT-CS-BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta-template goal-meta)
	      (params target-mps ?mps
	              cap-color ?cap-color)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-enter-field
	(?team-color)

	(bind ?goal (assert (goal (class ENTER-FIELD)
	            (id (sym-cat ENTER-FIELD- (gensym*)))
	            (sub-type SIMPLE)
	            (verbosity NOISY) (is-executable FALSE)
	            (params team-color ?team-color)
	            (meta-template goal-meta)
	)))
	(return ?goal)
)


(defrule goal-production-create-production-root
	"Create the production root under which all production trees for the orders
	are asserted"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(domain-facts-loaded)
	(not (goal (class PRODUCTION-ROOT)))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key game state) (value RUNNING))
	(wm-fact (key refbox team-color) (value ?color))
	(not (wm-fact (key domain fact rs-ring-spec args? $? rn NA)))
	; Ensure that a MachineInfo was received already.
	; So if there are ring stations with specs, then those specs are registered.
	(wm-fact (key domain fact mps-state args? m ?any-mps s IDLE))
	(wm-fact (key domain fact entered-field args? r robot1))

	;cap
	;(wm-fact (key domain fact order-cap-color args? ord ?order-id col ?cap-color))
	;(wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel PRODUCTION-ROOT))
	(modify ?g (meta do-not-finish) (priority 1.0))
)

(deffunction goal-production-assert-instruct-ds-deliver
	(?wp)

	(bind ?goal (assert (goal (class INSTRUCT-DS-DELIVER)
	  (id (sym-cat INSTRUCT-DS-DELIVER- (gensym*))) (sub-type SIMPLE)
	  (verbosity NOISY) (is-executable FALSE)
	  (params wp ?wp
	          target-mps C-DS)
	)))
	(goal-meta-assert ?goal central)
	(return ?goal)
)

(deffunction goal-production-assert-deliver
	"If there is a DS, do a normal delivery, otherwise do a RoboCup 2021 delivery. "
	(?wp)

	(bind ?goal nil)
	(bind ?goal (goal-tree-assert-central-run-parallel DELIVER
		(assert (goal (class DELIVER)
			(id (sym-cat DELIVER- (gensym*))) (sub-type SIMPLE)
			(verbosity NOISY) (is-executable FALSE) (meta-template goal-meta)
			(params wp ?wp
					target-mps C-DS
					target-side INPUT)
		))
		(goal-production-assert-instruct-ds-deliver ?wp)
	))

	(return ?goal)
)

(deffunction goal-production-assert-instruct-cs-mount-cap
	(?mps ?cap-color)
	(bind ?goal (assert (goal (class INSTRUCT-CS-MOUNT-CAP)
	      (id (sym-cat INSTRUCT-CS-MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps
	              cap-color ?cap-color)
	)))
	(goal-meta-assert ?goal central)
	(return ?goal)
)

(defrule  do-c0-prof
	(not (do-c0))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))

	;base
	(wm-fact (key domain fact order-base-color args? ord ?order-id col ?base-color))

	;cap
	(wm-fact (key domain fact order-cap-color args? ord ?order-id col ?cap-color))
	(wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))
	=>
	(assert (do-c0))

	(bind ?wp-order (sym-cat wp- ?order-id))
	(assert (domain-object (name ?wp-order) (type workpiece))
			(domain-fact (name wp-unused) (param-values ?wp-order))
			(wm-fact (key domain fact wp-cap-color args? wp ?wp-order col CAP_NONE) (type BOOL) (value TRUE))
	)

	(bind ?goal
		(goal-tree-assert-central-run-parallel PRODUCE-ORDER
			(goal-tree-assert-central-run-parallel PREPARE-CS
				(goal-tree-assert-central-run-parallel BUFFER-GOALS
					(goal-meta-assert (goal-production-assert-buffer-cap ?cs ?cap-color) robot1)
					(goal-meta-assert (goal-production-assert-instruct-cs-buffer-cap ?cs ?cap-color) central)
					;(goal-production-assert-discard UNKNOWN ?cs OUTPUT)
					(goal-meta-assert (goal-production-assert-instruct-cs-mount-cap ?cs ?cap-color) robot1)
				)
			)
			;(goal-production-assert-deliver ?wp-order)
		)
	)
	(modify ?goal (meta (fact-slot-value ?goal meta) for-order ?order-id) (parent ?root-id))
)

(deffunction goal-production-assert-discard
	(?wp ?cs ?side)

	(bind ?goal (assert (goal (class DISCARD)
	      (id (sym-cat DISCARD- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp wp-loc ?cs wp-side ?side) (meta-template goal-meta)
	)))
	(return ?goal)
)


(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(wm-fact (key central agent robot args? r ?robot))
	(not (wm-fact (key domain fact entered-field args? r ?robot)))
	(not (and (goal (id ?some-goal-id) (class ENTER-FIELD))
	          (goal-meta (goal-id ?some-goal-id) (assigned-to ?robot))))
	(domain-facts-loaded)
	(wm-fact (key refbox team-color) (value ?team-color))
	=>
	(printout t "Goal " ENTER-FIELD " formulated" crlf)
	(goal-meta-assert (goal-production-assert-enter-field ?team-color) robot1)
)
