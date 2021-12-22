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

(defrule goal-production-fill-in-unknown-wp-discard
	"Fill in missing workpiece information into the discard goals"
	?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED) (parent ?parent)
	            (params wp UNKNOWN wp-loc ?mps wp-side ?mps-side))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side))
	(not (wm-fact (key order meta wp-for-order args? wp ?wp $?)))
	=>
	(printout t "Unknown workpiece filled in to " ?wp crlf)
	(modify ?g (params wp ?wp wp-loc ?mps wp-side ?mps-side))
)

(defrule goal-production-discard-executable
" Discard output from a station.
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DISCARD)
	                          (mode FORMULATED)
	                          (params  wp ?wp&~UNKNOWN wp-loc ?wp-loc wp-side ?wp-side)
	                          (is-executable FALSE))
	(not (goal (class DISCARD) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	         (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side ?wp-side)))
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	=>
	(printout t "Goal DISCARD executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-mount-cap-base-executable
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP-BASE) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params wp ?wp base-mps ?base-mps base-color ?base-color cap-mps ?cap-mps cap-color ?cap-color)
	            (is-executable FALSE))
	(not (goal (class MOUNT-CAP-BASE) (mode SELECTED|DISPATCHED|COMMITTED|EXPANDED)))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key refbox team-color) (value ?team-color))
	; base MPS CEs
	(wm-fact (key domain fact mps-type args? m ?base-mps t BS))
	(wm-fact (key domain fact mps-state args? m ?base-mps s IDLE))
	(wm-fact (key domain fact mps-team args? m ?base-mps col ?team-color))
	(not (wm-fact (key domain fact wp-at args? wp ?any-base-wp m ?base-mps $?)))
	; cap MPS CEs
	(wm-fact (key domain fact mps-type args? m ?cap-mps t CS))
	(wm-fact (key domain fact mps-state args? m ?cap-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?cap-mps col ?team-color))
	(not (wm-fact (key domain fact wp-at args? wp ?any-cap-in-wp m ?cap-mps side INPUT)))
	(not (wm-fact (key domain fact wp-at args? wp ?any-cap-o-wp m ?cap-mps side OUTPUT)))
	(wm-fact (key domain fact cs-buffered args? m ?cap-mps col ?cap-color))
	(wm-fact (key domain fact cs-can-perform args? m ?cap-mps op MOUNT_CAP))
	; WP CEs
	(wm-fact (key domain fact wp-unused args? wp ?wp))
	=>
	(printout t "Goal MOUNT-CAP-BASE executable" crlf)
	(modify ?g (is-executable TRUE))
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


(defrule goal-production-deliver-executable
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DELIVER)
	            (mode FORMULATED)
	            (params wp ?wp mps ?mps)
	            (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(not (goal (class DELIVER) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
	(wm-fact (key refbox team-color) (value ?team-color))
	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	; MPS-CES
	(wm-fact (key domain fact mps-type args? m ?mps t DS))
	(wm-fact (key domain fact mps-state args? m ?mps s IDLE))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side INPUT)))
	; WP-CES
	(wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps side INPUT))
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	; Game time
	(wm-fact (key refbox game-time) (values $?game-time))
	(wm-fact (key refbox order ?order delivery-begin) (type UINT)
	         (value ?begin&:(< ?begin (nth$ 1 ?game-time))))
	=>
	(printout t "Goal DELIVER executable" crlf)
	(modify ?g (is-executable TRUE))
)

; ----------------------- Goal creation functions -------------------------------

(deffunction goal-production-assert-discard
	(?wp ?cs ?side)

	(bind ?goal (assert (goal (class DISCARD)
	      (id (sym-cat DISCARD- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp wp-loc ?cs wp-side ?side) (meta-template goal-meta)
	)))
	(return ?goal)
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

(deffunction goal-production-assert-mount-cap-base
	(?wp ?base-mps ?base-color ?cap-mps ?cap-color)

	(bind ?goal (assert (goal (class MOUNT-CAP-BASE)
	      (id (sym-cat MOUNT-CAP-BASE- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta-template goal-meta)
	      (params wp ?wp base-mps ?base-mps base-color ?base-color cap-mps ?cap-mps cap-color ?cap-color)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-deliver
	(?wp ?mps)

	(bind ?goal (assert (goal (class DELIVER)
	      (id (sym-cat DELIVER- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp mps ?mps) (meta-template goal-meta)
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

(defrule goal-production-c0-order
	"Create the goals for a C0 order."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(wm-fact (key domain fact order-complexity args? ord ?order-id com C0))
	(wm-fact (key domain fact order-base-color args? ord ?order-id col ?base-color))
	(wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?cap-color))
	=>
	(bind ?wp-for-order (sym-cat wp- ?order-id))
	(assert (domain-object (name ?wp-for-order) (type workpiece))
		  (domain-fact (name wp-unused) (param-values ?wp-for-order))
		  (wm-fact (key domain fact wp-base-color args? wp ?wp-for-order col BASE_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-cap-color args? wp ?wp-for-order col CAP_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring1-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring2-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring3-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key order meta wp-for-order args? wp ?wp-for-order ord O1))
	)
	(bind ?goal (goal-tree-assert-central-run-parallel PRODUCE-ORDER
		(goal-meta-assert (goal-production-assert-buffer-cap C-CS1 ?cap-color) robot1)
		(goal-meta-assert (goal-production-assert-discard UNKNOWN C-CS1 OUTPUT) robot1)
		(goal-meta-assert (goal-production-assert-mount-cap-base ?wp-for-order C-BS ?base-color C-CS1 ?cap-color) robot1)
		(goal-meta-assert (goal-production-assert-deliver ?wp-for-order C-DS) robot1)
		)
	)
  	(modify ?goal (meta (fact-slot-value ?goal meta) for-order ?order-id) (parent ?root-id))
	(printout t "Goal PRODUCE-ORDER formulated for " ?order-id crlf)
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
	=>
	(bind ?g (goal-tree-assert-central-run-parallel PRODUCTION-ROOT))
	(modify ?g (meta do-not-finish) (priority 1.0))
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
