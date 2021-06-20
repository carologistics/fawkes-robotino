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
	=>
	(assert (goal (id (sym-cat SEND-BEACON- (gensym*))) (sub-type SIMPLE)
	              (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET)
	              (meta assigned-to central)
	              (is-executable TRUE)))
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
	              (meta assigned-to central)
	              (params mps ?mps) (is-executable TRUE)))
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

(defrule goal-production-flush-executability
" A waiting robot got a new goal, clear executability and robot assignment from other goals. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	(goal (sub-type SIMPLE) (mode SELECTED)
	      (meta $? assigned-to ?robot2 $?)
 	      (is-executable TRUE) (type ACHIEVE) (class ~SEND-BEACON))
	(goal (sub-type SIMPLE) (mode FORMULATED)
	      (meta $? assigned-to ?robot $?))
	=>
	(delayed-do-for-all-facts ((?g goal))
		(and (eq ?g:is-executable TRUE) (neq ?g:class SEND-BEACON))
		(modify ?g (is-executable FALSE))
	)
	(if (neq ?robot central)
		then
		(delayed-do-for-all-facts ((?g goal))
			(and (subsetp (create$ assigned-to ?robot) ?g:meta)
			     (eq ?g:mode FORMULATED) (not (eq ?g:type MAINTAIN)))
			(modify ?g (meta (remove-robot-assignment-from-goal ?g:meta ?robot)))
		)
		(do-for-fact ((?waiting wm-fact))
			(and (wm-key-prefix ?waiting:key (create$ central agent robot-waiting))
		         (eq (wm-key-arg ?waiting:key r) ?robot))
			(retract ?waiting)
		)
	)
)

(defrule goal-production-unassign-robot-from-finshed-goals
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	?g <- (goal (sub-type SIMPLE) (mode RETRACTED)
	      (meta $? assigned-to ?robot $?) (parent ?parent))
	(not (goal (id ?parent) (type MAINTAIN)))
	=>
	(modify ?g (meta (remove-robot-assignment-from-goal (fact-slot-value ?g meta) ?robot)))
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

(defrule goal-production-buffer-cap-executable
" Bring a cap-carrier from a cap stations shelf to the corresponding mps input
  to buffer its cap. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class BUFFER-CAP) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-mps ?mps
	                    cap-color ?cap-color
	            )
	            (meta $? assigned-to ?robot $?)
	            (is-executable FALSE))

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
	        (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	    )
	    (and
	        (wm-fact (key domain fact holding args? r ?robot wp ?cc))
	        (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	    )
	)
	=>
	(printout t "Goal BUFFER-CAP executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-mount-cap-executable
" Bring a product to a cap station to mount a cap on it.
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t CS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?target-mps side INPUT)))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))
	(wm-fact (key domain fact cs-buffered args? m ?target-mps col ?cap-color))
	(wm-fact (key domain fact cs-can-perform args? m ?target-mps op MOUNT_CAP))
	; WP CEs
	(wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and ; Either the workpiece needs to picked up...
	         (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	             ; ... and it is a fresh base located in a base station
	         (or (and (wm-fact (key domain fact mps-type args? m ?wp-loc t BS))
	                  (wm-fact (key domain fact wp-unused args? wp ?wp))
	                  (wm-fact (key domain fact wp-base-color
	                            args? wp ?wp col BASE_NONE)))
	             ; ... or is already at some machine
	             (wm-fact (key domain fact wp-at
	                       args? wp ?wp m ?wp-loc side ?wp-side))
	         )
	    )
	    ; or the workpiece is already being held
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	=>
	(printout t "Goal MOUNT-CAP executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)


(defrule goal-production-deliver-executable
" Bring a product to the delivery station.
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DELIVER)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t DS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?target-mps side INPUT)))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))
	; WP CEs
	(wm-fact (key wp meta next-step args? wp ?wp) (value DELIVER))
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	         (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side ?wp-side)))
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	=>
	(printout t "Goal DELIVER executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-discard-executable
" Bring a product to a cap station to mount a cap on it.
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DISCARD)
	                          (mode FORMULATED)
	                          (params  wp ?wp&~UNKNOWN wp-loc ?wp-loc wp-side ?wp-side)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

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

(defrule goal-production-instruct-cs-mount-cap-executable
" Instruct cap station to buffer a cap. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class INSTRUCT-CS-MOUNT-CAP) (sub-type SIMPLE)
	             (mode FORMULATED)
	            (params target-mps ?mps
	                    cap-color ?cap-color
	             )
	             (is-executable FALSE))

	(not (goal (class INSTRUCT-CS-MOUNT-CAP) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
	(wm-fact (key refbox team-color) (value ?team-color))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
	(wm-fact (key domain fact cs-buffered args? m ?mps col ?any-cap-color))
	; WP CEs
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
	(wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side OUTPUT)))
	=>
	(printout t "Goal INSTRUCT-CS-MOUNT-CAP executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-instruct-bs-dispense-base-executable
" Instruct base station to dispense a base. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class INSTRUCT-BS-DISPENSE-BASE)
	            (mode FORMULATED)
	            (params wp ?wp
	                    target-mps ?mps
	                    target-side ?side
	                    base-color ?base-color)
	            (meta $? assigned-to ?robot $?) (is-executable FALSE))

	(wm-fact (key refbox team-color) (value ?team-color))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t BS))
	(wm-fact (key domain fact mps-state args? m ?mps s IDLE))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	; WP CEs
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps $?)))
  (wm-fact (key domain fact wp-unused args? wp ?wp))
	; wait until a robot actually needs the base before proceeding
	(domain-atomic-precondition (operator wp-get) (goal-id ?g-id) (plan-id ?p-id)
	                            (predicate wp-at) (param-values ?wp ?mps ?side)
	                            (grounded TRUE) (is-satisfied FALSE))
	(plan-action (action-name wp-get) (param-values ? ?wp ?mps ?side)
	             (goal-id ?g-id) (plan-id ?p-id) (state PENDING))
	(not (goal (class INSTRUCT-BS-DISPENSE-BASE) (mode SELECTED|DISPATCHED|COMMITTED|EXPANDED)))
	=>
	(printout t "Goal INSTRUCT-BS-DISPENSE executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-instruct-ds-deliver-executable
" Instruct base station to dispense a base. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DELIVER)
	            (mode FORMULATED)
	            (params wp ?wp target-mps ?mps)
	            (meta $? assigned-to ?robot $?) (is-executable FALSE))

	(not (goal (class INSTRUCT-DS-DELIVER) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
	(wm-fact (key refbox team-color) (value ?team-color))
	(wm-fact (key domain fact mps-type args? m ?mps t DS))
	(wm-fact (key domain fact mps-state args? m ?mps s IDLE))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key refbox game-time) (values $?game-time))
	(wm-fact (key refbox order ?order delivery-begin) (type UINT)
	         (value ?begin&:(< ?begin (nth$ 1 ?game-time))))
	=>
	(printout t "Goal INSTRUCT-DS-DELIVER executable" crlf)
	(modify ?g (is-executable TRUE))
)

(deffunction goal-production-assert-buffer-cap
	(?mps ?cap-color)

	(bind ?goal (assert (goal (class BUFFER-CAP)
	      (id (sym-cat BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps
	              cap-color ?cap-color)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-mount-cap
	(?wp ?mps ?wp-loc ?wp-side)

	(bind ?goal (assert (goal (class MOUNT-CAP)
	      (id (sym-cat MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
 	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp
	              target-mps ?mps
	              target-side INPUT
	              wp-loc ?wp-loc
	              wp-side ?wp-side)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-discard
	(?wp ?cs ?side)

	(bind ?goal (assert (goal (class DISCARD)
	      (id (sym-cat DISCARD- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp wp-loc ?cs wp-side ?side)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-deliver
	(?wp)

	(bind ?goal (assert (goal (class DELIVER)
	      (id (sym-cat DELIVER- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp
	              target-mps C-DS
	              target-side INPUT)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-instruct-cs-buffer-cap
 	(?mps ?cap-color)

 	(bind ?goal (assert (goal (class INSTRUCT-CS-BUFFER-CAP)
	      (id (sym-cat INSTRUCT-CS-BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta assigned-to central)
	      (params target-mps ?mps
	              cap-color ?cap-color)
 	)))
	(return ?goal)
)

(deffunction goal-production-assert-instruct-bs-dispense-base
	(?wp ?base-color ?side)

	(bind ?goal (assert (goal (class INSTRUCT-BS-DISPENSE-BASE)
          (id (sym-cat INSTRUCT-BS-DISPENSE-BASE- (gensym*))) (sub-type SIMPLE)
          (verbosity NOISY) (is-executable FALSE) (meta assigned-to central)
	      (params wp ?wp
	              target-mps C-BS
	              target-side ?side
	              base-color ?base-color)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-instruct-ds-deliver
	(?wp)

	(bind ?goal (assert (goal (class INSTRUCT-DS-DELIVER)
          (id (sym-cat INSTRUCT-DS-DELIVER- (gensym*))) (sub-type SIMPLE)
          (verbosity NOISY) (is-executable FALSE) (meta assigned-to central)
	      (params wp ?wp
	              target-mps C-DS)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-instruct-cs-mount-cap
 	(?mps ?cap-color)

	(bind ?goal (assert (goal (class INSTRUCT-CS-MOUNT-CAP)
	      (id (sym-cat INSTRUCT-CS-MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta assigned-to central)
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
	)))
	(return ?goal)
)

(deffunction goal-production-assert-c0
  (?root-id ?order-id ?wp-for-order ?cs ?cap-col ?base-col)

  (bind ?goal
    (goal-tree-assert-central-run-parallel PRODUCE-ORDER
		(goal-tree-assert-central-run-parallel PREPARE-CS
			(goal-tree-assert-central-run-parallel BUFFER-GOALS
				(goal-production-assert-buffer-cap ?cs ?cap-col)
				(goal-production-assert-instruct-cs-buffer-cap ?cs ?cap-col)
				(goal-production-assert-discard UNKNOWN ?cs OUTPUT)
			)
		)
		(goal-tree-assert-central-run-parallel MOUNT-GOALS
			(goal-tree-assert-central-run-one INTERACT-BS
				(goal-tree-assert-central-run-parallel OUTPUT-BS
					(goal-production-assert-mount-cap ?wp-for-order ?cs C-BS OUTPUT)
					(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?base-col OUTPUT)
				)
				(goal-tree-assert-central-run-parallel INPUT-BS
					(goal-production-assert-mount-cap ?wp-for-order ?cs C-BS INPUT)
					(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?base-col INPUT)
				)
			)
			(goal-production-assert-instruct-cs-mount-cap ?cs ?cap-col)
		)
		(goal-production-assert-deliver ?wp-for-order)
		(goal-production-assert-instruct-ds-deliver ?wp-for-order)
	)
  )
  (modify ?goal (meta (fact-slot-value ?goal meta) for-order ?order-id) (parent ?root-id))
)

(defrule goal-production-create-root
	"Create the production root under which all production trees for the orders
	are asserted"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(domain-facts-loaded)
	(not (goal (class PRODUCTION-ROOT)))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key game state) (value RUNNING))
	(wm-fact (key refbox team-color) (value ?color))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel PRODUCTION-ROOT))
	(modify ?g (meta do-not-finish))
)

(defrule goal-production-create-produce-for-order
	"Create for each incoming order a grounded production tree with the"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(wm-fact (key domain fact order-complexity args? ord ?order-id&:(eq ?order-id O1) com ?comp))
	(wm-fact (key domain fact order-base-color args? ord ?order-id col ?col-base))
	(wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?col-cap))
	(wm-fact (key domain fact cs-color args? m ?cs col ?col-cap))
	(wm-fact (key domain fact mps-type args? m ?cs t CS))
	(not (wm-fact (key order meta wp-for-order args? wp ?something ord O1)))
	=>
	(bind ?wp-for-order (sym-cat wp- ?order-id))
	(assert (domain-object (name ?wp-for-order) (type workpiece))
  		  (domain-fact (name wp-unused) (param-values ?wp-for-order))
		  (wm-fact (key domain fact wp-base-color args? wp ?wp-for-order col BASE_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-cap-color args? wp ?wp-for-order col CAP_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring1-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring2-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring3-color args? wp ?wp-for-order col RING_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key order meta wp-for-order args? wp ?wp-for-order ord ?order-id))
	)
	(if (eq ?comp C0)
		then
		(goal-production-assert-c0 ?root-id ?order-id ?wp-for-order ?cs ?col-cap ?col-base)
	)
)

(defrule goal-production-fill-in-unknown-wp-discard
	"Fill in missing workpiece information into the discard goals"
	?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED) (parent ?parent)
	            (params wp UNKNOWN wp-loc ?mps wp-side ?mps-side)
	            (meta $? assigned-to ?robot $?))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side))
	(goal (parent ?parent) (class INSTRUCT-CS-BUFFER-CAP) (mode DISPATCHED|FINISHED|RETRACTED))
	=>
	(modify ?g (params wp ?wp wp-loc ?mps wp-side ?mps-side))
)



;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;
; NAVIGATION CHALLENGE ;
;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;

(defrule goal-production-navigation-challenge-move-executable
" Move to a navgraph node
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class NAVIGATION-CHALLENGE-MOVE)
	                          (mode FORMULATED)
	                          (params target ?target)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))
	=>
	(printout t "Goal NAVIGATION-CHALLENGE-MOVE executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(deffunction goal-production-assert-navigation-challenge-move
	(?location)

	(bind ?goal (assert (goal (class NAVIGATION-CHALLENGE-MOVE)
					(id (sym-cat NAVIGATION-CHALLENGE-MOVE- (gensym*)))
					(sub-type SIMPLE)
					(verbosity NOISY) (is-executable FALSE)
					(params target (translate-location-map-to-grid ?location))
				)))
	(return ?goal)
)

(deffunction goal-production-assert-navigation-challenge
  (?root-id ?locations)

  (bind ?goals (create$))
  (foreach ?location ?locations
	(bind ?goals (insert$ ?goals (+ 1 (length$ ?goal))
				 (goal-production-assert-navigation-challenge-move ?location)))
  )

  (bind ?goal
    (goal-tree-assert-central-run-parallel NAVIGATION-CHALLENGE-PARENT
		?goals
	)
  )
  (modify ?goal (parent ?root-id))
)

(defrule goal-production-create-navigation-challenge-tree
  "Create a goal tree for the navigation challenge if there is a waypoint fact."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
  (wm-fact (key domain fact waypoints args? $?waypoints))
  (not (goal (class NAVIGATION-CHALLENGE-PARENT)))
  =>
  (goal-production-assert-navigation-challenge ?root-id ?waypoints)
)
