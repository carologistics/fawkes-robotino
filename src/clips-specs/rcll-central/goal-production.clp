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
	(printout error ?pos " " ?pos2 crlf)
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
	              (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET))
	              (meta assign-to central)
	              (is-executable TRUE))
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
	              (meta assign-to central)
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
	(not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
	(not (wm-fact (key domain fact cs-buffered args? m ?mps col ?any-cap-color)))
	(not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side INPUT)))
	; Capcarrier CEs
	(wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))
	(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
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
	                                   wp-loc ?wp-loc
	                                   wp-side ?wp-side
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
	             ; ... and will be dispensed at the BS soon (is already SELECTED
	             ; and just waits until a robot actually needs a workpiece
	         (or (goal (class INSTRUCT-BS-DISPENSE-BASE)
	                   (params wp ?wp target-mps ?wp-loc $?)
	                   (mode SELECTED))
	             ; ... or is already at some machine
	             (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side ?wp-side))
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
	                                   wp-loc ?wp-loc
	                                   wp-side ?wp-side
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
	                          (params  wp ?wp
	                                   wp-loc ?wp-loc
	                                   wp-side ?wp-side)
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

(defrule goal-production-test-C0
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	(not (wm-fact (key order meta wp-for-order args? wp WP-TEST ord O1)))
  (wm-fact (key domain fact order-base-color args? ord O1 col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord O1 col ?cap-color))
	(wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps $?))
	(wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
=>
	(assert
	  (domain-object (name WP-TEST) (type workpiece))
	  (domain-fact (name wp-unused) (param-values WP-TEST))
	  (domain-fact (name wp-base-color) (param-values WP-TEST BASE_NONE))
	  (domain-fact (name wp-ring1-color) (param-values WP-TEST RING_NONE))
	  (domain-fact (name wp-ring2-color) (param-values WP-TEST RING_NONE))
	  (domain-fact (name wp-ring3-color) (param-values WP-TEST RING_NONE))
	  (domain-fact (name wp-cap-color) (param-values WP-TEST CAP_NONE))
	  (wm-fact (key order meta wp-for-order args? wp WP-TEST ord O1))
	  (goal (id BUFFER-CAP1) (class BUFFER-CAP) (sub-type SIMPLE)
	        (verbosity NOISY)
	        (params target-mps ?mps
	                cap-color ?cap-color))
	  (goal (id MOUNT-CAP1) (class MOUNT-CAP) (sub-type SIMPLE)
	        (params wp WP-TEST
	                wp-loc C-BS
	                wp-side INPUT
	                target-mps ?mps
	                target-side INPUT))
	  (goal (id INSTRUCT-CS-BUFFER-CAP1) (class INSTRUCT-CS-BUFFER-CAP) (sub-type SIMPLE)
	        (verbosity NOISY)
	        (params target-mps ?mps cap-color ?cap-color)
	        (meta assigned-to central))
	  (goal (id INSTRUCT-DS-DELIVER1) (class INSTRUCT-DS-DELIVER) (sub-type SIMPLE)
	        (verbosity NOISY)
	        (params wp WP-TEST target-mps C-DS) (meta assigned-to central))
	  (goal (id INSTRUCT-CS-MOUNT-CAP1) (class INSTRUCT-CS-MOUNT-CAP) (sub-type SIMPLE)
	        (verbosity NOISY)
	        (params target-mps ?mps cap-color ?cap-color)
	        (meta assigned-to central))
	  (goal (id INSTRUCT-BS-DISPENSE-BASE1) (class INSTRUCT-BS-DISPENSE-BASE) (sub-type SIMPLE)
	        (verbosity NOISY)
	        (params  wp WP-TEST
	                 target-mps C-BS
	                 target-side INPUT
	                 base-color ?base-color)
	        (meta assigned-to central))
	  (goal (id DELIVER1) (class DELIVER) (sub-type SIMPLE)
	        (verbosity NOISY)
	        (params  wp WP-TEST
	                 wp-loc ?mps
	                 wp-side OUTPUT
	                 target-mps C-DS
	                 target-side INPUT))
	  (goal (id DISCARD1) (class DISCARD) (sub-type SIMPLE)
	        (verbosity NOISY)
	        (params wp ?wp wp-loc ?mps wp-side OUTPUT))
	)
)
