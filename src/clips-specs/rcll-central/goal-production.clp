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

(deffunction goal-production-get-machine-for-color
	(?col-ring)

	(bind ?rs FALSE)
	(do-for-all-facts ((?mps-type wm-fact)) (and (wm-key-prefix ?mps-type:key (create$ domain fact mps-type))
	                                             (eq (wm-key-arg ?mps-type:key t) RS))
		(bind ?machine (wm-key-arg ?mps-type:key m))
		(do-for-fact ((?rs-ring-spec wm-fact)) (and (wm-key-prefix ?rs-ring-spec:key (create$ domain fact rs-ring-spec))
		                                            (eq (wm-key-arg ?rs-ring-spec:key m) ?machine)
		                                            (eq (wm-key-arg ?rs-ring-spec:key r) ?col-ring))
			(bind ?rs ?machine)
		)
	)
	(return ?rs)
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
		(modify ?g (meta (assign-robot-to-goal ?g:meta ?robot)))
	)
)

(defrule goal-production-flush-executability
" A waiting robot got a new goal, clear executability and robot assignment from other goals. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	(goal (sub-type SIMPLE) (mode SELECTED)
	      (meta $? assigned-to ?robot $?)
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
	; cleaning goal dependencies by flushing grounded-with for formulated goals
	(delayed-do-for-all-facts ((?da dependency-assignment) (?g goal))
		(and (eq ?da:goal-id ?g:id) (neq ?da:grounded-with nil) (eq ?g:mode FORMULATED))
		(modify ?da (grounded-with nil))
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

(defrule goal-production-move-out-of-way-executable
" Moves an unproductive robot to the given position "
	(declare (salience (- ?*SALIENCE-GOAL-EXECUTABLE-CHECK* 1)))
	?g <- (goal (class MOVE-OUT-OF-WAY) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-pos ?target-pos location ?loc)
	            (meta $? assigned-to ?robot $?)
	            (is-executable FALSE))
	; is executable if no other goal exists which is executable for the robot
	; beside a goal of the same class
	(not (goal (meta $? assigned-to ?robot $?) (is-executable TRUE) (class ~MOVE-OUT-OF-WAY)))
	;    (goal (class MOVE-OUT-OF-WAY)))
	; check if target position is free
	(test (is-free ?target-pos))
	=>
	(printout t "Goal MOVE-OUT-OF-WAY executable for " ?robot " to pos " ?target-pos  crlf)
  (modify ?g (is-executable TRUE))
)

(defrule goal-production-pick-and-place-executable
"Check executability for pick and place
 Picks a wp from the output of the given mps
  and feeds it into the input of the same mps"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class PICK-AND-PLACE) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-mps ?mps )
	            (meta $? assigned-to ?robot $?)
	            (is-executable FALSE))
	(wm-fact (key domain fact mps-side-free args? m ?mps side INPUT))
	(or (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	(wm-fact (key domain fact maps args? m ?mps r ?robot))
	=>
	(printout t "Goal PICK-AND-PLACE executable for " ?robot " at " ?mps crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-move-robot-to-output-executable
"Check executability to move to the output of the given mps. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class MOVE) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-mps ?mps )
	            (meta $? assigned-to ?robot $?)
	            (is-executable FALSE))
	(wm-fact (key domain fact maps args? m ?mps r ?robot))
	=>
	(printout t "Goal MOVE executable for " ?robot " at " ?mps crlf)
	(modify ?g (is-executable TRUE))
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

(defrule goal-production-deliver-rc21-executable
" Bring a product to the insertion zone and drop it."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DELIVER-RC21)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))
	; WP CEs
	(wm-fact (key wp meta next-step args? wp ?wp) (value DELIVER))
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	         (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side ?wp-side)))
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	=>
	(printout t "Goal DELIVER-RC212 executable for " ?robot crlf)
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


(defrule goal-production-get-base-to-fill-rs-executable
"Fill the ring station with a fresh base from the base station."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class PAY-FOR-RINGS-WITH-BASE)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   wp-loc ?wp-loc
	                                   wp-side ?wp-side
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))
	(wm-fact (key refbox team-color) (value ?team-color))
	;MPS-RS CEs (a cap carrier can be used to fill a RS later)
	(wm-fact (key domain fact mps-type args? m ?target-mps t RS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))
	;check ring payment - prevention of overfilling rs
	(wm-fact (key domain fact rs-filled-with args? m ?target-mps n ?rs-before&ZERO|ONE|TWO))
	;check that not to many robots try to fill the rs at the same time
	(or (not (goal (class PAY-FOR-RINGS-WITH-BASE| PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                      PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	               (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	               (params $? target-mps ?target-mps $?)))
	    (and (goal (class PAY-FOR-RINGS-WITH-BASE| PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                      PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	               (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	               (params $? target-mps ?target-mps $?))
	         (test (< (+ (length$ (find-all-facts ((?other-goal goal))
	                         (and (or (eq ?other-goal:class PAY-FOR-RINGS-WITH-BASE)
	                                  (eq ?other-goal:class PAY-FOR-RINGS-WITH-CAP-CARRIER)
	                                  (eq ?other-goal:class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF))
	                              (is-goal-running ?other-goal:mode)
	                              (member$ ?target-mps ?other-goal:params))))
	                     (sym-to-int ?rs-before)) 3))
	   )
	)
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and ; Either the workpiece needs to picked up...
	         (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	             ; ... and it is a fresh base located in a base station
	         (or (and (wm-fact (key domain fact mps-type args? m ?wp-loc t BS))
	                  (wm-fact (key domain fact mps-state args? m ?wp-loc s ~BROKEN))
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
	(printout t "Goal " PAY-FOR-RINGS-WITH-BASE " executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-get-cap-carrier-to-fill-rs-executable
"Fill the ring station with a cap carrier located at the output of a cap station."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
	                          (mode FORMULATED)
	                          (params  wp ?preset-wp
	                                   wp-loc ?wp-loc
	                                   wp-side UNKNOWN
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	(wm-fact (key refbox team-color) (value ?team-color))
	;MPS-RS CEs (a cap carrier can be used to fill a RS later)
	(wm-fact (key domain fact mps-type args? m ?target-mps t RS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))
	;check ring payment - prevention of overfilling rs
	(wm-fact (key domain fact rs-filled-with args? m ?target-mps n ?rs-before&ZERO|ONE|TWO))
	;check that not to may robots try to fill the rs at the same time
	(or (not (goal (class PAY-FOR-RINGS-WITH-BASE| PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                      PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	               (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	               (params $? target-mps ?target-mps $?)))
	    (and (goal (class PAY-FOR-RINGS-WITH-BASE|PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                      PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	               (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	               (params $? target-mps ?target-mps $?))
	         (test (< (+ (length$ (find-all-facts ((?other-goal goal))
	                               (and (or (eq ?other-goal:class PAY-FOR-RINGS-WITH-BASE)
	                                        (eq ?other-goal:class PAY-FOR-RINGS-WITH-CAP-CARRIER)
	                                        (eq ?other-goal:class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF))
	                                    (is-goal-running ?other-goal:mode)
	                                    (member$ ?target-mps ?other-goal:params)
	                               )))
	                     (sym-to-int ?rs-before)) 3))
	   )
	)
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t CS))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	;check wp has no cap and is at the output of the CS
	(wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
	(or (and ; Either the workpiece needs to picked up...
	         (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	             ; ... and it is a fresh base located in a base station
	         (or (and (wm-fact (key domain fact mps-type args? m ?wp-loc t CS))
	                  (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side OUTPUT))
	                  (wm-fact (key domain fact wp-unused args? wp ?wp))
	                  (wm-fact (key domain fact wp-base-color
	                            args? wp ?wp col BASE_NONE)))
	             ; ... or is already at some machine
	             (wm-fact (key domain fact wp-at
	                       args? wp ?wp m ?wp-loc side ?wp-side))
	         )
	    )
	    ; or the workpiece is already being held
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp&:(eq ?wp ?preset-wp))))
	=>
	(bind ?wp-side nil)
	(do-for-fact ((?wp-at wm-fact))
	             (and (wm-key-prefix ?wp-at:key (create$ domain fact wp-at))
	                  (eq (wm-key-arg ?wp-at:key wp) ?wp))
	             (bind ?wp-side (wm-key-arg ?wp-at:key side))
	)
	(printout t "Goal "  PAY-FOR-RINGS-WITH-CAP-CARRIER " executable" crlf)
	(modify ?g (is-executable TRUE)(params wp ?wp
	                                       wp-loc ?wp-loc
	                                       wp-side ?wp-side
	                                       target-mps ?target-mps
	                                       target-side ?target-side)
	)
)

(defrule goal-production-pay-ring-with-carrier-from-shelf-executable
  "Get a capcarrier from a shelf to feed an rs with it later."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	                          (mode FORMULATED)
	                          (params  wp-loc ?wp-loc
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	(wm-fact (key refbox team-color) (value ?team-color))
	;MPS-RS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t RS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))
	;check ring payment - prevention of overfilling rs
	(wm-fact (key domain fact rs-filled-with args? m ?target-mps n ?rs-before&ZERO|ONE|TWO))
	;check that not to may robots try to fill the rs at the same time
	(or (not (goal (class PAY-FOR-RINGS-WITH-BASE|PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                      PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	               (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	               (params $? target-mps ?target-mps $?)))
	    (and (goal (class PAY-FOR-RINGS-WITH-BASE|PAY-FOR-RINGS-WITH-CAP-CARRIER|
	                      PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	               (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	               (params $? target-mps ?target-mps $?))
	         (test (< (+ (length$ (find-all-facts ((?other-goal goal))
	                               (and (or (eq ?other-goal:class PAY-FOR-RINGS-WITH-BASE)
	                                        (eq ?other-goal:class PAY-FOR-RINGS-WITH-CAP-CARRIER)
	                                        (eq ?other-goal:class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF))
	                              (is-goal-running ?other-goal:mode)
	                          (member$ ?target-mps ?other-goal:params))))
	                     (sym-to-int ?rs-before)) 3))
	   )
	)
	;MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t CS))
	(wm-fact (key domain fact mps-state args? m ?wp-loc s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	;either there is a wp on the shelf and we don't hold any or we hold one and it is
	;a CC (e.g. if the goal fails after pick-up)
	(or (and (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?wp-loc spot ?spot))
		     (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
		     (not (plan-action (action-name wp-get-shelf) (param-values $? ?wp $?)))
		)
		(and (domain-object (name ?wp) (type cap-carrier))
		     (wm-fact (key domain fact holding args? r ?robot wp ?wp))
		)
	)

	; Formulate the goal only if it is not already formulated (prevents doubling
	; the goals due to matching with RS-1 and RS-2)
	(not (goal (class  PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF) (parent goal-id)
	     (params robot ?robot cs ?wp-loc wp ?wp $?)))
	=>
	(printout t "Goal " PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF " executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-mount-ring-executable
" Bring a product to a ring station to mount a ring on it.
The workpiece remains in the output of the used ring station after
  successfully finishing this goal.
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-RING)
	                          (mode FORMULATED)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   wp-loc ?wp-loc
	                                   wp-side ?wp-side
	                                   $?)
	                          (meta $? assigned-to ?robot $?)
	                          (is-executable FALSE))

	; Robot CEs
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-RS CEs
	(wm-fact (key domain fact mps-type args?       m ?target-mps t RS))
	(wm-fact (key domain fact mps-state args?      m ?target-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args?       m ?target-mps col ?team-color))
	(wm-fact (key domain fact rs-filled-with args? m ?target-mps n ?bases-filled))
	; WP CEs
	(wm-fact (key wp meta next-step args? wp ?wp) (value ?ring))
	(wm-fact (key domain fact ?wp-ring-color&:(eq ?wp-ring-color
	         (sym-cat wp-ring (sub-string 5 5 ?ring) -color))
	          args? wp ?wp col RING_NONE ))
	; Order CEs
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key domain fact ?order-ring-color&:(eq ?order-ring-color
	         (sym-cat order-ring (sub-string 5 5 ?ring) -color))
	          args? ord ?order col ?ring-color ))
	(wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&C1|C2|C3))

	(not (wm-fact (key domain fact wp-at args? wp ?wp-loc m ?target-mps side INPUT)))
	(wm-fact (key domain fact mps-type args? m ?other-rs&~?target-mps t RS))
	(wm-fact (key domain fact mps-team args? m ?other-rs col ?team-color))
	; There is at least one other rs side, except for the target input, that
	; is free (because occupying all 4 sides at once can cause deadlocks)
	(or (wm-fact (key domain fact mps-side-free args? m ?target-mps side OUTPUT))
	 (wm-fact (key domain fact mps-side-free args? m ?other-rs side ?any-side)))

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
	;If not ring Kosten bezahlt
	; (plan-assert-action fill-rs )
	(printout t "Goal MOUNT-RING executable for " ?robot crlf)
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


(defrule goal-production-instruct-rs-mount-ring-executable
" Instruct ring station to mount a ring on the product. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class INSTRUCT-RS-MOUNT-RING) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-mps ?mps
	                    ring-color ?ring-color
	             )
	             (is-executable FALSE))

	(wm-fact (key refbox team-color) (value ?team-color))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t RS))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(wm-fact (key domain fact rs-filled-with args? m ?mps n ?bases-filled))
	; Ring Cost
	(wm-fact (key domain fact rs-ring-spec
            args? m ?mps r ?ring-color&~RING_NONE rn ?bases-needed))
	(wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))

	; WP CEs
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
	(wm-fact (key wp meta next-step args? wp ?wp) (value ?ring))
	(wm-fact (key domain fact ?wp-ring-color&:(eq ?wp-ring-color
	         (sym-cat wp-ring (sub-string 5 5 ?ring) -color))
	          args? wp ?wp col RING_NONE ))
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key domain fact ?order-ring-color&:(eq ?order-ring-color
	         (sym-cat order-ring (sub-string 5 5 ?ring) -color))
	          args? ord ?order col ?ring-color ))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side OUTPUT)))
	(not (goal (class INSTRUCT-RS-MOUNT-RING) (mode EXPANDED|SELECTED|DISPATCHED|COMMITTED)))
	=>
	(printout t "Goal INSTRUCT-RS-MOUNT-RING executable" crlf)
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

(deffunction goal-production-assert-pick-and-place
	(?mps ?robot)
	(bind ?goal (assert (goal (class PICK-AND-PLACE)
	      (id (sym-cat PICK-AND-PLACE- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps)
	      (meta assigned-to ?robot)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-move-robot-to-output
	(?mps ?robot)
	(bind ?goal (assert (goal (class MOVE)
	      (id (sym-cat MOVE- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps)
	      (meta assigned-to ?robot)
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

(deffunction goal-production-assert-mount-ring
	(?wp ?rs ?wp-loc ?wp-side)
	(bind ?goal (assert (goal (class MOUNT-RING)
	      (id (sym-cat MOUNT-RING- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params  wp ?wp
	               target-mps ?rs
	               target-side INPUT
	               wp-loc ?wp-loc
	               wp-side ?wp-side
	               )
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

(deffunction goal-production-assert-pay-for-rings-with-base
	(?wp ?wp-loc ?wp-side ?target-mps ?target-side)
	(bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-BASE)
	      (id (sym-cat PAY-FOR-RINGS-WITH-BASE- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params  wp ?wp
	               wp-loc ?wp-loc
	               wp-side ?wp-side
	               target-mps ?target-mps
	               target-side ?target-side
	               )
	)))
	(return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-cap-carrier
	(?wp ?wp-loc ?wp-side ?target-mps ?target-side)

	(bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
	      (id (sym-cat PAY-FOR-RINGS-WITH-CAP-CARRIER- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params  wp ?wp
	               wp-loc ?wp-loc
	               wp-side ?wp-side
	               target-mps ?target-mps
	               target-side ?target-side
	               )
	)))
)

(deffunction goal-production-assert-deliver-rc21
	(?wp)

	(bind ?goal (assert (goal (class DELIVER-RC21)
	      (id (sym-cat DELIVER-RC21- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-cap-carrier-from-shelf
	(?wp-loc ?target-mps ?target-side)

	(bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	      (id (sym-cat PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params  wp-loc ?wp-loc
	               target-mps ?target-mps
	               target-side ?target-side
	               )
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

(deffunction goal-production-assert-instruct-rs-mount-ring
	(?mps ?col-ring)
	(bind ?goal (assert (goal (class INSTRUCT-RS-MOUNT-RING)
	      (id (sym-cat INSTRUCT-RS-MOUNT-RING- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta assigned-to central)
	            (params target-mps ?mps
	                    ring-color ?col-ring
	             )
	)))
	(return ?goal)
)

(deffunction goal-production-assert-payment-goals
	(?rs ?cols-ring)
	(bind ?goals (create$))

	(bind ?found-payment FALSE)
	(bind ?index 1)
	(bind ?first-rs nil)
	(loop-for-count (length$ ?rs)
		(bind ?price 0)
		(do-for-fact ((?rs-ring-spec wm-fact))
			(and (wm-key-prefix ?rs-ring-spec:key (create$ domain fact rs-ring-spec))
					(eq (wm-key-arg ?rs-ring-spec:key r ) (nth$ ?index ?cols-ring))
			)
			(bind ?num (wm-key-arg ?rs-ring-spec:key rn))
			(if (eq ?num ONE) then
				(bind ?price 1))
			(if (eq ?num TWO) then
				(bind ?price 2))
		)
		(if (and (not ?found-payment) (> ?price 0)) then
			(bind ?found-payment TRUE)
			(bind ?price (- ?price 1))
			(bind ?first-rs (nth$ ?index ?rs))
		)

		(loop-for-count ?price
			(bind ?wp-base-pay (sym-cat BASE-PAY- (gensym*)))
			(assert (domain-object (name ?wp-base-pay) (type workpiece))
					(domain-fact (name wp-unused) (param-values ?wp-base-pay))
					(wm-fact (key domain fact wp-base-color args? wp ?wp-base-pay col BASE_NONE)
						(type BOOL) (value TRUE))
			)
			(bind ?goals
				(insert$ ?goals (+ (length$ ?goals) 1)
					(goal-tree-assert-central-run-one PAY-FOR-RING-GOAL
						(goal-tree-assert-central-run-parallel INPUT-BS
							(goal-production-assert-pay-for-rings-with-base ?wp-base-pay C-BS INPUT (nth$ ?index ?rs) INPUT)
							(goal-production-assert-instruct-bs-dispense-base ?wp-base-pay BASE_RED INPUT)
						)
						(goal-production-assert-pay-for-rings-with-cap-carrier-from-shelf C-CS1 (nth$ ?index ?rs) INPUT)
					)
				)
			)
	 	)
		(bind ?index (+ ?index 1))
	)
	(if (eq ?found-payment TRUE) then
		(bind ?goals (insert$ ?goals 1 (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN C-CS1 UNKNOWN ?first-rs INPUT)))
	)

	(return ?goals)
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

(deffunction goal-production-assert-move-out-of-way
	(?location)
	(bind ?goal (assert (goal (class MOVE-OUT-OF-WAY)
	            (id (sym-cat MOVE-OUT-OF-WAY- (gensym*)))
	            (sub-type SIMPLE)
	            (verbosity NOISY) (is-executable FALSE)
	            (params target-pos (translate-location-map-to-grid ?location) location ?location)
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
		;deliver-rc21 is a change specific to robocup 2021, for the normal game keep
		;the old structure
		;(goal-production-assert-deliver ?wp-for-order)
		;(goal-production-assert-instruct-ds-deliver ?wp-for-order)
		(goal-production-assert-deliver-rc21 ?wp-for-order)
	)
  )
  (modify ?goal (meta (fact-slot-value ?goal meta) for-order ?order-id) (parent ?root-id))
)

(deffunction goal-production-assert-c1
  (?root-id ?order-id ?wp-for-order ?cs ?rs ?col-cap ?col-base ?col-ring1)

  (bind ?goal
    (goal-tree-assert-central-run-parallel PRODUCE-ORDER
		;deliver-rc21 is a change specific to robocup 2021, for the normal game keep
		;the old structure
		;(goal-production-assert-deliver ?wp-for-order)
		(goal-production-assert-deliver-rc21 ?wp-for-order)
		(goal-tree-assert-central-run-parallel PREPARE-CS
			(goal-tree-assert-central-run-parallel BUFFER-GOALS
				(goal-production-assert-buffer-cap ?cs ?col-cap)
				(goal-production-assert-instruct-cs-buffer-cap ?cs ?col-cap)
			)
		)
		(goal-tree-assert-central-run-parallel MOUNT-GOALS
			(goal-tree-assert-central-run-one INTERACT-BS
				(goal-tree-assert-central-run-parallel OUTPUT-BS
					(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs C-BS OUTPUT)
					(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base OUTPUT)
				)
				(goal-tree-assert-central-run-parallel INPUT-BS
					(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs C-BS INPUT)
					(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base INPUT)
				)
			)
			(goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap)
			(goal-production-assert-instruct-rs-mount-ring ?rs ?col-ring1)
		)
		;deliver-rc21 is a change specific to robocup 2021, for the normal game keep
		;the old structure
		;(goal-production-assert-instruct-ds-deliver ?wp-for-order)
		(goal-tree-assert-central-run-parallel PAYMENT-GOALS
			(goal-production-assert-payment-goals (create$ ?rs) (create$ ?col-ring1))
		)
	)
  )
  (modify ?goal (meta (fact-slot-value ?goal meta) for-order ?order-id) (parent ?root-id))
)

(deffunction goal-production-assert-c2
  (?root-id ?order-id ?wp-for-order ?cs ?rs1 ?rs2 ?col-cap ?col-base ?col-ring1 ?col-ring2)

  (bind ?goal
    (goal-tree-assert-central-run-parallel PRODUCE-ORDER
		;deliver-rc21 is a change specific to robocup 2021, for the normal game keep
		;the old structure
		;(goal-production-assert-deliver ?wp-for-order)
		(goal-production-assert-deliver-rc21 ?wp-for-order)
		(goal-tree-assert-central-run-parallel PREPARE-CS
			(goal-tree-assert-central-run-parallel BUFFER-GOALS
				(goal-production-assert-buffer-cap ?cs ?col-cap)
				(goal-production-assert-instruct-cs-buffer-cap ?cs ?col-cap)
			)
		)
		(goal-tree-assert-central-run-parallel MOUNT-GOALS
			(goal-tree-assert-central-run-one INTERACT-BS
				(goal-tree-assert-central-run-parallel OUTPUT-BS
					(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs2 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs2 ?rs1 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs1 C-BS OUTPUT)
					(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base OUTPUT)
				)
				(goal-tree-assert-central-run-parallel INPUT-BS
					(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs2 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs2 ?rs1 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs1 C-BS INPUT)
					(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base INPUT)
				)
			)
			(goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap)
			(goal-production-assert-instruct-rs-mount-ring ?rs1 ?col-ring1)
			(goal-production-assert-instruct-rs-mount-ring ?rs2 ?col-ring2)
		)
		;deliver-rc21 is a change specific to robocup 2021, for the normal game keep
		;the old structure
		;(goal-production-assert-instruct-ds-deliver ?wp-for-order)
		(goal-tree-assert-central-run-parallel PAYMENT-GOALS
			(goal-production-assert-payment-goals (create$ ?rs1 ?rs2) (create$ ?col-ring1 ?col-ring2))
		)
	)
  )
  (modify ?goal (meta (fact-slot-value ?goal meta) for-order ?order-id) (parent ?root-id))
)

(deffunction goal-production-assert-c3
  (?root-id ?order-id ?wp-for-order ?cs ?rs1 ?rs2 ?rs3 ?col-cap ?col-base ?col-ring1 ?col-ring2 ?col-ring3)

  (bind ?goal
    (goal-tree-assert-central-run-parallel PRODUCE-ORDER
		;deliver-rc21 is a change specific to robocup 2021, for the normal game keep
		;the old structure
		;(goal-production-assert-deliver ?wp-for-order)
		(goal-production-assert-deliver-rc21 ?wp-for-order)
		(goal-tree-assert-central-run-parallel PREPARE-CS
			(goal-tree-assert-central-run-parallel BUFFER-GOALS
				(goal-production-assert-buffer-cap ?cs ?col-cap)
				(goal-production-assert-instruct-cs-buffer-cap ?cs ?col-cap)
			)
		)
		(goal-tree-assert-central-run-parallel MOUNT-GOALS
			(goal-tree-assert-central-run-one INTERACT-BS
				(goal-tree-assert-central-run-parallel OUTPUT-BS
					(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs3 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs3 ?rs2 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs2 ?rs1 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs1 C-BS OUTPUT)
					(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base OUTPUT)
				)
				(goal-tree-assert-central-run-parallel INPUT-BS
					(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs3 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs3 ?rs2 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs2 ?rs1 OUTPUT)
					(goal-production-assert-mount-ring ?wp-for-order ?rs1 C-BS INPUT)
					(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base INPUT)
				)
			)
			(goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap)
			(goal-production-assert-instruct-rs-mount-ring ?rs1 ?col-ring1)
			(goal-production-assert-instruct-rs-mount-ring ?rs2 ?col-ring2)
			(goal-production-assert-instruct-rs-mount-ring ?rs3 ?col-ring3)
		)
		;deliver-rc21 is a change specific to robocup 2021, for the normal game keep
		;the old structure
		;(goal-production-assert-instruct-ds-deliver ?wp-for-order)
		(goal-tree-assert-central-run-parallel PAYMENT-GOALS
			(goal-production-assert-payment-goals (create$ ?rs1 ?rs2 ?rs3) (create$ ?col-ring1 ?col-ring2 ?col-ring3))
		)
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
	(not (wm-fact (key domain fact rs-ring-spec args? $? rn  NA)))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel PRODUCTION-ROOT))
	(modify ?g (meta do-not-finish))
)

(defrule goal-production-create-move-out-of-way
	"Creates a move out of way goal. As soon as it is completed it's reset"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(not (goal (class MOVE-OUT-OF-WAY)))
	(not (wm-fact (key config rcll pick-and-place-challenge) (value TRUE)))
	=>
	(bind ?g1 (goal-production-assert-move-out-of-way M_Z41))
	(bind ?g2 (goal-production-assert-move-out-of-way M_Z31))
	(modify ?g1 (parent ?root-id))
	(modify ?g2 (parent ?root-id))
)

(defrule goal-production-create-pick-and-place
	 "Creates pick and place"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(wm-fact (key config rcll pick-and-place-challenge) (value TRUE))
	(not (goal (class PICK-AND-PLACE)))
	?mps1-free <- (wm-fact (key domain fact mps-side-free args? m C-BS side OUTPUT ))
	?mps2-free <- (wm-fact (key domain fact mps-side-free args? m C-CS1 side OUTPUT ))
	?mps3-free <- (wm-fact (key domain fact mps-side-free args? m C-RS1 side OUTPUT ))
	=>
	(retract ?mps1-free)
	(retract ?mps2-free)
	(retract ?mps3-free)
	(assert
	   (domain-object (name WP-ONE) (type workpiece))
	   (domain-object (name WP-TWO) (type workpiece))
	   (domain-object (name WP-THREE) (type workpiece))
	   (domain-fact (name wp-usable) (param-values WP-ONE))
	   (domain-fact (name wp-usable) (param-values WP-TWO))
	   (domain-fact (name wp-usable) (param-values WP-THREE))
	   (wm-fact (key domain fact maps args? m C-BS r robot1))
	   (wm-fact (key domain fact maps args? m C-CS1 r robot2))
	   (wm-fact (key domain fact maps args? m C-RS1 r robot3))
	   (wm-fact (key domain fact wp-at args? wp WP-ONE m C-BS side OUTPUT))
	   (wm-fact (key domain fact wp-at args? wp WP-TWO m C-CS1 side OUTPUT))
	   (wm-fact (key domain fact wp-at args? wp WP-THREE m C-RS1 side OUTPUT))
	)

	(bind ?g (goal-tree-assert-central-run-parallel PICK-AND-PLACE
		( goal-production-assert-pick-and-place C-BS robot1)
		( goal-production-assert-pick-and-place C-BS robot1)
		( goal-production-assert-pick-and-place C-BS robot1)
		( goal-production-assert-move-robot-to-output C-BS robot1)
	))
	(modify ?g (parent ?root-id))
	(bind ?g1 (goal-tree-assert-central-run-parallel PICK-AND-PLACE
		( goal-production-assert-pick-and-place C-CS1 robot2)
		( goal-production-assert-pick-and-place C-CS1 robot2)
		( goal-production-assert-pick-and-place C-CS1 robot2)
		( goal-production-assert-move-robot-to-output C-CS1 robot2)
	))
	(modify ?g1 (parent ?root-id))
	(bind ?g2 (goal-tree-assert-central-run-parallel PICK-AND-PLACE
		( goal-production-assert-pick-and-place C-RS1 robot3)
		( goal-production-assert-pick-and-place C-RS1 robot3)
		( goal-production-assert-pick-and-place C-RS1 robot3)
		( goal-production-assert-move-robot-to-output C-RS1 robot3)
	))
	(modify ?g2 (parent ?root-id))
)

(defrule goal-production-create-produce-for-order
	"Create for each incoming order a grounded production tree with the"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(wm-fact (key config rcll pick-and-place-challenge) (value FALSE))
	(wm-fact (key domain fact order-complexity args? ord ?order-id&:(eq ?order-id O1) com ?comp))
	(wm-fact (key domain fact order-base-color args? ord ?order-id col ?col-base))
	(wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?col-cap))
	(wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?col-ring1))
	(wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?col-ring2))
	(wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?col-ring3))
	(wm-fact (key domain fact cs-color args? m ?cs col ?col-cap))
	(wm-fact (key domain fact mps-type args? m ?cs t CS))
	(not (wm-fact (key order meta wp-for-order args? wp ?something ord O1)))
	(or (wm-fact (key domain fact order-ring1-color args? ord ?order-id col RING_NONE))
	    (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?col-ring1 $?)))
	(or (wm-fact (key domain fact order-ring2-color args? ord ?order-id col RING_NONE))
	    (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?col-ring2 $?)))
	(or (wm-fact (key domain fact order-ring3-color args? ord ?order-id col RING_NONE))
	    (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?col-ring3 $?)))
	=>
	;find the necessary ringstations
	(bind ?rs1 (goal-production-get-machine-for-color ?col-ring1))
	(bind ?rs2 (goal-production-get-machine-for-color ?col-ring2))
	(bind ?rs3 (goal-production-get-machine-for-color ?col-ring3))

	;create facts for workpiece
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
	(if (and (eq ?comp C1) ?rs1)
		then
		(goal-production-assert-c1 ?root-id ?order-id ?wp-for-order ?cs ?rs1 ?col-cap ?col-base ?col-ring1)
	)
	(if (and (eq ?comp C2) ?rs1 ?rs2)
		then
		(goal-production-assert-c2 ?root-id ?order-id ?wp-for-order ?cs
	              ?rs1 ?rs2 ?col-cap ?col-base ?col-ring1 ?col-ring2)
	)
	(if (and (eq ?comp C3) ?rs1 ?rs2 ?rs3)
		then
		(goal-production-assert-c3 ?root-id ?order-id ?wp-for-order ?cs
	              ?rs1 ?rs2 ?rs3 ?col-cap ?col-base ?col-ring1 ?col-ring2 ?col-ring3)
	)

)

(defrule goal-production-fill-in-unknown-wp-discard
	"Fill in missing workpiece information into the discard goals"
	?g <- (goal (id ?goal-id) (class DISCARD) (mode FORMULATED) (parent ?parent)
	            (params wp UNKNOWN wp-loc ?mps wp-side ?mps-side)
	            (meta $? assigned-to ?robot $?))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side))
	(not (wm-fact (key order meta wp-for-order args? wp ?wp $?)))
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
	                          (params target ?target $?)
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
					(params target (translate-location-map-to-grid ?location) location ?location)
				)))
	(return ?goal)
)

(deffunction goal-production-assert-navigation-challenge
  (?root-id $?locations)

  (bind ?goals (create$))
  (foreach ?location ?locations

	(bind ?goals (insert$ ?goals (+ 1 (length$ ?goals))
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
  (wm-fact (key domain fact waypoints args?) (values $?waypoints))
  (not (goal (class NAVIGATION-CHALLENGE-PARENT)))
  =>
  (goal-production-assert-navigation-challenge ?root-id ?waypoints)
)

(defrule goal-production-assert-wait-nothing-executable
  "When the robot is stuck, assert a new goal that keeps it waiting"
  (declare (salience 0))
  (goal (id ?p) (class PRODUCTION-ROOT))
  (goal (mode FORMULATED) (meta assigned-to ?robot&~central))
  (not (goal (mode FORMULATED) (is-executable TRUE)))
  =>
  (bind ?goal (assert (goal (class WAIT-NOTHING-EXECUTABLE)
	            (id (sym-cat WAIT-NOTHING-EXECUTABLE- (gensym*)))
	            (sub-type SIMPLE)
	            (verbosity NOISY) (is-executable TRUE)
	            (meta assigned-to ?robot)
  )))
  (modify ?goal (parent ?p))
)

(defrule goal-production-remove-retracted-wait-nothing-executable
  "When a waith-nothing-executable goal is retracted, remove it to prevent spam"
  (declare (salience 0))
  ?g <- (goal (class WAIT-NOTHING-EXECUTABLE) (mode RETRACTED))
  =>
  (retract ?g)
)

