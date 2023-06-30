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

; ----------------------- Production GOALS -------------------------------

(defrule goal-production-enter-field-executable
 " ENTER-FIELD is executable for a robot if it has not entered the field yet."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?id) (class ENTER-FIELD) (sub-type SIMPLE) (mode FORMULATED)
	      (params team-color ?team-color)
	      (is-executable FALSE))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(wm-fact (key refbox state) (value RUNNING))
	(wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
	(wm-fact (key refbox team-color) (value ?team-color))
	; (NavGraphGeneratorInterface (final TRUE))
	(not (wm-fact (key domain fact entered-field
	               args? r ?robot)))
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
	?g <- (goal (id ?id) (class MOVE-OUT-OF-WAY) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-pos ?pos)
	            (is-executable FALSE))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	; check if target position is free
	(not (wm-fact (key domain fact at args? r ?any-robot&:(neq ?robot ?any-robot) m ?pos side WAIT)))
	; check if target position is not last position
	(not (wm-fact (key monitoring robot last-wait-position args? r ?robot) (value ?pos)))
	=>
	(printout t "Goal MOVE-OUT-OF-WAY executable for " ?robot " to pos " ?pos  crlf)
  (modify ?g (is-executable TRUE))
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
	(not (goal (class BUFFER-CAP)
	            (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
	            (params target-mps ?mps
	                    cap-color ?
	            )))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))
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
	        (not (plan-action (action-name wp-get-shelf) (param-values $? ?mps $?)))
	        (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	    )
	    (and
	        (wm-fact (key domain fact holding args? r ?robot wp ?cc))
	        (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	        (domain-object (name ?cc) (type cap-carrier))
	    )
	)
	(domain-fact (name zone-content) (param-values ?zz ?mps))
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
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))
	(wm-fact (key domain fact order-complexity args? ord ?order com ?order-complexity))
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
	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?wp-loc-type))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))
	(not (and (test (eq ?wp-loc-type BS))
	          (wm-fact (key domain fact wp-at args? wp ~?wp m ?wp-loc side ?))))


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
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	;the BS is not in use
	(not (wm-fact (key mps meta bs-in-use args? bs ?wp-loc $?)))

	; prevent other goals from interfering (goal takeover, etc.)
	(wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(not (and
		(goal (class MOUNT-CAP) (id ?oid&~?goal-id) (mode ~FORMULATED) (params $? target-mps ?target-mps $?))
		(goal-meta (goal-id ?oid) (assigned-to ~nil))
	))
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
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))

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
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))

	; refbox CEs, prevent blocking DS by delivering too early
  	(wm-fact (key refbox game-time) (values $?game-time))
	(wm-fact (key refbox order ?order delivery-begin) (type UINT)
	        (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*))))
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
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))

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
	(domain-fact (name zone-content) (param-values ?zz ?wp-loc))
	=>
	(printout t "Goal DELIVER-RC212 executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-discard-executable
" Remove an unused workpiece by bringing it to the delivery station.
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DISCARD)
	                          (mode FORMULATED)
	                          (params  wp ?wp&~UNKNOWN wp-loc ?wp-loc wp-side ?wp-side)
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))

	; Robot CEs
	(wm-fact (key central agent robot args? r ?robot))
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	; MPS-CS CEs
	(wm-fact (key domain fact mps-type args? m ?target-mps t DS))
	(wm-fact (key domain fact mps-state args? m ?target-mps s ~BROKEN))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?target-mps side INPUT)))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?team-color))

	(or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	         (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side ?wp-side)))
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	(domain-fact (name zone-content) (param-values ?zz ?wp-loc))
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
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))
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
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?wp-loc-type))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))
	(not (and (test (eq ?wp-loc-type BS))
	          (wm-fact (key domain fact wp-at args? wp ~?wp m ?wp-loc side ?))))

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
	(domain-fact (name zone-content) (param-values ?zz1 ?wp-loc))
	(domain-fact (name zone-content) (param-values ?zz2 ?target-mps))
	;the BS is not in use
	(not (wm-fact (key mps meta bs-in-use args? bs ?wp-loc $?)))
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
	                                   wp-side ?wp-side
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?other-params)
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (order-id ?order))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))
	(goal (id ?buffer-goal-id) (class BUFFER-CAP) (mode ~FORMULATED))
	(goal-meta (goal-id ?buffer-goal-id) (order-id ?order))
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
	(domain-object (name ?wp) (type cap-carrier))
	(or (and ; Either the workpiece needs to picked up...
	         (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	         (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side OUTPUT))
	    )
	    ; or the workpiece is already being held
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp))
	)
	(not (goal (class DISCARD|PAY-FOR-RINGS-WITH-CAP-CARRIER)
			   (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
			   (params wp ?wp $?))
	)
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	=>
	(bind ?wp-side nil)
	(do-for-fact ((?wp-at wm-fact))
	             (and (wm-key-prefix ?wp-at:key (create$ domain fact wp-at))
	                  (eq (wm-key-arg ?wp-at:key wp) ?wp))
	             (bind ?wp-side (wm-key-arg ?wp-at:key side))
	)
	(printout t "Goal "  PAY-FOR-RINGS-WITH-CAP-CARRIER " executable for WP " ?wp crlf)
	(modify ?g (is-executable TRUE) (params wp ?wp wp-loc ?wp-loc wp-side ?wp-side target-mps ?target-mps target-side ?target-side ?other-params))
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
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))
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
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
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
	                                   ring-color ?ring-color
	                                   $?)
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) (ring-nr ?nr))
	(not (wm-fact (key monitoring goal-in-retry-wait-period args? goal-id ?goal-id robot ?robot)))
	; Robot CEs
	(wm-fact (key refbox team-color) (value ?team-color))
	(not (and (wm-fact (key domain fact mps-type args? m ?wp-loc t BS))
	          (wm-fact (key domain fact wp-at args? wp ~?wp m ?wp-loc side ?))))

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
	(test (eq (sym-cat (sub-string 5 5 ?ring)) (sym-cat (sym-to-int ?nr))))
	; Ring spec & costs
	;(wm-fact (key domain fact rs-ring-spec
	;          args? m ?target-mps r ?ring-color&~RING_NONE rn ?bases-needed))
	;(wm-fact (key domain fact rs-sub args? minuend ?bases-filled
	;                                  subtrahend ?bases-needed
	;                                  difference ?bases-remain&ZERO|ONE|TWO|THREE))


    ; Our target-RS' input must be free
	(wm-fact (key domain fact mps-side-free args? m ?target-mps side INPUT))
    ; One RS side must be kept free
	(wm-fact (key domain fact mps-type args? m ?other-mps&~?target-mps t RS));make sure other mps is a RS
    (or
		(wm-fact (key domain fact mps-side-free args? m ?other-mps side INPUT|OUTPUT))
		(wm-fact (key domain fact mps-side-free args? m ?target-mps side OUTPUT))
		(wm-fact (key domain fact mps-type args? m ?wp-loc t RS))
    )
	; If we want to move the workpiece to a different RS, make sure that there is
    ; no workpiece at the target RS output, that needs to be put into the same input
    (not
        (and
            (wm-fact (key domain fact wp-at args? wp ?other-wp m ?target-mps&~?wp-loc side OUTPUT))
            (wm-fact (key wp meta next-machine args? wp ?other-wp) (value ?target-mps))
        )
    )


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
	(domain-fact (name zone-content) (param-values ?zz1 ?target-mps))
	(domain-fact (name zone-content) (param-values ?zz2 ?wp-loc))
	;the BS is not in use
	(not (wm-fact (key mps meta bs-in-use args? bs ?wp-loc $?)))

	; Goal CEs
	(not (goal (class MOUNT-RING) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED) (params $? target-mps ?target-mps $?)))
	=>
	(printout t "Goal MOUNT-RING executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)




; ----------------------- MPS Instruction GOALS -------------------------------

(defrule goal-production-instruct-cs-buffer-cap-executable
" Instruct cap station to buffer a cap. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class INSTRUCT-CS-BUFFER-CAP) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (id ?goal-id)
	            (params target-mps ?mps
	                    cap-color ?cap-color
	             )
	             (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (order-id ?order-id))
	(not (goal (class INSTRUCT-CS-BUFFER-CAP) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED) (params target-mps ?mps $?)))

	(goal (id ?buffer-goal-id) (class BUFFER-CAP) (sub-type SIMPLE) (mode ~FORMULATED))
	(goal-meta (goal-id ?buffer-goal-id) (order-id ?order-id))

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
	(domain-fact (name zone-content) (param-values ?mpsz ?mps))

	(not (plan-action (action-name wp-check) (param-values $? ?mps $?) (state ~FINAL)))
	=>
	(printout t "Goal INSTRUCT-CS-BUFFER-CAP executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-instruct-cs-mount-cap-executable
" Instruct cap station to buffer a cap. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class INSTRUCT-CS-MOUNT-CAP) (sub-type SIMPLE)
	            (id ?goal-id)
	            (mode FORMULATED)
	            (params target-mps ?mps
	                    cap-color ?cap-color
	             )
	             (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (order-id ?order-id))
	(goal (id ?mount-goal-id) (class MOUNT-CAP) (sub-type SIMPLE) (mode ~FORMULATED))
	(goal-meta (goal-id ?mount-goal-id) (order-id ?order-id))

	(not (goal (class INSTRUCT-CS-MOUNT-CAP) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
	(wm-fact (key refbox team-color) (value ?team-color))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~DOWN&~PROCESSED))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
	(wm-fact (key domain fact cs-buffered args? m ?mps col ?any-cap-color))
	; WP CEs
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
	(wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side OUTPUT)))
	(domain-fact (name zone-content) (param-values ?mpsz ?mps))

	(not (plan-action (action-name wp-check) (param-values $? ?mps $?) (state ~FINAL)))
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
	            (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ~nil) (order-id ?order-id))
	(wm-fact (key refbox team-color) (value ?team-color))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t BS))
	(wm-fact (key domain fact mps-state args? m ?mps s IDLE))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	; WP CEs
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps $?)))
	(wm-fact (key domain fact wp-unused args? wp ?wp))
	; wait until a robot actually needs the base before proceeding
	(plan-action (action-name wait-for-wp) (param-values ?robot ?mps ?side ?wp)
	             (goal-id ?oid) (state PENDING|RUNNING)
	             (precondition ?precondition-id))
	(goal-meta (goal-id ?oid) (order-id ?order-id))
	(not (goal (class INSTRUCT-BS-DISPENSE-BASE) (mode SELECTED|DISPATCHED|COMMITTED|EXPANDED)))
	(domain-fact (name zone-content) (param-values ?mpsz ?mps))

	(not (plan-action (action-name wp-check) (param-values $? ?mps $?) (state ~FINAL)))
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
	            (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(not (goal (class INSTRUCT-DS-DELIVER|INSTRUCT-DS-DISCARD) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
	(wm-fact (key refbox team-color) (value ?team-color))
	(wm-fact (key domain fact mps-type args? m ?mps t DS))
	(wm-fact (key domain fact mps-state args? m ?mps s IDLE))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key refbox game-time) (values $?game-time))
	(wm-fact (key refbox order ?order delivery-begin) (type UINT)
	         (value ?begin&:(< ?begin (nth$ 1 ?game-time))))
	(domain-fact (name zone-content) (param-values ?mpsz ?mps))

	(not (plan-action (action-name wp-check) (param-values $? ?mps $?) (state ~FINAL)))
	=>
	(printout t "Goal INSTRUCT-DS-DELIVER executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-instruct-discard-executable
" Instruct the DS to consume the product to be discarded"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DISCARD) (mode FORMULATED)
	            (params wp ?wp target-mps ?mps) (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(not (goal (class INSTRUCT-DS-DELIVER|INSTRUCT-DS-DISCARD) (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))

	; Refbox CEs
	(wm-fact (key refbox team-color) (value ?team-color))

	; MPS-Source CEs
	(wm-fact (key domain fact mps-type args? m ?mps t DS))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))

	(not (plan-action (action-name wp-check) (param-values $? ?mps $?) (state ~FINAL)))
	
	(domain-fact (name zone-content) (param-values ?mpsz ?mps))
	=>
	(printout t "Goal INSTRUCT-DS-DISCARD executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-instruct-rs-mount-ring-executable
" Instruct ring station to mount a ring on the product. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (class INSTRUCT-RS-MOUNT-RING) (sub-type SIMPLE)
	            (mode FORMULATED) (id ?oid)
	            (params target-mps ?mps
	                    ring-color ?ring-color
	             )
	             (is-executable FALSE))
	(goal-meta (goal-id ?oid) (ring-nr ?ring-num))
	(wm-fact (key refbox team-color) (value ?team-color))
	; MPS CEs
	(wm-fact (key domain fact mps-type args? m ?mps t RS))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~DOWN))
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
	(wm-fact (key wp meta next-step args? wp ?wp)
	         (value ?ring&:(eq (sub-string 5 5 ?ring) (str-cat (sym-to-int ?ring-num)))))
	(wm-fact (key domain fact ?wp-ring-color&:(eq ?wp-ring-color
	         (sym-cat wp-ring (sub-string 5 5 ?ring) -color))
	          args? wp ?wp col RING_NONE ))
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key domain fact ?order-ring-color&:(eq ?order-ring-color
	         (sym-cat order-ring (sub-string 5 5 ?ring) -color))
	          args? ord ?order col ?ring-color ))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side OUTPUT)))
	(not (goal (class INSTRUCT-RS-MOUNT-RING) (mode EXPANDED|SELECTED|DISPATCHED|COMMITTED) (params target-mps ?mps $?)))
	(domain-fact (name zone-content) (param-values ?mpsz ?mps))

	(not (plan-action (action-name wp-check) (param-values $? ?mps $?) (state ~FINAL)))

	(not (and
		(points-timer (goal-id ?fill-goal-id) (action-name wp-put-slide-cc))
		(goal (id ?fill-goal-id) (params $? target-mps ?mps $?))
	))
	=>
	(printout t "Goal INSTRUCT-RS-MOUNT-RING executable" crlf)
	(modify ?g (is-executable TRUE))
)

; ----------------------- MPS Instruction GOALS -------------------------------

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



