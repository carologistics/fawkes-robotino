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

(defrule goal-production-transport-executable
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class TRANSPORT)
	                          (mode FORMULATED)
	                          (params src-mps ?src-mps src-side ?src-side
	                    			  dst-mps ?dst-mps dst-side ?dst-side)
	                          (is-executable FALSE))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps side ?src-side))
	(wm-fact (key domain fact mps-side-free args? m ?dst-mps side ?dst-side))
	(not (wm-fact (key domain fact wp-at args? wp ?other-wp m ?dst-mps side ?dst-side)))
	=>
	(printout t "Goal TRANSPORT executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-discard-executable
" Discard output from a station."
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DISCARD)
	                          (mode FORMULATED)
	                          (params  wp ?wp&~UNKNOWN wp-loc ?wp-loc wp-side ?wp-side)
	                          (is-executable FALSE))

	; MPS-Source CEs
	(wm-fact (key refbox team-color) (value ?team-color))
	(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
	(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

	(or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
	         (wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side ?wp-side)))
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
	=>
	(printout t "Goal DISCARD executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-pay-ring-executable
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class PAY-RING) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params wp ?wp src-mps ?src-mps	ring-mps ?ring-mps)
	            (is-executable FALSE))

	; src MPS CEs
	(wm-fact (key refbox team-color) (value ?team-color))
	(or (wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps $?))
		(and (wm-fact (key domain fact mps-type args? m ?src-mps t BS))
			 (wm-fact (key domain fact mps-state args? m ?src-mps s IDLE))
			 (wm-fact (key domain fact mps-team args? m ?src-mps col ?team-color))
			 (not (wm-fact (key domain fact wp-at args? wp ?any-base-wp m ?src-mps $?)))
		)
	)
	; ring MPS CEs
	(wm-fact (key domain fact mps-type args? m ?ring-mps t RS))
	(wm-fact (key domain fact mps-state args? m ?ring-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?ring-mps col ?team-color))
	(wm-fact (key domain fact rs-filled-with args? m ?ring-mps n ?rs-before&ZERO|ONE|TWO))
	=>
	(printout t "Goal PAY-RING executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-buffer-cap-executable
" Bring a cap-carrier from a cap stations shelf to the corresponding mps input
  to buffer its cap. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class BUFFER-CAP) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params target-mps ?mps cap-color ?cap-color)
	            (is-executable FALSE))
	(not (goal (class BUFFER-CAP) (params target-mps ?mps $?)
			   (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED) (is-executable TRUE)))
	(not (goal (class MOUNT-CAP) (params $? cap-mps ?mps $?)
			   (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED) (is-executable TRUE)))

	; MPS CEs
	(wm-fact (key refbox team-color) (value ?team-color))
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	(wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(wm-fact (key domain fact mps-side-free args? m ?mps side INPUT))
	(wm-fact (key domain fact mps-side-free args? m ?mps side OUTPUT))
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
	(printout t "Goal BUFFER-CAP executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-mount-cap-executable
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-CAP) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params wp ?wp src-mps ?src-mps cap-mps ?cap-mps cap-color ?cap-color)
	            (is-executable FALSE))
	(not (goal (class BUFFER-CAP) (params target-mps ?cap-mps $?)
			   (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED) (is-executable TRUE)))
	(not (goal (class MOUNT-CAP) (params $? cap-mps ?cap-mps $?)
			   (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED) (is-executable TRUE)))
	
	; src MPS CEs
	(wm-fact (key refbox team-color) (value ?team-color))
	(or (wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps $?))
		(and (wm-fact (key domain fact mps-type args? m ?src-mps t BS))
			 (wm-fact (key domain fact mps-state args? m ?src-mps s IDLE))
			 (wm-fact (key domain fact mps-team args? m ?src-mps col ?team-color))
			 (not (wm-fact (key domain fact wp-at args? wp ?any-base-wp m ?src-mps $?)))
		)
	)

	; WP-CES
	(or (wm-fact (key domain fact wp-unused args? wp ?wp))
		(wm-fact (key wp meta next-step args? wp ?wp) (value CAP)))

	; cap MPS CEs
	(wm-fact (key domain fact mps-type args? m ?cap-mps t CS))
	(wm-fact (key domain fact mps-state args? m ?cap-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?cap-mps col ?team-color))
	(wm-fact (key domain fact mps-side-free args? m ?cap-mps side INPUT))
	(wm-fact (key domain fact mps-side-free args? m ?cap-mps side OUTPUT))
	(not (wm-fact (key domain fact wp-at args? wp ?any-cap-in-wp m ?cap-mps side INPUT)))
	(not (wm-fact (key domain fact wp-at args? wp ?any-cap-o-wp m ?cap-mps side OUTPUT)))
	(wm-fact (key domain fact cs-buffered args? m ?cap-mps col ?cap-color))
	(wm-fact (key domain fact cs-can-perform args? m ?cap-mps op MOUNT_CAP))
	=>
	(printout t "Goal MOUNT-CAP executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-mount-ring-executable
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class MOUNT-RING) (sub-type SIMPLE)
	            (mode FORMULATED)
	            (params wp ?wp src-mps ?src-mps
						ring-mps ?ring-mps ring-color ?ring-color ring-nr ?ring-nr)
	            (is-executable FALSE))
	(not (goal (class MOUNT-RING) (params $? ring-mps ?ring-mps $?)
			   (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED) (is-executable TRUE)))
	
	; src MPS CEs
	(wm-fact (key refbox team-color) (value ?team-color))
	(or (wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps $?))
		(and (wm-fact (key domain fact mps-type args? m ?src-mps t BS))
			 (wm-fact (key domain fact mps-state args? m ?src-mps s IDLE))
			 (wm-fact (key domain fact mps-team args? m ?src-mps col ?team-color))
			 (not (wm-fact (key domain fact wp-at args? wp ?any-base-wp m ?src-mps $?)))
		)
	)

	; WP-CES
	(wm-fact (key wp meta next-step args? wp ?wp) (value ?step&:(eq ?step (sym-cat RING ?ring-nr))))
	
	; ring MPS CEs
	(wm-fact (key domain fact mps-type args? m ?ring-mps t RS))
	(wm-fact (key domain fact mps-state args? m ?ring-mps s ~BROKEN))
	(wm-fact (key domain fact mps-team args? m ?ring-mps col ?team-color))
	(wm-fact (key domain fact mps-side-free args? m ?ring-mps side INPUT))
	(or (wm-fact (key domain fact mps-side-free args? m ?ring-mps side OUTPUT))
		(test (eq ?ring-mps ?src-mps)))
	(wm-fact (key domain fact rs-ring-spec args? m ?ring-mps r ?ring-color rn ?bases-needed))
	(wm-fact (key domain fact rs-filled-with args? m ?ring-mps n ?bases-filled))
	(wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
	=>
	(printout t "Goal MOUNT-RING executable" crlf)
	(modify ?g (is-executable TRUE))
)

(defrule goal-production-deliver-executable
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class DELIVER)
	            (mode FORMULATED)
	            (params wp ?wp mps ?mps)
	            (is-executable FALSE))
	(not (goal (class DELIVER) (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED) (is-executable TRUE)))
	(not (and  (goal (class TRANSPORT) (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED)
			   		 (params src-mps ?src-mps src-side ?src-side
	                    dst-mps ?dst-mps dst-side ?dst-side))
				(wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps side ?src-side))))

	; MPS-CES
	(wm-fact (key refbox team-color) (value ?team-color))
	(wm-fact (key domain fact mps-type args? m ?mps t DS))
	(wm-fact (key domain fact mps-state args? m ?mps s IDLE))
	(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
	(not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side INPUT)))
	; WP-CES
	(wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps side ?src-side))
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key wp meta next-step args? wp ?wp) (value DELIVER))
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

(deffunction goal-production-assert-dispense-base
	(?wp ?base-mps ?base-color)

	(bind ?goal (assert (goal (class DISPENSE-BASE)
	      (id (sym-cat DISPENSE-BASE- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta-template goal-meta)
	      (params wp ?wp base-mps ?base-mps base-color ?base-color)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-mount-cap
	(?wp ?src-mps ?cap-mps ?cap-color)

	(bind ?goal (assert (goal (class MOUNT-CAP)
	      (id (sym-cat MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta-template goal-meta)
	      (params wp ?wp src-mps ?src-mps cap-mps ?cap-mps cap-color ?cap-color)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-pay-ring
	(?wp ?src-mps ?ring-mps)

	(bind ?goal (assert (goal (class PAY-RING)
	      (id (sym-cat PAY-RING- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta-template goal-meta)
	      (params wp ?wp src-mps ?src-mps ring-mps ?ring-mps)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-mount-ring
	(?wp ?src-mps ?ring-mps ?ring-color ?ring-nr)

	(bind ?goal (assert (goal (class MOUNT-RING)
	      (id (sym-cat MOUNT-RING- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE) (meta-template goal-meta)
	      (params wp ?wp src-mps ?src-mps
		  		  ring-mps ?ring-mps ring-color ?ring-color ring-nr ?ring-nr)
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

(deffunction goal-production-get-ring-machine-for-color
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

(deffunction goal-production-initialize-wp (?wp)
	"Initialize facts for a workpiece."
	(assert
		  (domain-object (name ?wp) (type workpiece))
		  (domain-fact (name wp-unused) (param-values ?wp))
		  (wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col RING_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE) (type BOOL) (value TRUE))
		  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE) (type BOOL) (value TRUE))
		  
	)
)

(defrule goal-production-finish-order
	"Remove a PRODUCE-ORDER goal when finished."
	?g <- (goal (id ?goal-id) (class PRODUCE-ORDER) (mode FORMULATED))
	(not (goal (id ?child-goal) (parent ?goal-id)))
	=>
  	(modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule goal-production-fill-shelf-cs
	"Add a new cap carrier when there isn't one on spot RIGHT."
	(wm-fact (key domain fact cs-color args? m ?cap-mps col ?cap-color))
	(not (wm-fact (key domain fact wp-on-shelf args? wp ? m ?cap-mps spot LEFT)))
	=>
	(bind ?wp (sym-cat wp-cap-carrier- (gensym*)))
	(assert (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color) (type BOOL) (value TRUE))
			(wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?cap-mps spot LEFT) (type BOOL) (value TRUE))
			)
	(printout t "Filled " ?cap-mps " with workpiece " ?wp crlf)
)

(defrule goal-production-order
	"Create the goals for an order."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(wm-fact (key domain fact order-complexity args? ord ?order-id com ?com))
	(wm-fact (key domain fact order-base-color args? ord ?order-id col ?base-color))
	(wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?cap-color))
	(wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?ring1-color))
	(wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?ring2-color))
	(wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?ring3-color))
	(wm-fact (key domain fact cs-color args? m ?cap-mps col ?cap-color))
	(not (wm-fact (key domain fact order-fulfilled args? ord ?order-id)))
	
	(wm-fact (key refbox game-time) (values $?game-time))
	(wm-fact (key refbox order ?order-id delivery-begin) (value ?order-begin))
	(or
		; Wait until the order delivery window starts in 30 seconds.
		(test (< ?order-begin (+ (nth$ 1 ?game-time) 30)))
		; or if we don't have any goals and no other order can be delivered earlier.
		(and (not (and  (goal (id ?other-produce-goal) (class PRODUCE-ORDER) (mode FORMULATED))
						(goal (parent ?other-produce-goal) (class ?c&~DELIVER))
						(goal (id ?other-produce-goal2) (class PRODUCE-ORDER) (mode FORMULATED))
						(goal (parent ?other-produce-goal2) (class ?c2&~DELIVER))
						(test (neq ?other-produce-goal ?other-produce-goal2))
						))
			 (not (and  (wm-fact (key refbox order ?other-order-id delivery-begin)
	         		  	   		(value ?other-begin&:(< ?other-begin ?order-begin)))
					    (not (goal (class PRODUCE-ORDER) (params order ?other-order-id)))
				   )
			  )
		)
	)

	; Don't create a goal twice.
	(not (goal (class PRODUCE-ORDER) (params order ?order-id) (mode FORMULATED)))
	=>
	(bind ?wp-for-order (sym-cat wp- ?order-id))
	(goal-production-initialize-wp ?wp-for-order)
	(assert (wm-fact (key order meta wp-for-order args? wp ?wp-for-order ord ?order-id)))
	(if (eq ?com C0) then
		(bind ?goal (goal-tree-assert-central-run-parallel PRODUCE-ORDER
			(goal-production-assert-mount-cap ?wp-for-order C-BS ?cap-mps ?cap-color)
			(goal-production-assert-deliver ?wp-for-order C-DS)
		))
	)
	(if (eq ?com C1) then
		(bind ?ring1-mps (goal-production-get-ring-machine-for-color ?ring1-color))
		(bind ?goal (goal-tree-assert-central-run-parallel PRODUCE-ORDER
			(goal-production-assert-mount-ring ?wp-for-order C-BS
									?ring1-mps ?ring1-color 1)
			(goal-production-assert-mount-cap ?wp-for-order ?ring1-mps
									?cap-mps ?cap-color)
			(goal-production-assert-deliver ?wp-for-order C-DS)
		))
	)
	(if (eq ?com C2) then
		(bind ?ring1-mps (goal-production-get-ring-machine-for-color ?ring1-color))
		(bind ?ring2-mps (goal-production-get-ring-machine-for-color ?ring2-color))
		(bind ?goal (goal-tree-assert-central-run-parallel PRODUCE-ORDER
			(goal-production-assert-mount-ring ?wp-for-order C-BS
									?ring1-mps ?ring1-color 1)
			(goal-production-assert-mount-ring ?wp-for-order ?ring1-mps
									?ring2-mps ?ring2-color 2)
			(goal-production-assert-mount-cap ?wp-for-order ?ring2-mps
									?cap-mps ?cap-color)
			(goal-production-assert-deliver ?wp-for-order C-DS)
		))
	)
	(if (eq ?com C3) then
		(bind ?ring1-mps (goal-production-get-ring-machine-for-color ?ring1-color))
		(bind ?ring2-mps (goal-production-get-ring-machine-for-color ?ring2-color))
		(bind ?ring3-mps (goal-production-get-ring-machine-for-color ?ring3-color))
		(bind ?goal (goal-tree-assert-central-run-parallel PRODUCE-ORDER
			(goal-production-assert-mount-ring ?wp-for-order C-BS
									?ring1-mps ?ring1-color 1)
			(goal-production-assert-mount-ring ?wp-for-order ?ring1-mps
									?ring2-mps ?ring2-color 2)
			(goal-production-assert-mount-ring ?wp-for-order ?ring2-mps
									?ring3-mps ?ring3-color 3)
			(goal-production-assert-mount-cap ?wp-for-order ?ring3-mps
									?cap-mps ?cap-color)
			(goal-production-assert-deliver ?wp-for-order C-DS)
		))
	)
  	(modify ?goal (meta (fact-slot-value ?goal meta) for-order ?order-id)
	  			  (parent ?root-id) (params order ?order-id))
	(printout t "Goal PRODUCE-ORDER formulated for " ?order-id crlf)
)

(defrule goal-production-create-payment
" Create a new goal for paying the ring station whenever there isn't one."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))

	; We have space in the ring station.
	(wm-fact (key domain fact rs-filled-with args? m ?ring-mps n ?bases-filled&ZERO|ONE|TWO))

	; We don't have something we can discard.
	(not (and (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
		 	  (wm-fact (key domain fact mps-type args? m ?mps t CS))
		 	  (not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?)))))

	; And we don't have another payment goal.
	(not (goal (class PAY-RING) (params wp ?other-wp src-mps ?src-mps ring-mps ?ring-mps $?)))
	=>
	(bind ?wp-pay (sym-cat wp-pay1- (gensym*)))
	(goal-production-initialize-wp ?wp-pay)
	(bind ?goal (goal-production-assert-pay-ring ?wp-pay C-BS ?ring-mps))
  	(modify ?goal (parent ?root-id))
)

(defrule goal-production-create-payment-cap-carrier
"Create a new goal for paying the ring station with a cap carrier."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))

	; Whenever there is a workpiece in the cap station output,
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	; that workpiece is not needed for an order,
	(not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?)))
	; we have space in the ring station,
	(wm-fact (key domain fact rs-filled-with args? m ?ring-mps n ?bases-filled&ZERO|ONE|TWO))
	; and no payment goal, nor discard goal.
	(not (goal (class PAY-RING) (params wp ?other-wp src-mps ?src-mps ring-mps ?ring-mps $?)))
	(not (goal (class DISCARD)  (params wp ?wp wp-loc ?mps wp-side OUTPUT)))
	=>
	(bind ?goal (goal-production-assert-pay-ring ?wp ?mps ?ring-mps))
  	(modify ?goal (parent ?root-id))
)

(defrule goal-production-create-discard
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))

	; Whenever there is a workpiece in the cap station output,
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	(wm-fact (key domain fact mps-type args? m ?mps t CS))
	; that workpiece is not needed for an order,
	(not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?)))
	; we don't have space in any ring station.
	(not (wm-fact (key domain fact rs-filled-with args? m ?ring-mps n ?bases-filled&ZERO|ONE|TWO)))
	; and we don't have a discard goal.
	(not (goal (class DISCARD) (params  wp ?wp wp-loc ?mps wp-side OUTPUT)))
	=>
	(bind ?goal (goal-production-assert-discard ?wp ?mps OUTPUT))
	(modify ?goal (parent ?root-id))
)

(defrule goal-production-buffer-finished-product
	"Buffer a finished work product that can't be delivered."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))

	; Whenever we have a finished product we can't yet deliver.
	(wm-fact (key wp meta next-step args? wp ?wp) (value DELIVER))
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps side OUTPUT))
	(wm-fact (key refbox game-time) (values $?game-time))
	(wm-fact (key refbox order ?order delivery-begin)
	         	  (value ?begin&:(> ?begin (nth$ 1 ?game-time))))

	; and we have space in the storage station.
	(wm-fact (key domain fact mps-side-free args? m C-SS side ?side))
	(not (wm-fact (key domain fact wp-at args? wp ? m C-SS side ?side)))

	; and we don't have a transport goal yet.
	(not (goal (class TRANSPORT) (params src-mps ?src-mps src-side OUTPUT
										 dst-mps C-SS dst-side ?side)))
	=>

	; We put into the storage station.
	(assert (goal (class TRANSPORT)
	      (id (sym-cat TRANSPORT- (gensym*))) (parent ?root-id) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params src-mps ?src-mps src-side OUTPUT
				  dst-mps C-SS dst-side ?side)
		  (meta-template goal-meta)))
)

(defrule goal-production-create-buffer-cap
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	; Whenever there is a cap station without a cap buffered.
	(wm-fact (key domain fact cs-color args? m ?cap-mps col ?cap-color))
	(not (wm-fact (key domain fact cs-buffered args? m ?cap-mps col ?cap-color)))
	; and we don't have a buffer cap goal for that machine.
	(not (goal (class BUFFER-CAP) (params target-mps ?cap-mps cap-color ?cap-color)))
	=>
	; Buffer a cap.
	(bind ?goal (goal-production-assert-buffer-cap ?cap-mps ?cap-color))
	(modify ?goal (parent ?root-id))
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
	(wm-fact (key domain fact entered-field args? r robot2))
	(wm-fact (key domain fact entered-field args? r robot3))
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
	(goal-meta-assert (goal-production-assert-enter-field ?team-color) ?robot)
)
