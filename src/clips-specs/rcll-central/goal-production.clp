;---------------------------------------------------------------------------
;  goal-production.clp - Generate production goals of RCLL
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

; ----------------------- Util -------------------------------

(defglobal
  ?*PRODUCTION-C0-PRIORITY* = 30
  ?*PRODUCTION-C1-PRIORITY* = 40
  ?*PRODUCTION-C2-PRIORITY* = 50
  ?*PRODUCTION-C3-PRIORITY* = 60

  ?*PRODUCE-C0-AHEAD-TIME* = 150
  ?*PRODUCE-C1-AHEAD-TIME* = 250
  ?*PRODUCE-C2-AHEAD-TIME* = 350
  ?*PRODUCE-C3-AHEAD-TIME* = 450
  ?*DELIVER-AHEAD-TIME* = 60
  
  ?*RS-WORKLOAD-THRESHOLD* = 9
)

(deffunction goal-production-produce-ahead-check (?gt ?begin ?complexity)
	"Checks whether the current game time is within the produce ahead time 
	of the given order's complexity"
	(if (eq ?complexity C3) then
		(return (< ?begin (+ ?gt ?*PRODUCE-C3-AHEAD-TIME*)))
	)
	(if (eq ?complexity C2) then
		(return (< ?begin (+ ?gt ?*PRODUCE-C2-AHEAD-TIME*)))
	)
	(if (eq ?complexity C1) then
		(return (< ?begin (+ ?gt ?*PRODUCE-C1-AHEAD-TIME*)))
	)
	(if (eq ?complexity C0) then
		(return (< ?begin (+ ?gt ?*PRODUCE-C0-AHEAD-TIME*)))
	)
	(return nil)
)

(deffunction goal-production-produce-ahead-terminate (?gt ?end ?complexity)
	"Checks whether the current game time is within the produce ahead time 
	of the given order's complexity"
	(if (eq ?complexity C3) then
		(return (< ?end (+ ?gt ?*PRODUCE-C3-AHEAD-TIME*)))
	)
	(if (eq ?complexity C2) then
		(return (< ?end (+ ?gt ?*PRODUCE-C2-AHEAD-TIME*)))
	)
	(if (eq ?complexity C1) then
		(return (< ?end (+ ?gt ?*PRODUCE-C1-AHEAD-TIME*)))
	)
	(if (eq ?complexity C0) then
		(return (< ?end (+ ?gt ?*PRODUCE-C0-AHEAD-TIME*)))
	)
	(return nil)
)

(deffunction goal-production-count-active-orders ()
	"Count the number of order production root nodes that are not retracted."
	(bind ?order-roots 0)
	(do-for-all-facts 
		((?goal goal) (?goal-meta goal-meta))
		(and 
			(eq ?goal:id ?goal-meta:goal-id) 
			(neq ?goal-meta:root-for-order nil)
			(neq ?goal:mode RETRACTED)
		)
		(bind ?order-roots (+ 1 ?order-roots))
	)

	(return ?order-roots)
)


(deffunction goal-meta-assign-robot-to-goal (?goal ?robot)
"Changes an existing goal-meta fact and assign it to the given robot"
	(if (eq (fact-slot-value ?goal id) FALSE) then
		(printout t "Goal has no id! " ?goal crlf)
		(return)
	)
	(if (eq ?robot nil) then (return ))
	(if (not (do-for-fact ((?f goal-meta))
			(and (eq ?f:goal-id (fact-slot-value ?goal id))
			     (or (eq ?f:restricted-to ?robot)
			         (eq ?f:restricted-to nil)))
			(modify ?f (assigned-to ?robot))))
	 then
		(printout t "FAILED assign robot " ?robot " to goal "
		  (fact-slot-value ?goal id) crlf)
	)
)

(deffunction goal-meta-assert (?goal ?robot ?order-id ?ring-nr)
"Creates the goal-meta fact, assigns the goal to the robot and to its order"
	(assert (goal-meta (goal-id (fact-slot-value ?goal id))
	                   (assigned-to ?robot)
	                   (order-id ?order-id)
	                   (ring-nr ?ring-nr)))
	(return ?goal)
)

(deffunction goal-meta-assert-restricted (?goal ?robot)
"Creates the goal-meta fact and restricts the goal to the robot"
	(if (neq ?robot nil) then
		(assert (goal-meta (goal-id (fact-slot-value ?goal id))
		                   (restricted-to ?robot)))
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
  (or (wm-fact (key config rcll use-static-navgraph) (type BOOL) (value TRUE))
      (forall
        (wm-fact (key central agent robot args? r ?robot))
        (NavGraphWithMPSGeneratorInterface (id ?id&:(eq ?id (remote-if-id ?robot "navgraph-generator-mps"))) (final TRUE))
      )
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

; ----------------------- Maintenance Goals -------------------------------

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
	              (is-executable TRUE))))
	(goal-meta-assert ?goal central nil nil)
)

(defrule goal-production-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
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
	(bind ?goal (assert (goal (id (sym-cat REFILL-SHELF- (gensym*)))
	              (class REFILL-SHELF) (sub-type SIMPLE)
	              (parent ?maintain-id) (verbosity QUIET)
	              (params mps ?mps) (is-executable TRUE))))
	(goal-meta-assert ?goal central nil nil)
)

; ----------------------- Robot Assignment -------------------------------

(defrule goal-production-assign-robot-to-simple-goals
" Before checking SIMPLE goals for their executability, pick a waiting robot
  that should get a new goal assigned to it next. "
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
;	"a simple unassigned goal"
	(goal (id ?oid) (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE))
	(goal-meta (goal-id ?oid) (assigned-to nil))
	(wm-fact (key central agent robot args? r ?robot))
	(not (goal-meta (assigned-to ?robot)))
	(wm-fact (key central agent robot-waiting args? r ?robot))
	=>
	(bind ?longest-waiting 0)
	(bind ?longest-waiting-robot ?robot)
	(delayed-do-for-all-facts ((?waiting wm-fact))
	  (wm-key-prefix ?waiting:key (create$ central agent robot-waiting))
	  (if (or (eq ?longest-waiting 0) (< (fact-index ?waiting) ?longest-waiting))
	   then
	    (bind ?longest-waiting-robot (wm-key-arg ?waiting:key r))
	    (bind ?longest-waiting (fact-index ?waiting))
	  )
	)
	(delayed-do-for-all-facts ((?g goal))
		(and (eq ?g:is-executable FALSE)
		     (eq ?g:sub-type SIMPLE) (eq ?g:mode FORMULATED)
		     (or (not (any-factp ((?gm  goal-meta))
		                        (eq ?gm:goal-id ?g:id)))
		         (any-factp ((?gm goal-meta))
		            (and (eq ?gm:goal-id ?g:id)
		                 (eq ?gm:assigned-to nil)))))
		(goal-meta-assign-robot-to-goal ?g ?robot)
	)
	(modify ?longest-waiting)
)

(defrule goal-production-unassign-robot-from-finished-goals
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	?g <- (goal (id ?id) (sub-type SIMPLE) (mode RETRACTED)
	      (parent ?parent))
	(not (goal (id ?parent) (type MAINTAIN)))
	(goal-meta (goal-id ?id) (assigned-to ?robot&~nil))
	=>
	(remove-robot-assignment-from-goal-meta ?g)
)

; ----------------------- Assert Goal Functions -------------------------------

(deffunction goal-production-assert-buffer-cap
	(?mps ?cap-color ?order-id)

	(bind ?goal (assert (goal (class BUFFER-CAP)
	      (id (sym-cat BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps
	              cap-color ?cap-color)
	)))
	(goal-meta-assert ?goal nil ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-pick-and-place
	(?mps ?robot)
	(bind ?goal (assert (goal (class PICK-AND-PLACE)
	      (id (sym-cat PICK-AND-PLACE- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps)
	)))
	(goal-meta-assert-restricted ?goal ?robot)
	(return ?goal)
)

(deffunction goal-production-assert-move-robot-to-output
	(?mps ?robot)
	(bind ?goal (assert (goal (class MOVE)
	      (id (sym-cat MOVE- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps)
	)))
	(goal-meta-assert-restricted ?goal ?robot)
	(return ?goal)
)

(deffunction goal-production-assert-mount-cap
	(?wp ?mps ?wp-loc ?wp-side ?order-id)

	(bind ?goal (assert (goal (class MOUNT-CAP)
	      (id (sym-cat MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
 	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp
	              target-mps ?mps
	              target-side INPUT
	              wp-loc ?wp-loc
	              wp-side ?wp-side)
	)))
	(goal-meta-assert ?goal nil ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-mount-ring
	(?wp ?rs ?wp-loc ?wp-side ?ring-color ?order-id ?ring-nr)
	(bind ?goal (assert (goal (class MOUNT-RING)
	      (id (sym-cat MOUNT-RING- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params  wp ?wp
	               target-mps ?rs
	               target-side INPUT
	               wp-loc ?wp-loc
	               wp-side ?wp-side
	               ring-color ?ring-color
	               )
	)))
	(goal-meta-assert ?goal nil ?order-id ?ring-nr)
	(return ?goal)
)

(deffunction goal-production-assert-discard
	(?wp ?cs ?side ?order-id)

	(bind ?goal (assert (goal (class DISCARD)
	      (id (sym-cat DISCARD- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp wp-loc ?cs wp-side ?side)
	)))
	(goal-meta-assert ?goal nil ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-deliver-rc21
	(?wp ?order-id)

	(bind ?goal (assert (goal (class DELIVER-RC21)
	      (id (sym-cat DELIVER-RC21- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp)
	)))
	(goal-meta-assert ?goal nil ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-instruct-ds-deliver
	(?wp ?order-id)

	(bind ?goal (assert (goal (class INSTRUCT-DS-DELIVER)
	  (id (sym-cat INSTRUCT-DS-DELIVER- (gensym*))) (sub-type SIMPLE)
	  (verbosity NOISY) (is-executable FALSE)
	  (params wp ?wp
	          target-mps C-DS)
	)))
	(goal-meta-assert ?goal central ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-deliver
	"If there is a DS, do a normal delivery, otherwise do a RoboCup 2021 delivery. "
	(?wp ?order-id ?instruct-parent)

	(bind ?goal nil)
	(if (any-factp ((?state domain-fact)) (and (eq ?state:name mps-state)
	                                           (member$ C-DS ?state:param-values))
	    )
	then

		(bind ?instruct-goal (goal-production-assert-instruct-ds-deliver ?wp ?order-id))
		(modify ?instruct-goal (parent ?instruct-parent))

		(bind ?goal
			(goal-meta-assert (assert (goal (class DELIVER)
				(id (sym-cat DELIVER- (gensym*))) (sub-type SIMPLE)
				(verbosity NOISY) (is-executable FALSE)
				(params wp ?wp
						target-mps C-DS
						target-side INPUT)
			)) nil ?order-id nil)
		)
	else
		(bind ?goal (goal-production-assert-deliver-rc21 ?wp ?order-id))
	)

	(return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-base
	(?wp ?wp-loc ?wp-side ?target-mps ?target-side ?order-id)
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
	(goal-meta-assert ?goal nil ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-cap-carrier
	(?wp ?wp-loc ?wp-side ?target-mps ?target-side ?order-id)

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
	(goal-meta-assert ?goal nil ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-pay-for-rings-with-cap-carrier-from-shelf
	(?wp-loc ?target-mps ?target-side ?order-id)

	(bind ?goal (assert (goal (class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	      (id (sym-cat PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params  wp-loc ?wp-loc
	               target-mps ?target-mps
	               target-side ?target-side
	               )
	)))
	(goal-meta-assert ?goal nil ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-instruct-cs-buffer-cap
	(?mps ?cap-color ?order-id)

	(bind ?goal (assert (goal (class INSTRUCT-CS-BUFFER-CAP)
	      (id (sym-cat INSTRUCT-CS-BUFFER-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps
	              cap-color ?cap-color)
	)))
	(goal-meta-assert ?goal central ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-instruct-bs-dispense-base
	(?wp ?base-color ?side ?order-id)

	(bind ?goal (assert (goal (class INSTRUCT-BS-DISPENSE-BASE)
	  (id (sym-cat INSTRUCT-BS-DISPENSE-BASE- (gensym*))) (sub-type SIMPLE)
	  (verbosity NOISY) (is-executable FALSE)
	      (params wp ?wp
	              target-mps C-BS
	              target-side ?side
	              base-color ?base-color)
	)))
	(goal-meta-assert ?goal central ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-instruct-cs-mount-cap
	(?mps ?cap-color ?order-id)
	(bind ?goal (assert (goal (class INSTRUCT-CS-MOUNT-CAP)
	      (id (sym-cat INSTRUCT-CS-MOUNT-CAP- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	      (params target-mps ?mps
	              cap-color ?cap-color)
	)))
	(goal-meta-assert ?goal central ?order-id nil)
	(return ?goal)
)

(deffunction goal-production-assert-instruct-rs-mount-ring
	(?mps ?col-ring ?order-id ?ring-nr)
	(bind ?goal (assert (goal (class INSTRUCT-RS-MOUNT-RING)
	      (id (sym-cat INSTRUCT-RS-MOUNT-RING- (gensym*))) (sub-type SIMPLE)
	      (verbosity NOISY) (is-executable FALSE)
	            (params target-mps ?mps
	                    ring-color ?col-ring
	             )
	)))
	(goal-meta-assert ?goal central ?order-id ?ring-nr)
	(return ?goal)
)

(deffunction goal-production-assert-payment-goals
	(?rs ?cols-ring ?cs ?order-id ?instruct-parent ?prio)
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
			(bind ?price (sym-to-int (wm-key-arg ?rs-ring-spec:key rn)))
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
					(goal-production-assert-pay-for-rings-with-base ?wp-base-pay C-BS INPUT (nth$ ?index ?rs) INPUT ?order-id)
				)
			)

			(bind ?instruct-goal (goal-production-assert-instruct-bs-dispense-base ?wp-base-pay BASE_RED INPUT ?order-id))
			(modify ?instruct-goal (parent ?instruct-parent))
	 	)
		(bind ?index (+ ?index 1))
	)
	(if (eq ?found-payment TRUE) then
		(bind ?goals (insert$ ?goals 1 (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs UNKNOWN ?first-rs INPUT ?order-id)))
	)
	(if (eq ?found-payment FALSE) then
		(bind ?goals (insert$ ?goals 1 (goal-production-assert-discard UNKNOWN ?cs OUTPUT ?order-id)))
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
	            (meta-template goal-meta)
	)))
	(return ?goal)
)

(deffunction goal-production-assert-move-out-of-way
	(?location)
	(bind ?goal (assert (goal (class MOVE-OUT-OF-WAY)
	            (id (sym-cat MOVE-OUT-OF-WAY- (gensym*)))
	            (sub-type SIMPLE)
	            (verbosity NOISY) (is-executable FALSE)
	            (meta-template goal-meta)
	            (params target-pos (translate-location-map-to-grid ?location) location ?location)
	)))
	(return ?goal)
)

(deffunction goal-production-assign-order-and-prio-to-goal (?goal ?order-id ?prio)
	(bind ?goal-id (fact-slot-value ?goal id))
	(modify ?goal (priority ?prio))
	(do-for-fact ((?goal-meta goal-meta)) (eq ?goal-meta:goal-id ?goal-id)
		(modify ?goal-meta (root-for-order ?order-id))
	)
)

(deffunction goal-production-assert-c0
	(?root-id ?order-id ?wp-for-order ?cs ?col-cap ?col-base)

	;assert the instruct goals
	(bind ?instruct-goals
		(goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C0-PRIORITY*
			(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base OUTPUT ?order-id)
			(goal-production-assert-instruct-cs-buffer-cap ?cs ?col-cap ?order-id)
			(goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap ?order-id)
		)
	)
	(bind ?instruct-parent (fact-slot-value ?instruct-goals id))
	(modify ?instruct-goals (parent ?root-id))
	(goal-production-assign-order-and-prio-to-goal ?instruct-goals ?order-id ?*PRODUCTION-C0-PRIORITY*)

	;assert the main production tree
	(bind ?goal
		(goal-tree-assert-central-run-parallel-prio PRODUCE-ORDER ?*PRODUCTION-C0-PRIORITY*
			(goal-tree-assert-central-run-all-prio PREPARE-CS ?*PRODUCTION-C0-PRIORITY*
				(goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent)
				(goal-production-assert-discard UNKNOWN ?cs OUTPUT ?order-id)
				(goal-production-assert-buffer-cap ?cs ?col-cap ?order-id)
			)
			(goal-tree-assert-central-run-all-prio MOUNT-GOALS ?*PRODUCTION-C0-PRIORITY*
				(goal-production-assert-mount-cap ?wp-for-order ?cs C-BS OUTPUT ?order-id)
			)
		)
	)

	(goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C0-PRIORITY*)
)

(deffunction goal-production-assert-c1
	(?root-id ?order-id ?wp-for-order ?cs ?rs1 ?col-cap ?col-base ?col-ring1)

	;assert the instruct goals
	(bind ?instruct-goals
		(goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C1-PRIORITY*
			(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base OUTPUT ?order-id)
			(goal-production-assert-instruct-cs-buffer-cap ?cs ?col-cap ?order-id)
			(goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap ?order-id)
			(goal-production-assert-instruct-rs-mount-ring ?rs1 ?col-ring1 ?order-id ONE)
		)
	)
	(bind ?instruct-parent (fact-slot-value ?instruct-goals id))
	(modify ?instruct-goals (parent ?root-id))
	(goal-production-assign-order-and-prio-to-goal ?instruct-goals ?order-id ?*PRODUCTION-C1-PRIORITY*)

	;assert the main production tree
	(bind ?goal
		(goal-tree-assert-central-run-parallel-prio PRODUCE-ORDER ?*PRODUCTION-C1-PRIORITY*
			(goal-tree-assert-central-run-parallel-prio PREPARE-CS ?*PRODUCTION-C1-PRIORITY*
				(goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent)
				(goal-production-assert-buffer-cap ?cs ?col-cap ?order-id)
			)
			(goal-tree-assert-central-run-all-prio MOUNT-GOALS ?*PRODUCTION-C1-PRIORITY*
				(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs1 OUTPUT ?order-id)
				(goal-production-assert-mount-ring ?wp-for-order ?rs1 C-BS OUTPUT ?col-ring1 ?order-id ONE)
			)
			(goal-tree-assert-central-run-parallel-prio PAYMENT-GOALS ?*PRODUCTION-C1-PRIORITY*
				(goal-production-assert-payment-goals (create$ ?rs1) (create$ ?col-ring1) ?cs ?order-id ?instruct-parent ?*PRODUCTION-C1-PRIORITY*)
			)
		)
	)

	(goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C1-PRIORITY*)
)

(deffunction goal-production-assert-c2
	(?root-id ?order-id ?wp-for-order ?cs ?rs1 ?rs2 ?col-cap ?col-base ?col-ring1 ?col-ring2)

	(bind ?instruct-goals
		(goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C2-PRIORITY*
			(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base OUTPUT ?order-id)
			(goal-production-assert-instruct-cs-buffer-cap ?cs ?col-cap ?order-id)
			(goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap ?order-id)
			(goal-production-assert-instruct-rs-mount-ring ?rs1 ?col-ring1 ?order-id ONE)
			(goal-production-assert-instruct-rs-mount-ring ?rs2 ?col-ring2 ?order-id TWO)
		)
	)
	(bind ?instruct-parent (fact-slot-value ?instruct-goals id))
	(modify ?instruct-goals (parent ?root-id))
	(goal-production-assign-order-and-prio-to-goal ?instruct-goals ?order-id ?*PRODUCTION-C2-PRIORITY*)

	(bind ?goal
		(goal-tree-assert-central-run-parallel-prio PRODUCE-ORDER ?*PRODUCTION-C2-PRIORITY*
			(goal-tree-assert-central-run-parallel-prio PREPARE-CS ?*PRODUCTION-C2-PRIORITY*
				(goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent)
				(goal-production-assert-buffer-cap ?cs ?col-cap ?order-id)
			)
			(goal-tree-assert-central-run-all-prio MOUNT-GOALS ?*PRODUCTION-C2-PRIORITY*
				(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs2 OUTPUT ?order-id)
				(goal-production-assert-mount-ring ?wp-for-order ?rs2 ?rs1 OUTPUT ?col-ring2 ?order-id TWO)
				(goal-production-assert-mount-ring ?wp-for-order ?rs1 C-BS OUTPUT ?col-ring1 ?order-id ONE)
			)
			(goal-tree-assert-central-run-parallel-prio PAYMENT-GOALS ?*PRODUCTION-C2-PRIORITY*
				(goal-production-assert-payment-goals (create$ ?rs1 ?rs2) (create$ ?col-ring1 ?col-ring2) ?cs ?order-id ?instruct-parent ?*PRODUCTION-C2-PRIORITY*)
			)
		)
	)

	(goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C2-PRIORITY*)
)

(deffunction goal-production-assert-c3
	(?root-id ?order-id ?wp-for-order ?cs ?rs1 ?rs2 ?rs3 ?col-cap ?col-base ?col-ring1 ?col-ring2 ?col-ring3)

	(bind ?instruct-goals
		(goal-tree-assert-central-run-parallel-prio INSTRUCT-ORDER ?*PRODUCTION-C3-PRIORITY*
			(goal-production-assert-instruct-bs-dispense-base ?wp-for-order ?col-base OUTPUT ?order-id)
			(goal-production-assert-instruct-cs-buffer-cap ?cs ?col-cap ?order-id)
			(goal-production-assert-instruct-cs-mount-cap ?cs ?col-cap ?order-id)
			(goal-production-assert-instruct-rs-mount-ring ?rs1 ?col-ring1 ?order-id ONE)
			(goal-production-assert-instruct-rs-mount-ring ?rs2 ?col-ring2 ?order-id TWO)
			(goal-production-assert-instruct-rs-mount-ring ?rs3 ?col-ring3 ?order-id THREE)
		)
	)
	(bind ?instruct-parent (fact-slot-value ?instruct-goals id))
	(modify ?instruct-goals (parent ?root-id))
	(goal-production-assign-order-and-prio-to-goal ?instruct-goals ?order-id ?*PRODUCTION-C3-PRIORITY*)

	(bind ?goal
		(goal-tree-assert-central-run-parallel-prio PRODUCE-ORDER ?*PRODUCTION-C3-PRIORITY*
			(goal-tree-assert-central-run-parallel-prio PREPARE-CS ?*PRODUCTION-C3-PRIORITY*
				(goal-production-assert-deliver ?wp-for-order ?order-id ?instruct-parent)
				(goal-production-assert-buffer-cap ?cs ?col-cap ?order-id)
			)
			(goal-tree-assert-central-run-all-prio MOUNT-GOALS ?*PRODUCTION-C3-PRIORITY*
				(goal-production-assert-mount-cap ?wp-for-order ?cs ?rs3 OUTPUT ?order-id)
				(goal-production-assert-mount-ring ?wp-for-order ?rs3 ?rs2 OUTPUT ?col-ring3 ?order-id THREE)
				(goal-production-assert-mount-ring ?wp-for-order ?rs2 ?rs1 OUTPUT ?col-ring2 ?order-id TWO)
				(goal-production-assert-mount-ring ?wp-for-order ?rs1 C-BS OUTPUT ?col-ring1 ?order-id ONE)
			)
			(goal-tree-assert-central-run-parallel-prio PAYMENT-GOALS ?*PRODUCTION-C3-PRIORITY*
				(goal-production-assert-payment-goals (create$ ?rs1 ?rs2 ?rs3) (create$ ?col-ring1 ?col-ring2 ?col-ring3) ?cs ?order-id ?instruct-parent ?*PRODUCTION-C3-PRIORITY*)
			)
		)
	)

	(goal-production-assign-order-and-prio-to-goal ?goal ?order-id ?*PRODUCTION-C3-PRIORITY*)
)

(defrule goal-production-create-instruction-root
	"Create the production root under which all instruction goal trees for the orders
	are asserted"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(domain-facts-loaded)
	(not (goal (class INSTRUCTION-ROOT)))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key game state) (value RUNNING))
	(wm-fact (key refbox team-color) (value ?color))
	(not (wm-fact (key domain fact rs-ring-spec args? $? rn NA)))
	; Ensure that a MachineInfo was received already.
	; So if there are ring stations with specs, then those specs are registered.
	(wm-fact (key domain fact mps-state args? m ?any-mps s IDLE))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel INSTRUCTION-ROOT))
	(modify ?g (meta do-not-finish) (priority 1.0))
)

(defrule goal-production-create-wait-root
	"Create the WAIT root, which has low priority and dispatches WAIT goals if
	 nothing else is executable.
	"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(domain-facts-loaded)
	(not (goal (class WAIT-ROOT)))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key game state) (value RUNNING))
	(wm-fact (key refbox team-color) (value ?color))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel WAIT-ROOT))
	(modify ?g (meta do-not-finish) (priority 0))
)

(defrule goal-production-create-move-out-of-way
	"Creates a move out of way goal. As soon as it is completed it's reset"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (class INSTRUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(goal (id ?root-id) (class WAIT-ROOT))
	(not (goal (class MOVE-OUT-OF-WAY)))
	(not (wm-fact (key config rcll pick-and-place-challenge) (value TRUE)))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel MOVE-OUT-OF-WAY
	        (goal-production-assert-move-out-of-way M_Z41)
	        (goal-production-assert-move-out-of-way M_Z31))
	)
	(modify ?g (parent ?root-id) (priority 1.0))
)

(defrule goal-production-change-priority-move-out-of-way
	?g <- (goal (id ?goal-id) (class MOVE-OUT-OF-WAY)
	            (type ACHIEVE) (sub-type SIMPLE)
	            (mode FORMULATED) (parent ?pa-id&~nil)
	            (priority ?p&:(or (eq ?p 2) (eq ?p 1)))
	      )
	=>
	(printout t "modify priority of " ?goal-id crlf)
	(modify ?g (priority (- ?p 2)))
)

(defrule goal-production-debug-cap
	"If there is a mismatch between machines and orders, produce output"
	(wm-fact (key domain fact order-cap-color args? ord ?order-id col ?col))
	(not (wm-fact (key domain fact cs-color args? m ?cs col ?col)))
	(goal (id ?root-id) (class INSTRUCTION-ROOT))
	=>
	(printout error "Can not build order " ?order-id " with cap color " ?col " because there is no capstation for it" crlf)
)

(defrule goal-production-debug-ring1
	"If there is a mismatch between machines and orders, produce output"
	(wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?col&~RING_NONE))
	(not (wm-fact (key domain fact rs-ring-spec args? $? r ?col $?)))
	(goal (id ?root-id) (class INSTRUCTION-ROOT))
	=>
	(printout error "Can not build order " ?order-id " with ring-1 color " ?col " because there is no ringstation for it" crlf)
)

(defrule goal-production-debug-ring2
	"If there is a mismatch between machines and orders, produce output"
	(wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?col-ring&~RING_NONE))
	(not (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?col-ring $?)))
	(goal (id ?root-id) (class INSTRUCTION-ROOT))
	=>
	(printout error "Can not build order " ?order-id " with ring-2 color " ?col-ring " because there is no ringstation for it" crlf)
)

(defrule goal-production-debug-ring3
	"If there is a mismatch between machines and orders, produce output"
	(wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?col-ring&~RING_NONE))
	(not (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?col-ring $?)))
	(goal (id ?root-id) (class INSTRUCTION-ROOT))
	=>
	(printout error "Can not build order " ?order-id " with ring-3 color " ?col-ring " because there is no ringstation for it" crlf)
)

(defrule goal-production-init-order-preference-facts
	"Initialise the possible and preferred order facts to track orders of each 
	complexity for production flow control."
	(not (wm-fact (key order fact possible-orders $?)))
	=>
	(assert
		(wm-fact (key order fact possible-orders) (is-list TRUE) (type SYMBOL))
		(wm-fact (key order fact filtered-orders args? filter delivery-ahead) (is-list TRUE) (type SYMBOL))
		(wm-fact (key order fact filtered-orders args? filter delivery-limit) (is-list TRUE) (type SYMBOL))
		(wm-fact (key order fact filtered-orders args? filter workload) (is-list TRUE) (type SYMBOL))
	)
)

(defrule goal-production-append-possible-orders
	"An order is possible if it's not been fulfilled yet and if the machine occupancy
	allows it to be pursued."
	;facts to modify
	?poss <- (wm-fact (key order fact possible-orders) (values $?values))
	;meta information
	(wm-fact (key refbox team-color) (value ?team-color))
	;neither delivered, nor started
	(wm-fact (key domain fact quantity-delivered args? ord ?order-id team ?team-color) (value 0))
	(not (goal-meta (root-for-order ?order-id)))
	;it is not possible yet
	(test (not (member$ ?order-id ?values)))
	=>
	(modify ?poss (values $?values ?order-id))
)

(defrule goal-production-remove-from-possible-orders-active
	"An order that has been started, fulfilled is not possible anymore."
	(wm-fact (key order fact possible-orders) (values $? ?order-id $?))
	(wm-fact (key refbox team-color) (value ?team-color))
	(or 
		(wm-fact (key domain fact quantity-delivered args? ord ?order-id team ?team-color) (value ~0))
		(goal-meta (root-for-order ?order-id))
	)
	?poss <- (wm-fact (key order fact possible-orders) (values $?values))
	=> 
	(modify ?poss (values (delete$ ?values (member$ ?order-id ?values) (member$ ?order-id ?values))))
)


;filter delivery-ahead
(defrule goal-production-filter-orders-delivery-ahead-add
	"Add an order to this filter if its production ahead window is open and isn't closed yet."
	?filtered <- (wm-fact (key order fact filtered-orders args? filter delivery-ahead) (values $?values))
	(wm-fact (key order fact possible-orders) (values $? ?order-id $?))
	(not (wm-fact (key order fact filtered-orders args? filter delivery-ahead) (values $? ?order-id $?)))
	(wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
	;filter condition
	(wm-fact (key refbox order ?order-id delivery-begin) (value ?begin))
	(wm-fact (key refbox order ?order-id delivery-end) (value ?end))
	(wm-fact (key refbox game-time) (values ?gt $?))
	(test 
		(and 
			(goal-production-produce-ahead-check ?gt ?begin ?comp)
			(not (goal-production-produce-ahead-terminate ?gt ?end ?comp))
		)
	)
	=>
	(modify ?filtered (values $?values ?order-id))
)

(defrule goal-production-filter-orders-delivery-ahead-remove
	"Remove an order from this filter if its production ahead window has finally closed."
	?filtered <- (wm-fact (key order fact filtered-orders args? filter delivery-ahead) (values $?values))
	(wm-fact (key order fact filtered-orders args? filter delivery-ahead) (values $? ?order-id $?))
	(wm-fact (key order fact filtered-orders args? filter delivery-ahead) (values $? ?order-id $?))
	(wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
	(or 
		(not (wm-fact (key order fact possible-orders) (values $? ?order-id $?)))
		(and
			;reverse filter condition
			(wm-fact (key refbox order ?order-id delivery-end) (value ?end))
			(wm-fact (key refbox game-time) (values ?gt $?))
			(test (goal-production-produce-ahead-terminate ?gt ?end ?comp))
		)
	)
	=>
	(modify ?filtered (values (delete$ ?values (member$ ?order-id ?values) (member$ ?order-id ?values))))
)

;filter delivery-limit
(defrule goal-production-filter-orders-delivery-limit-add
	"Add an order to this filter its delivery window end is in the future."
	?filtered <- (wm-fact (key order fact filtered-orders args? filter delivery-limit) (values $?values))
	(wm-fact (key order fact possible-orders) (values $? ?order-id $?))
	(not (wm-fact (key order fact filtered-orders args? filter delivery-limit) (values $? ?order-id $?)))
	;filter condition
	(wm-fact (key refbox order ?order-id delivery-end) (value ?end))
	(wm-fact (key refbox game-time) (values ?gt $?))
	(test (< ?gt ?end))
	=>
	(modify ?filtered (values $?values ?order-id))
)

(defrule goal-production-filter-orders-delivery-limit-remove
	"Remove an order from this filter if its delivery window end has arrived."
	?filtered <- (wm-fact (key order fact filtered-orders args? filter delivery-limit) (values $?values))
	(wm-fact (key order fact filtered-orders args? filter delivery-limit) (values $? ?order-id $?))
	(or 
		(not (wm-fact (key order fact possible-orders) (values $? ?order-id $?)))
		(and
			;reverse filter condition
			(wm-fact (key refbox order ?order-id delivery-end) (value ?end))
			(wm-fact (key refbox game-time) (values ?gt $?))
			(test (> ?gt ?end))
		)
	)
	=>
	(modify ?filtered (values (delete$ ?values (member$ ?order-id ?values) (member$ ?order-id ?values))))
)

;filter machine workload
(defrule goal-production-filter-orders-workload-add
	"Add an order to this filter its workload doesn't push the summed workload over any machine's limit."
	?filtered <- (wm-fact (key order fact filtered-orders args? filter workload) (values $?values))
	(wm-fact (key order fact possible-orders) (values $? ?order-id $?))
	(not (wm-fact (key order fact filtered-orders args? filter workload) (values $? ?order-id $?)))
	;filter condition
	(not 
		(and
			(wm-fact (key mps workload overall args? m ?any-rs) (value ?workload))
			(wm-fact (key mps workload order args? m ?any-rs ord ?order-id) (value ?added-workload))
			(test (> (+ ?workload ?added-workload) ?*RS-WORKLOAD-THRESHOLD*))
		)
	)
	=>
	(modify ?filtered (values $?values ?order-id))
)

(defrule goal-production-filter-orders-workload-remove
	"Remove an order from this filter if its workload would push the summed workload over the limit."
	?filtered <- (wm-fact (key order fact filtered-orders args? filter workload) (values $?values))
	(wm-fact (key order fact filtered-orders args? filter workload) (values $? ?order-id $?))
	(or 
		(not (wm-fact (key order fact possible-orders) (values $? ?order-id $?)))
		(and
			(wm-fact (key mps workload overall args? m ?any-rs) (value ?workload))
			(wm-fact (key mps workload order args? m ?any-rs ord ?order-id) (value ?added-workload))
			(test (> (+ ?workload ?added-workload) ?*RS-WORKLOAD-THRESHOLD*))
		)
	)
	=>
	(modify ?filtered (values (delete$ ?values (member$ ?order-id ?values) (member$ ?order-id ?values))))
)

(defrule goal-production-create-produce-for-order
	"Create for each incoming order a grounded production tree with the"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(goal (id ?root-id) (class INSTRUCTION-ROOT) (mode FORMULATED|DISPATCHED))
	(wm-fact (key config rcll pick-and-place-challenge) (value FALSE))
	(wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
	(wm-fact (key domain fact order-base-color args? ord ?order-id col ?col-base))
	(wm-fact (key domain fact order-cap-color  args? ord ?order-id col ?col-cap))
	(wm-fact (key domain fact order-ring1-color args? ord ?order-id col ?col-ring1))
	(wm-fact (key domain fact order-ring2-color args? ord ?order-id col ?col-ring2))
	(wm-fact (key domain fact order-ring3-color args? ord ?order-id col ?col-ring3))
	(wm-fact (key domain fact cs-color args? m ?cs col ?col-cap))
	(wm-fact (key domain fact mps-type args? m ?cs t CS))
	(not (wm-fact (key order meta wp-for-order args? wp ?something ord ?order-id)))
	(or (wm-fact (key domain fact order-ring1-color args? ord ?order-id col RING_NONE))
	    (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?col-ring1 $?)))
	(or (wm-fact (key domain fact order-ring2-color args? ord ?order-id col RING_NONE))
	    (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?col-ring2 $?)))
	(or (wm-fact (key domain fact order-ring3-color args? ord ?order-id col RING_NONE))
	    (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?col-ring3 $?)))
 
	;check if the order should be pursued, i.e. if the following conditions hold
	; - it is a possible order
	; - there is no order of a higher complexity that also meets the criteria
	; - or
	;	- it fulfills all the filters
	;	- no goal fulfills all the filters and no goal is currently pursued
	(wm-fact (key order fact possible-orders) (values $? ?order-id $?))
	(not 
		(and
			(wm-fact (key order fact possible-orders) (values $? ?o-order-id&~?order-id $?))
			(wm-fact (key domain fact order-complexity args? ord ?o-order-id com ?comp-comp))
			(not (wm-fact (key order fact filtered-orders $?) (values $?values&:(not (member$ ?o-order-id ?values)))))
			(test (eq 1 (str-compare ?comp-comp ?comp)))
		)
	)
	(or 
		(not (wm-fact (key order fact filtered-orders $?) (values $?values&:(not (member$ ?order-id ?values)))))
		(not 
			(and
				(goal (id ?oid) (mode FORMULATED|SELECTED|EXPANDED|COMMITTED|DISPATCHED))
				(goal-meta (goal-id ?oid) (root-for-order ~nil))
			)
		)
	)
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
	            (params wp UNKNOWN wp-loc ?mps wp-side ?mps-side))
	(goal-meta (goal-id ?goal-id) (order-id ?order-id))
	(goal (id ?buffer-goal-id) (class BUFFER-CAP) (mode ~FORMULATED))
	(goal-meta (goal-id ?buffer-goal-id) (order-id ?order-id))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side))
	(not (wm-fact (key order meta wp-for-order args? wp ?wp $?)))
	(goal (id ?instruct-goal) (class INSTRUCT-CS-BUFFER-CAP) (mode DISPATCHED|FINISHED|RETRACTED))
	(goal-meta (goal-id ?instruct-goal) (order-id ?order-id))
	=>
	(modify ?g (params wp ?wp wp-loc ?mps wp-side ?mps-side))
)

(defrule goal-production-assert-wait-nothing-executable
  "When the robot is stuck, assert a new goal that keeps it waiting"
  (goal (id ?p) (class WAIT-ROOT))
  (not (goal (parent ?p) (mode FORMULATED)))
  =>
  (bind ?goal (assert (goal (class WAIT-NOTHING-EXECUTABLE)
	            (id (sym-cat WAIT-NOTHING-EXECUTABLE- (gensym*)))
	            (sub-type SIMPLE) (parent ?p) (priority 0.0) (meta-template goal-meta)
	            (verbosity NOISY)
  )))
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
	(goal-production-assert-enter-field ?team-color)
)

(defrule goal-production-remove-enter-field
  "Enter the field (drive outside of the starting box)."
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	?gf <- (goal (id ?some-goal-id) (class ENTER-FIELD) (mode RETRACTED))
	?gm <- (goal-meta (goal-id ?some-goal-id))
	=>
	(printout t "Goal " ENTER-FIELD " removed after entering" crlf)
	(retract ?gf ?gm)
)
