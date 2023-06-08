;---------------------------------------------------------------------------
;  goal-challenges.clp - Generate production goals of RCLL
;
;  Created: Fri 29 Apr 2022 18:41:00 CET
;  Copyright  2021  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	=>
	(printout t "Goal NAVIGATION-CHALLENGE-MOVE executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(deffunction goal-production-navigation-challenge-assert-move
	(?zone)

	(bind ?goal (assert (goal (class NAVIGATION-CHALLENGE-MOVE)
					(id (sym-cat NAVIGATION-CHALLENGE-MOVE- (gensym*)))
					(sub-type SIMPLE) (meta-template goal-meta)
					(verbosity NOISY) (is-executable FALSE)
					(params zone ?zone)
				)))
	(return ?goal)
)

(deffunction goal-production-navigation-challenge-assert-root
	(?root-id $?locations)

	(bind ?goals (create$))
	(foreach ?location ?locations
		(bind ?goals (insert$ ?goals (+ 1 (length$ ?goals))
		             (goal-production-navigation-challenge-assert-move ?location)))
	)

	(bind ?goal
	  (goal-tree-assert-central-run-parallel NAVIGATION-CHALLENGE-ROOT
	                                         ?goals
	  )
	)

  (modify ?goal (parent ?root-id))
)

(defrule goal-production-navigation-challenge-create-tree
  "Create a goal tree for the navigation challenge if there is a waypoint fact."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?root-id) (class PRODUCTION-ROOT) (mode FORMULATED|DISPATCHED))
  (wm-fact (key domain fact waypoints args?) (values $?waypoints))
  (not (goal (class NAVIGATION-CHALLENGE-ROOT)))
  =>
  (goal-production-navigation-challenge-assert-root ?root-id ?waypoints)
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; PICK-AND-PLACE CHALLENGE ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defrule goal-production-pick-and-place-challenge-create
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
