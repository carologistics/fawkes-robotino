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
	(?location)

	(bind ?goal (assert (goal (class NAVIGATION-CHALLENGE-MOVE)
					(id (sym-cat NAVIGATION-CHALLENGE-MOVE- (gensym*)))
					(sub-type SIMPLE) (meta-template goal-meta)
					(verbosity NOISY) (is-executable FALSE)
					(params target (translate-location-map-to-grid ?location) location ?location)
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



;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;
; EXPLORATION CHALLENGE ;
;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule goal-production-exploration-challenge-remove-empty-targets
" If no more target are available, remove the fact, this triggers the
  creation of new targets.
"
	?targets <- (wm-fact (key exploration targets args? $?) (values))
	=>
	(retract ?targets)
)


(defrule goal-production-exploration-challenge-create-targets
" Create exploration targets, a list of zones that is targeted in order."
	(not (wm-fact (key exploration targets args? $?)))
	(wm-fact (key exploration active) (value TRUE))
	; start to explore the grid only if grid coordinates are available
	(wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE))
	(confval (path ?min-path&:(eq ?min-path (str-cat ?*NAVGRAPH_GENERATOR_MPS_CONFIG* "bounding-box/p1")))
	         (list-value ?x_min ?y_min))
	(confval (path ?max-path&:(eq ?max-path (str-cat ?*NAVGRAPH_GENERATOR_MPS_CONFIG* "bounding-box/p2")))
	         (list-value ?x_max ?y_max))
	=>
	(bind ?zones (create$))
	(loop-for-count (?x ?x_min -1)
		(loop-for-count (?y (+ 1 ?y_min) ?y_max)
			(bind ?zones (append$ ?zones (translate-location-grid-to-map (abs ?x) ?y)))
		)
	)
	(loop-for-count (?x 1 ?x_max)
		(loop-for-count (?y (+ 1 ?y_min) ?y_max)
			(bind ?zones (append$ ?zones (translate-location-grid-to-map (abs ?x) ?y)))
		)
	)

	(assert (wm-fact (key exploration targets args?)
	                 (is-list TRUE)
	                 (values (randomize$ ?zones)))
	)
)

(defrule goal-production-exploration-challenge-assert-root
	"Create the exploration root where all goals regarding the finding of stations
   are located"
	(declare (salience ?*SALIENCE-GOAL-FORMULATE*))
	(domain-facts-loaded)
	(not (goal (class EXPLORATION-ROOT)))
	(wm-fact (key config rcll start-with-waiting-robots) (value TRUE))
	(wm-fact (key refbox phase) (value EXPLORATION|PRODUCTION))
	(wm-fact (key game state) (value RUNNING))
	(wm-fact (key refbox team-color) (value ?color))
	(wm-fact (key exploration active) (value TRUE))
	=>
	(bind ?g (goal-tree-assert-central-run-parallel EXPLORATION-ROOT))
	(modify ?g (meta do-not-finish) (priority 0.0))
)

(defrule goal-production-exploration-challenge-move-executable
" Move to a navgraph node
"
	(declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
	?g <- (goal (id ?goal-id) (class EXPLORATION-MOVE)
	                          (mode FORMULATED)
	                          (params target ?target $?)
	                          (is-executable FALSE))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(navgraph-node (name ?str-target&:(eq ?str-target (str-cat ?target))))
	=>
	(printout t "Goal EXPLORATION-MOVE executable for " ?robot crlf)
	(modify ?g (is-executable TRUE))
)

(deffunction goal-production-exploration-challenge-assert-move
	(?location)

	(bind ?goal (assert (goal (class EXPLORATION-MOVE)
	        (id (sym-cat EXPLORATION-MOVE- (gensym*)))
	        (sub-type SIMPLE)
	        (priority 1.0)
	        (meta-template goal-meta)
	        (verbosity NOISY) (is-executable FALSE)
	        (params target (translate-location-map-to-grid ?location) location ?location)
	        )))
	(return ?goal)
)

(defrule goal-production-cleanup-exploration-move
" A exploration move that is not executable can be removed as the target can
  never be targeted again.
"
	(declare (salience ?*SALIENCE-GOAL-REJECT*))
	?g <- (goal (id ?goal-id) (class EXPLORATION-MOVE) (mode FORMULATED) (is-executable FALSE))
	?gm <- (goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	=>
	(retract ?g ?gm)
)

(defrule goal-production-exploration-create-move-goal-lacking-choice
  "The robot has nothing it can do, move it across the map to explore"
	(goal (id ?root-id) (class EXPLORATION-ROOT) (mode FORMULATED|DISPATCHED))
	(wm-fact (key central agent robot-waiting args? r ?robot))
	?exp-targ <- (wm-fact (key exploration targets args?) (values ?location $?locations))
	(not (goal (class EXPLORATION-MOVE) (mode FORMULATED)))
	(wm-fact (key exploration active) (type BOOL) (value TRUE))
	=>
	(bind ?goal
	      (goal-production-exploration-challenge-assert-move ?location)
	)
	(modify ?goal (parent ?root-id))
	(modify ?exp-targ (values ?locations))
)

(defrule goal-production-exploration-challenge-cleanup
	?g <- (goal (class EXPLORATION-MOVE) (mode RETRACTED) (outcome FAILED|COMPLETED))
	=>
	(retract ?g)
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
