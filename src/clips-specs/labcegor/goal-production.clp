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

; ----------------------- Robot Assignment -------------------------------

(defrule goal-production-assign-robot-to-enter-field
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (goal (id ?oid) (class ENTER-FIELD)  (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE))
  ?gm <- (goal-meta (goal-id ?oid) (assigned-to nil))
  (not (goal-meta (assigned-to ?robot)))
  =>
  (modify ?gm (assigned-to ?robot))
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


(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key central agent robot args? r ?robot))
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  (not (goal (id ?some-goal-id) (class ENTER-FIELD) (mode FORMULATED|SELECTED|EXPANDED|COMMITTED)))
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

;;;;;;;;;;;;  CHANGES BY VISHWAS JAIN  ;;;;;;;;;;;

;(defrule goal-production-create-testgoal
;  "Enter the field (drive outside of the starting box)."
;  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;  (wm-fact (key central agent robot args? r ?robot))
;  (wm-fact (key domain fact entered-field args? r ?robot))
;  (not (goal (id ?some-goal-id) (class TESTGOAL)))
;  (domain-facts-loaded)
;  (wm-fact (key refbox team-color) (value ?team-color))
;   =>
;
;  (bind ?goal-1-id (sym-cat TESTGOAL- (gensym*)))
;  (assert (goal (class TESTGOAL)
;                (id ?goal-1-id)
;                (sub-type SIMPLE)
;                (verbosity NOISY) (is-executable FALSE)
;                (params bs C-BS)
;                (meta-template goal-meta)
;  ))
;  (assert (goal-meta (goal-id ?goal-1-id) (assigned-to robot1)))
;
;
;  (bind ?goal-2-id (sym-cat TESTGOAL- (gensym*)))
;  (assert (goal (class TESTGOAL)
;                (id ?goal-2-id)
;                (sub-type SIMPLE)
;				(parent ?goal-1-id)
;                (verbosity NOISY) (is-executable FALSE)
;                (params target-cs C-CS1 cc CCG1)
;                (meta-template goal-meta)
;  ))
;  (assert (goal-meta (goal-id ?goal-2-id) (assigned-to robot1)))
;
;  (printout t "Goal " TESTGOAL " formulated" crlf)
;
;)

(deffunction goal-production-g1-c1-base
	(?rnd-id ?base-clr ?wp ?robot)
	(bind ?goal-1-id (sym-cat ?rnd-id 1))
  (assert (goal (class ORDER1)
                (id ?goal-1-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE)
                (params bs C-BS base-color ?base-clr workpiece ?wp)
                (meta-template goal-meta)
  ))
  (assert (goal-meta (goal-id ?goal-1-id) (assigned-to ?robot)))
)




(deffunction g1-goal-production-assert-c1 ()

	(bind ?goal 
		(goal-tree-assert-central-run-all-sequence PRODUCE-C1 
			()
			()
			()
	

(defrule goal-production-create-from-order
	"Take goal from refbox"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
	(wm-fact (key domain fact order-complexity args?  ord ?ord comp C1))
	(wm-fact (key domain fact order-ring1-color args? ord ?ord col ?rng-clr))
	(wm-fact (key domain fact order-complexity args?  ord  ?ord comp ?ord-cmplx))
	(wm-fact (key domain fact order-base-color args? ord ?ord  col ?base-clr))
	(wm-fact (key domain fact order-cap-color args? ord ?ord col ?cap-clr))
	(wm-fact (key domain fact order-gate args? ord ?ord gate ?ds-gate))
  ;(not (goal (id ?some-goal-id) (class ORDER1)))
  (domain-facts-loaded)
  (wm-fact (key refbox team-color) (value ?team-color))
	=>
	(bind ?rnd-id (gensym*))
  
	(if (or (eq ?rng-clr RING_BLUE) (eq ?rng-clr RING_YELLOW))
		then 
			(bind ?rs C-RS2)
		else 
			(bind ?rs C-RS1)
	)

	(bind ?goal-1-id (sym-cat ?rnd-id 1))
  (assert (goal (class ORDER1)
                (id ?goal-1-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE)
                (params bs C-BS base-color ?base-clr workpiece CCB1)
                (meta-template goal-meta)
  ))
  (assert (goal-meta (goal-id ?goal-1-id) (assigned-to robot1)))


	(bind ?goal-2-id (sym-cat ?rnd-id 2))
  (assert (goal (class ORDER1)
                (id ?goal-2-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE)
                (params target-rs ?rs ring-color ?rng-clr ring-before 1 ring-after 0 ring-req 1 workpiece CCB1)
                (meta-template goal-meta)
  ))
  (assert (goal-meta (goal-id ?goal-2-id) (assigned-to robot1)))


	(bind ?goal-3-id (sym-cat ?rnd-id 3))
  (assert (goal (class ORDER1)
                (id ?goal-3-id)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE)
                (params ds C-DS order ?ord)
                (meta-template goal-meta)
  ))
  (assert (goal-meta (goal-id ?goal-1-id) (assigned-to robot1)))
)



;(defrule goal-production-create-testgoal-1
;  "Enter the field (drive outside of the starting box)."
;  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;  (wm-fact (key central agent robot args? r ?robot))
;  (wm-fact (key domain fact entered-field args? r ?robot))
;  (not (goal (id ?some-goal-id) (class TESTGOAL)))
;  (domain-facts-loaded)
;  (wm-fact (key refbox team-color) (value ?team-color))
;  (goal (id ?goal-2-id) (class TESTGOAL) (mode RETRACTED))
;  =>
;
;  (bind ?goal-1-id (sym-cat TESTGOAL- (gensym*)))
;  (assert (goal (class TESTGOAL)
;                (id ?goal-1-id)
;                (sub-type SIMPLE)
;                (verbosity NOISY) (is-executable FALSE)
;                (params bs C-BS base-color BASE_BLACK workpiece CCB1)
;                (meta-template goal-meta)
;  ))
;  (assert (goal-meta (goal-id ?goal-1-id) (assigned-to robot1)))
;)
;
;
;
;(defrule goal-production-create-testgoal-2
;  "Enter the field (drive outside of the starting box)."
;  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
;  (wm-fact (key central agent robot args? r ?robot))
;  (wm-fact (key domain fact entered-field args? r ?robot))
;  (not (goal (id ?some-goal-id) (class TESTGOAL)))
;  (domain-facts-loaded)
;  (wm-fact (key refbox team-color) (value ?team-color))
;  (goal (class TESTGOAL) (id ?other-goal-id))
;  =>
;
;  (bind ?goal-2-id (sym-cat TESTGOAL- (gensym*)))
;  (assert (goal (class TESTGOAL)
;                (id ?goal-2-id)
;                (sub-type SIMPLE)
;                (parent ?other-goal-id)
;				(verbosity NOISY) (is-executable FALSE)
;                (params target-cs C-CS1 cc CCG1)
;                (meta-template goal-meta)
;  ))
;  (assert (goal-meta (goal-id ?goal-2-id) (assigned-to robot1)))
;)

