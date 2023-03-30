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



;--------------------------------------mygoal------------------------
(defrule goal-production-create-mygoal
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (domain-facts-loaded)
  (wm-fact (key refbox team-color) (value ?team-color))

	(wm-fact (key domain fact order-complexity args? ord ?order com C0))
  ; order not done yet (avoid duplicate deliver
  (not (wm-fact (key domain fact order-fulfilled args? ord ?order)))
  ; no C0 goal doing
  (not (goal (id ?some-goal-id) (class C0-ORDER)))

  ; info about base
	(wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  ; info about cap
	(wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))

  =>
  ; init base wp
  (bind ?wp (sym-cat ABCDE- (gensym*)))
  (assert (domain-object (name ?wp) (type workpiece))
          (domain-fact (name wp-unused) (param-values ?wp))
          (wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE) (type BOOL) (value TRUE))
          (domain-fact (name wp-ring1-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-ring2-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-ring3-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-cap-color) (param-values ?wp CAP_NONE))
          (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order) (type BOOL) (value TRUE))
      )

  ; c0 goal
  (printout t "Goal " C0-ORDER " formulated" crlf)
  (bind ?goal-id-c0 (sym-cat C0-ORDER- (gensym*)))
  (assert (goal (class C0-ORDER)
                (id ?goal-id-c0)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE)
                (params wp ?wp base-color ?base-color cap-color ?cap-color cc ?cc target-mps  ?mps)
                (meta-template goal-meta))
  )
  (assert (goal-meta (goal-id ?goal-id-c0)))
)

(defrule goal-production-create-MyC1Goal
  "Enter the field (drive outside of the starting box)."
  ; what this mean
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (id ?some-goal-id) (class C1-ORDER)))
  (domain-facts-loaded)
  (wm-fact (key refbox team-color) (value ?team-color))

  ; info about base
	(wm-fact (key domain fact order-complexity args? ord ?order com C1))
	(wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))

  ; ring todo

  ; info about cap
	(wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))

  =>
  ; init base wp
  (bind ?wp (sym-cat ABCDE- (gensym*)))
  (assert (domain-object (name ?wp) (type workpiece))
          (domain-fact (name wp-unused) (param-values ?wp))
          (wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE) (type BOOL) (value TRUE))
          (domain-fact (name wp-ring1-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-ring2-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-ring3-color) (param-values ?wp RING_NONE))
          (domain-fact (name wp-cap-color) (param-values ?wp CAP_NONE))
          (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order) (type BOOL) (value TRUE))
      )

  (printout t "Goal " C1-ORDER " formulated" crlf)
  (bind ?goal-id-c1 (sym-cat C1-ORDER- (gensym*)))
  (assert (goal (class C1-ORDER)
                (id ?goal-id-c1)
                (sub-type SIMPLE)
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta)
                (params wp ?wp base-color ?base-color cap-color ?cap-color cc ?cc target-mps ?mps) 
                ))
                
  (assert (goal-meta (goal-id ?goal-id-c1)))
)


(defrule goal-production-remove-c0
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?gf <- (goal (id ?some-goal-id) (mode RETRACTED))
  ?gm <- (goal-meta (goal-id ?some-goal-id))
  =>
  (printout t "Goal removed after c0 complete" crlf)
  (retract ?gf ?gm)
)