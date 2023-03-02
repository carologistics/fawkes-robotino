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
  (not (goal (id ?some-goal-id) (class C0-ORDER)))
  (domain-facts-loaded)
  (wm-fact (key refbox team-color) (value ?team-color))
  =>

; c0 goal
  (printout t "Goal " C0-ORDER " formulated" crlf)
  (bind ?goal-id-c0 (sym-cat C0-ORDER- (gensym*)))
  (assert (goal (class C0-ORDER)
                (id ?goal-id-c0)
                (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta))
  )
  (assert (goal-meta (goal-id ?goal-id-c0)))


; init base wp
  (bind ?wp (sym-cat ABCDE- (gensym*)))
  (assert (domain-object (name ?wp) (type workpiece))
          (domain-fact (name wp-unused) (param-values ?wp))
          (wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE) (type BOOL) (value TRUE))
          )
; subgoal 1 holding Base and cap buffered
  (printout t "Goal " BASE-CAP-READY " formulated" crlf)
  (bind ?goal-id-1 (sym-cat BASE-CAP-READY- (gensym*)))
  (assert (goal (class BASE-CAP-READY)
                (id ?goal-id-1)
                (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
                (parent ?goal-id-c0)
                (verbosity NOISY) (is-executable FALSE)
                (meta-template goal-meta))
  )
  (assert (goal-meta (goal-id ?goal-id-1)))

; subgoal 2 mount cap upon base
  (bind ?goal-id-2 (sym-cat MOUNT-CAP-THEN-GET-WP-GOAL- (gensym*)))
	(assert (goal (class MOUNT-CAP-THEN-GET-WP-GOAL)
                (id ?goal-id-2)
                (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS)
                (parent ?goal-id-c0)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
	))
	(assert (goal-meta (goal-id ?goal-id-2)))

; subgoal 3 deliver
  (bind ?goal-id-3 (sym-cat INSTRUCT-DS-DELIVER- (gensym*)))
	(assert (goal (class INSTRUCT-DS-DELIVER)
                (id ?goal-id-3)
                (sub-type SIMPLE)
                (parent ?goal-id-c0)
	              (params wp ?wp target-mps C-DS)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
	))
	(assert (goal-meta (goal-id ?goal-id-3) (assigned-to robot1)))


; subgoal 1-1 buffer cap then discard base 
  (bind ?goal-id-1-1 (sym-cat BUFFER-CAP-DISCARD-GOAL- (gensym*)))
	(assert (goal (class BUFFER-CAP-DISCARD-GOAL)
					(id ?goal-id-1-1)
					(sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL)
					(parent ?goal-id-1)
					(verbosity NOISY) (is-executable TRUE)
					(meta-template goal-meta)
	))
	(assert (goal-meta (goal-id ?goal-id-1-1)))


; subgoal 1-2 Prepare and get base
  (bind ?goal-id-1-2 (sym-cat PRE-GET-BASE-GOAL- (gensym*)))
	(assert (goal (class PRE-GET-BASE-GOAL)
                (id ?goal-id-1-2)
                (sub-type SIMPLE)
                (parent ?goal-id-1)
                (verbosity NOISY) (is-executable TRUE)
                ;wp base-color facts
                (params wp ?wp target-mps C-BS target-side OUTPUT base-color BASE_RED)
                (meta-template goal-meta)
          )
  )
	(assert (goal-meta (goal-id ?goal-id-1-2)))

; subgoal 2-1 mount cap upon base
  (bind ?goal-id-2-1 (sym-cat MOUNT-CAP-GOAL- (gensym*)))
	(assert (goal (class MOUNT-CAP-GOAL)
                (id ?goal-id-2-1)
                (sub-type SIMPLE)
                (parent ?goal-id-2)
                (params target-mps C-CS1 cap-color CCG1)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta))
  )
	(assert (goal-meta (goal-id ?goal-id-2-1) (assigned-to robot2)))

; subgoal 2-2 get mounted base
  (bind ?goal-id-2-2 (sym-cat GET-MOUNTED-BASE-GOAL- (gensym*)))
	(assert (goal (class GET-MOUNTED-BASE-GOAL)
                (id ?goal-id-2-2)
                (sub-type SIMPLE)
                (parent ?goal-id-2)
	              (params wp ?wp target-mps C-CS1)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta))
	)
	(assert (goal-meta (goal-id ?goal-id-2-2) (assigned-to robot2)))


; subgoal 1-1-1 buffer cap
  (bind ?goal-id-1-1-1 (sym-cat BUFFER-CAP-GOAL- (gensym*)))
	(assert (goal (class BUFFER-CAP-GOAL)
					(id ?goal-id-1-1-1)
					(sub-type SIMPLE)
					(parent ?goal-id-1-1)
					(verbosity NOISY) (is-executable TRUE)
					(params target-cs C-CS1 cc CCG1)
					(meta-template goal-meta)
	))
	(assert (goal-meta (goal-id ?goal-id-1-1-1) (assigned-to robot1)))

; subgoal 1-1-2 discard          todo
  (bind ?goal-id-1-1-2 (sym-cat DISCARD-GOAL- (gensym*)))
	(assert (goal (class DISCARD-GOAL)
					(id ?goal-id-1-1-2)
					(sub-type SIMPLE)
					(parent ?goal-id-1-1)
					(verbosity NOISY) (is-executable TRUE)
					(params target-cs C-CS1 cc CCG1)
					(meta-template goal-meta)
	))
	(assert (goal-meta (goal-id ?goal-id-1-1-2) (assigned-to robot1)))
)

;-----------------------------------------selector--------------------------------
(defrule goal-reasoner-mygoal-select
	?g <- (goal (id ?goal-id) (class C0-ORDER) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)



(defrule goal-reasoner-mygoal-1-select
	?g <- (goal (id ?goal-id) (class BASE-CAP-READY) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)

(defrule goal-reasoner-mygoal-2-select
	?g <- (goal (id ?goal-id) (class MOUNT-CAP-THEN-GET-WP-GOAL) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)

(defrule goal-reasoner-mygoal-3-select
	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DELIVER) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)


; goal 1-1
(defrule goal-reasoner-buffer-cap-discard-goal-select
	?g <- (goal (id ?goal-id) (class BUFFER-CAP-DISCARD-GOAL) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)


; goal 1-2
(defrule goal-reasoner-pre-get-base-goal-select
	?g <- (goal (id ?goal-id) (class PRE-GET-BASE) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)

; todo goal 1-1-1
(defrule goal-reasoner-buffer-cap-goal-select
	?g <- (goal (id ?goal-id) (class BUFFER-CAP-GOAL) (mode FORMULATED))
	; if avaliable robts
  =>
  ; assign robot
  ;	modify (goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(modify ?g (mode SELECTED))
)
(defrule goal-reasoner-discard-goal-select
	?g <- (goal (id ?goal-id) (class DISCARD-GOAL) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)

; goal 2-1
(defrule goal-reasoner-mount-cap-goal-select
	?g <- (goal (id ?goal-id) (class MOUNT-CAP-GOAL) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)

(defrule goal-reasoner-get-mounted-cap-goal-select
	?g <- (goal (id ?goal-id) (class GET-MOUNTED-BASE-GOAL) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)

