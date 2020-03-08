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

(defrule goal-production-create-beacon-maintain
" The parent goal for beacon signals. Allows formulation of
  goals that periodically communicate with the refbox.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class BEACON-MAINTAIN)))
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
  (not (goal (class SEND-BEACON)))
  ; TODO: make interval a constant
  =>
  (assert (goal (id (sym-cat SEND-BEACON- (gensym*))) (sub-type SIMPLE)
                (class SEND-BEACON) (parent ?maintain-id) (verbosity QUIET)))
)


(defrule goal-production-create-wp-spawn-maintain
  "Maintain Spawning if the spawning-master token is held"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
	(not (mutex (name ?n&:(eq ?n (resource-to-mutex wp-spawn))) (state LOCKED)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (not (goal (class WP-SPAWN-MAINTAIN)))
 =>
  (bind ?goal (goal-tree-assert-run-endless WP-SPAWN-MAINTAIN 1))
  (modify ?goal (required-resources wp-spawn)
                (params frequency 1 retract-on-REJECTED)
                (verbosity QUIET))
)


(defrule goal-production-create-wp-spawn-achieve
  "Spawn a WP for each robot, if you are the spawn-master"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (time $?now)
  ?g <- (goal (id ?maintain-id) (class WP-SPAWN-MAINTAIN) (mode SELECTED))
  (not (goal (class SPAWN-WP)))
  (not (goal (class SPAWN-SS-C0)))
  (domain-object (name ?robot) (type robot))
  (not
    (and
    (domain-object (name ?wp) (type workpiece))
    (wm-fact (key domain fact wp-spawned-for args? wp ?wp r ?robot)))
  )
  (wm-fact (key refbox phase) (value PRODUCTION))
  =>
  (assert (goal (id (sym-cat SPAWN-WP- (gensym*))) (sub-type SIMPLE)
                (class SPAWN-WP) (parent ?maintain-id)
                (params robot ?robot)))
)


(defrule goal-production-create-ss-spawn
" Spawn a C0 into the storage station.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?g <- (goal (id ?maintain-id) (class WP-SPAWN-MAINTAIN) (mode SELECTED))
  (not (goal (class SPAWN-WP)))
  (not (goal (class SPAWN-SS-C0)))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  ; Give Replenisher some time to place the C0 into the SS
  (wm-fact (key refbox game-time) (values ?sec&:(> ?sec 45) $?))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact self args? r ?robot))
  ;Standing Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com C0))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key refbox order ?order delivery-begin) (value 0))
  (wm-fact (key config rcll store-standing-c0) (value ?store-standing))
  (wm-fact (key config rcll use-ss) (value TRUE))
  (wm-fact (key domain fact wp-cap-color args? wp ? col ?other-cap-color))
  (test (or (and (eq ?other-cap-color ?cap-color) ?store-standing)
            (and (not ?store-standing)
                 (not (eq ?other-cap-color ?cap-color))
                 (not (eq ?other-cap-color CAP_NONE)))))
  ;SS CEs
  (wm-fact (key domain fact mps-type args? m ?ss t SS))
  (wm-fact (key domain fact mps-state args? m ?ss s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?ss col ?team-color))
  (not (wm-fact (key domain fact ss-initialized args? m ?ss)))
  =>
  (assert (goal (id (sym-cat SPAWN-SS-C0- (gensym*))) (sub-type SIMPLE)
                (class SPAWN-SS-C0) (parent ?maintain-id)
                (params robot ?robot ss ?ss base ?base-color cap ?other-cap-color)))
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
                (params mps ?mps)))
)


(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (final TRUE))
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


(defrule goal-production-create-production-maintain
" The parent production goal. Allows formulation of
  production goals only if the proper game state selected
  and the domain got loaded. Other production goals are
  formulated as sub-goals of this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class PRODUCTION-MAINTAIN)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (NavGraphWithMPSGeneratorInterface (final TRUE))
  (wm-fact (key navgraph waitzone generated))
  =>
  (goal-tree-assert-run-endless PRODUCTION-MAINTAIN 1)
)

(defrule goal-production-create-mps-handling-maintain
" The parent mps handling goal. Allows formulation of
  mps handling goals, if requested by a production goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class MPS-HANDLING-MAINTAIN)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  =>
  (goal-tree-assert-run-endless MPS-HANDLING-MAINTAIN 1)
)

(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class ENTER-FIELD)))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  ; (NavGraphGeneratorInterface (final TRUE))
  ; (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (assert (goal (id (sym-cat ENTER-FIELD- (gensym*)))
                (class ENTER-FIELD) (sub-type SIMPLE)))
)

