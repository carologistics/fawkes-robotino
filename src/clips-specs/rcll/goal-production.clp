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

(defglobal
  ; production order priorities
  ?*PRIORITY-FIND-MISSING-MPS* = 110
  ?*PRIORITY-DELIVER* = 100
  ?*PRIORITY-RESET* = 98
  ?*PRIORITY-CLEAR-BS* = 96
  ?*PRIORITY-FILL-RS-CLEAR-BS* = 97
  ?*PRIORITY-PRODUCE-C3* = 96
  ?*PRIORITY-PRODUCE-C2* = 95
  ?*PRIORITY-PRODUCE-C1* = 94
  ?*PRIORITY-PRODUCE-C0* = 90
  ?*PRIORITY-MOUNT-NEXT-RING* = 92
  ?*PRIORITY-MOUNT-FIRST-RING* = 91
  ?*PRIORITY-CLEAR-CS* = 70
  ?*PRIORITY-CLEAR-CS-NEEDED* = 91
  ?*PRIORITY-CLEAR-RS* = 55
  ?*PRIORITY-FILL-RS-CLEAR-CS* = 73
  ?*PRIORITY-FILL-RS-CLEAR-RS* = 58
  ?*PRIORITY-PREFILL-CS* = 50 ;This priority can be increased by +1
  ?*PRIORITY-WAIT-MPS-PROCESS* = 45
  ?*PRIORITY-PREFILL-RS-WITH-FRESH-BASE* = 40
  ?*PRIORITY-PREFILL-RS* = 30 ;This priority can be increased by up to +4
  ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING* = 20
  ?*PRIORITY-DISCARD* = 10
  ?*PRIORITY-WAIT* = 2
  ?*PRIORITY-GO-WAIT* = 1
  ?*PRIORITY-NOTHING-TO-DO* = -1
  ;TODO:The priorities are copied from old agent
  ;     for the current moment. Filter out unneeded
  ;     later. For now needed for reference.

  ?*PRODUCE-C0-AHEAD-TIME* = 150
  ?*PRODUCE-C0-LATEST-TIME* = 30
  ?*PRODUCE-CX-AHEAD-TIME* = 90
  ?*PRODUCE-CX-LATEST-TIME* = 30
  ?*PRODUCE-CAP-AHEAD-TIME* = 90
  ?*PRODUCE-RING-AHEAD-TIME* = 120

  ?*DELIVER-AHEAD-TIME* = 0
  ?*DELIVER-LATEST-TIME* = 10
  ?*DELIVER-ABORT-TIMEOUT* = 30

  ?*BLOCKING-THRESHOLD* = 60
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
  (wm-fact (key refbox game-time) (type UINT) (values ?now&:(> ?now 120) ?))
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

(defrule goal-production-create-wait
  "Keep waiting at one of the waiting positions."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class NO-PROGRESS) (mode FORMULATED))
  (wm-fact (key domain fact self args? r ?self))
  (domain-object (type waitpoint) (name ?waitpoint))
  (wm-fact (key domain fact at args? r ?self m ?waitpoint&:
               (eq (str-length (str-cat ?waitpoint)) 10) side WAIT))
  =>
  (printout t "Goal " WAIT " formulated" crlf)
  (assert (goal (id (sym-cat WAIT- (gensym*)))
               (class WAIT) (sub-type SIMPLE)
               (priority ?*PRIORITY-WAIT*)
               (parent ?production-id)
               (params r ?self
                       point ?waitpoint)
               (required-resources ?waitpoint)
  ))
)


(defrule goal-production-create-go-wait
  "Drive to a waiting position and wait there."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class NO-PROGRESS) (mode FORMULATED))
  (goal (id ?urgent) (class URGENT) (mode FORMULATED))
  (wm-fact (key domain fact self args? r ?self))
  (domain-object (type waitpoint) (name ?waitpoint&:
               (eq (str-length (str-cat ?waitpoint)) 10)))
  =>
  (do-for-fact ((?wm wm-fact)) (wm-key-prefix ?wm:key (create$ monitoring shame))
    (retract ?wm)
    (bind ?production-id ?urgent)
  )
  (printout t "Goal " GO-WAIT " formulated" crlf)
  (assert (goal (id (sym-cat GO-WAIT- (gensym*)))
                (class GO-WAIT) (sub-type SIMPLE)
                (priority  ?*PRIORITY-GO-WAIT*)
                (parent ?production-id)
                (params r ?self
                        point ?waitpoint
                )
                (required-resources ?waitpoint)
  ))
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


(defrule goal-production-detect-fill-cap-for-competitive-order
" Fill a cap into a cap station.
  Use a capcarrier from the corresponding shelf to feed it into a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?urgent) (class URGENT) (mode FORMULATED))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ?g <- (goal (class FILL-CAP) (mode FORMULATED) (parent ~?urgent)
              (params robot ?robot
                        mps ?mps
                         cc ?cc))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key order meta competitive args? ord ?order) (value TRUE))
  (wm-fact (key config rcll competitive-order-priority) (value "HIGH"))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
           (value ?qd&:(> ?qr ?qd)))
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
           (value ?end&:(> ?end (nth$ 1 ?game-time))))
  =>
  (printout t "Goal " FILL-CAP " for competitive order " ?order
              " is urgent." crlf)
  (modify ?g (parent ?urgent))
)


(defrule goal-production-create-reset-mps
" Reset an mps to restore a consistent world model after getting a workpiece
  from it failed too often.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class URGENT) (mode FORMULATED))
  (wm-fact (key domain fact self args? r ?self))
  ?t <- (wm-fact (key evaluated reset-mps args? m ?mps))
  =>
  (printout t "Goal " RESET-MPS " formulated" crlf)
  (assert (goal (id (sym-cat RESET-MPS- (gensym*)))
                (class RESET-MPS) (priority  ?*PRIORITY-RESET*)
                (parent ?production-id) (sub-type SIMPLE)
                (params r ?self
                        m ?mps
                )
                (required-resources ?mps)
  ))
)


(defrule goal-production-mps-handling-create-prepare-goal
  "Prepare and model processing of a mps"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?pg <- (goal (id ?mps-handling-id) (class MPS-HANDLING-MAINTAIN) (mode SELECTED))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;Requested process CEs
; Separate conditions apply for delivery stations
  (wm-fact (key domain fact mps-type args? m ?mps t ~DS))
  (wm-fact (key mps-handling prepare ?prepare-action ?mps args? $?prepare-params))
  (wm-fact (key mps-handling process ?process-action ?mps args? $?process-params))
  ;MPS CEs
  (wm-fact (key domain fact mps-state args? m ?mps s IDLE))
  (not (wm-fact (key domain fact wp-at args? wp ? m ?mps side OUTPUT)))
  =>
  (bind ?resources (create$ ?mps (sym-cat ?mps -OUTPUT) (sym-cat ?mps -INPUT)))
  (assert (goal (id (sym-cat PROCESS-MPS- ?mps - (gensym*)))
                (class PROCESS-MPS) (sub-type SIMPLE)
                (priority ?*PRIORITY-RESET*)
                (parent ?mps-handling-id)
                (params m ?mps
                )
                (required-resources ?resources)
  ))
  (modify ?pg (mode EXPANDED))
)


(defrule goal-production-mps-handling-create-prepare-goal-delivery
  "Prepare and model processing of a delivery"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?pg <- (goal (id ?mps-handling-id) (class MPS-HANDLING-MAINTAIN) (mode SELECTED))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;Requested process CEs
  ; Separate conditions apply for delivery stations
  (wm-fact (key domain fact mps-type args? m ?mps t DS))
  (wm-fact (key mps-handling prepare ?prepare-action ?mps args? m ?mps ord ?order))
  (wm-fact (key mps-handling process ?process-action ?mps
            args? ord ?order wp ?wp m ?ds $?other))
  (wm-fact (key refbox order ?order delivery-begin) (value ?delivery-begin))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order delivery-begin) (type UINT)
           (value ?begin&:(< ?begin (nth$ 1 ?game-time))))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
  (wm-fact (key domain fact mps-state args? m ?mps s IDLE))
  =>
  (bind ?resources (create$ ?mps ?wp (sym-cat ?mps -INPUT)))
  (assert (goal (id (sym-cat PROCESS-MPS- ?mps - (gensym*)))
                (class PROCESS-MPS) (sub-type SIMPLE)
                (priority ?*PRIORITY-RESET*)
                (parent ?mps-handling-id)
                (params m ?mps ord ?order)
                (required-resources ?resources)
  ))
  (modify ?pg (mode EXPANDED))
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


(defrule goal-production-cleanup-wp-for-order-facts
  "Unbind a workpiece from it's order when it can not be used anymore."
  ?wp-for-order <- (wm-fact (key domain fact wp-for-order
                                 args? wp ?wp ord ?order)
                            (value TRUE))
  (not (wm-fact (key domain fact wp-usable args? wp ?wp)))
  ?do <- (domain-object (name ?wp))
  =>
  (retract ?wp-for-order)
  (delayed-do-for-all-facts ((?wm wm-fact))
    (and (wm-key-prefix ?wm:key (create$ wp meta))
         (eq (wm-key-arg ?wm:key wp) ?wp))
    (retract ?wm)
  )
  (printout debug "WP " ?wp " no longer tied to Order " ?order " because it is
    not usable anymore" crlf)
  (retract ?do)
)
