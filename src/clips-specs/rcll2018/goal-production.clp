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
  ?*PRIORITY-CLEAR-BS* = 97
  ?*PRIORITY-PRODUCE-C3* = 96
  ?*PRIORITY-PRODUCE-C2* = 95
  ?*PRIORITY-PRODUCE-C1* = 94
  ?*PRIORITY-PRODUCE-C0* = 90
  ?*PRIORITY-MOUNT-NEXT-RING* = 92
  ?*PRIORITY-MOUNT-FIRST-RING* = 91
  ?*PRIORITY-CLEAR-CS* = 70
  ?*PRIORITY-CLEAR-RS* = 55
  ?*PRIORITY-PREFILL-CS* = 50 ;This priority can be increased by +1
  ?*PRIORITY-WAIT-MPS-PROCESS* = 45
  ?*PRIORITY-PREFILL-RS-WITH-FRESH-BASE* = 40
  ?*PRIORITY-PREFILL-RS* = 30 ;This priority can be increased by up to +4
  ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING* = 20
  ?*PRIORITY-DISCARD-UNKNOWN* = 10
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

  ?*DELIVER-AHEAD-TIME* = 60
  ?*DELIVER-LATEST-TIME* = 10
  ?*DELIVER-ABORT-TIMEOUT* = 30

)

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
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
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
  (domain-object (name ?robot) (type robot))
  (not
    (and
    (domain-object (name ?wp) (type workpiece))
    (wm-fact (key domain fact wp-spawned-for args? wp ?wp r ?robot)))
  )
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  =>
  (assert (goal (id (sym-cat SPAWN-WP- (gensym*))) (sub-type SIMPLE)
                (class SPAWN-WP) (parent ?maintain-id)
                (params robot ?robot)))
)


(defrule goal-production-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
  (not (mutex (name ?n&:(eq ?n (resource-to-mutex refill-shelf))) (state LOCKED)))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
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
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
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
  (this-fact-wont-ever-exist)
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
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
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
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
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
  (wm-fact (key domain fact self args? r ?self))
  (domain-object (type waitpoint) (name ?waitpoint&:
               (eq (str-length (str-cat ?waitpoint)) 10)))
  =>
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
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd&:(> ?qr ?qd)))
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
           (value ?end&:(> ?end (nth$ 1 ?game-time))))
  =>
  (printout t "Goal " FILL-CAP " for competitive order " ?order
              " is urgent." crlf)
  (modify ?g (parent ?urgent))
)


(defrule goal-production-create-fill-cap
" Fill a cap into a cap station.
  Use a capcarrier from the corresponding shelf to feed it into a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PREPARE-CAPS) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
  ;MPS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?any-cap-color)))
  (not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side INPUT)))
  ;Capcarrier CEs
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  =>
  (bind ?priority-increase 0)
  ;If there is ...
  (if (any-factp ((?order-cap-color wm-fact))
                 ;... an order that requires the prepared cap color ...
                 (and (wm-key-prefix ?order-cap-color:key (create$ domain fact order-cap-color))
                      (eq (wm-key-arg ?order-cap-color:key col) ?cap-color)
                      ;... and this order is currently being processed ...
                      (any-factp ((?wp-for-order wm-fact))
                        (and (wm-key-prefix ?wp-for-order:key (create$ order meta wp-for-order))
                             (eq (wm-key-arg ?wp-for-order:key ord) (wm-key-arg ?order-cap-color:key ord)))))
      )
  then
    ;... then this is more important than pre-filling a cap station with a
    ;  color that is not necessarily needed in the future.
    (bind ?priority-increase 1)
    (printout t "Goal " FILL-CAP " formulated with higher priority" crlf)
  else
    (printout t "Goal " FILL-CAP " formulated" crlf)
  )
  (bind ?distance (node-distance (str-cat ?mps -I)))
  (assert (goal (id (sym-cat FILL-CAP- (gensym*)))
                (class FILL-CAP) (sub-type SIMPLE)
                (priority (+ ?priority-increase ?*PRIORITY-PREFILL-CS* (goal-distance-prio ?distance)))
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        cc ?cc
                )
                (required-resources (sym-cat ?mps -INPUT) ?cc)
  ))
)


(defrule goal-production-create-clear-rs-from-expired-product
  "Remove an unfinished product from the output of a ring station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class CLEAR) (id ?maintain-id) (mode FORMULATED))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;WP CEs
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))

  ;TODO: Discuss strategy, throwing away expired products is usually not desired.
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
    (value ?end&:(< ?end (nth$ 1 ?game-time))))
  =>
  (printout t "Goal " CLEAR-MPS " ("?mps") formulated" crlf)
  (assert (goal (id (sym-cat CLEAR-MPS- (gensym*))) (class CLEAR-MPS)
                (sub-type SIMPLE)
                (priority ?*PRIORITY-CLEAR-RS*)
                (parent ?maintain-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side OUTPUT
                )
                (required-resources (sym-cat ?mps -OUTPUT) ?wp)
  ))
)


(defrule goal-production-create-clear-cs-for-capless-carriers
" Remove a capless capcarrier from the output of a cap station after
  retrieving a cap from it.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class CLEAR) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS CEs
  ;Maybe add a check for the base_color
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
;  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;WP CEs
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  =>
  (printout t "Goal " CLEAR-MPS " ("?mps") formulated" crlf)
  (bind ?prio ?*PRIORITY-CLEAR-CS*)
  (if (any-factp ((?wm wm-fact)) (and (wm-key-prefix ?wm:key (create$ domain fact wp-at))
                                      (eq (wm-key-arg ?wm:key m) ?mps)
                                      (eq (wm-key-arg ?wm:key side) INPUT)))
    then
      (bind ?prio (+ 1 ?prio))
      (printout warn "Enhance CLEAR-MPS priority, since there is a product at the input already" crlf)
  )
  (assert (goal (id (sym-cat CLEAR-MPS- (gensym*)))
                (class CLEAR-MPS) (sub-type SIMPLE)
                (priority ?prio)
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side OUTPUT
                )
                (required-resources (sym-cat ?mps -OUTPUT) ?wp)
  ))
)


(defrule goal-production-create-clear-bs
  "Remove a workpiece from the base station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class URGENT) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t BS))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
  ;WP CEs
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?side))
  =>
  (printout t "Goal " CLEAR-MPS " ("?mps") formulated" crlf)
  (assert (goal (id (sym-cat CLEAR-MPS- (gensym*)))
                (class CLEAR-MPS) (sub-type SIMPLE)
                (priority ?*PRIORITY-CLEAR-BS*)
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side ?side
                )
                (required-resources (sym-cat ?mps - ?side) ?wp)
  ))
)


(defrule goal-production-clear-cs-from-expired-product
  "Remove a finished product from a cap station after it's deadline passed."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class CLEAR) (mode FORMULATED))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~DOWN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;WP CEs
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))

  (wm-fact (key refbox order ?order delivery-end) (type UINT)
           (value ?end&:(< ?end (nth$ 1 ?game-time))))
  =>
  (printout t "Goal " CLEAR-MPS " (" ?mps ") formulated" crlf)
  (assert (goal (id (sym-cat CLEAR-MPS- (gensym*)))
                (class CLEAR-MPS) (sub-type SIMPLE)
                (priority ?*PRIORITY-CLEAR-CS*)
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side OUTPUT
                )
                (required-resources (sym-cat ?mps -OUTPUT) ?wp)
  ))
)


(defrule goal-production-increase-priority-to-prefill-rs-for-started-order
" Add a priority increase of +2 for goals that pre-fill a ring station which
  requires additional bases such that the production of a started product
  can be continued.
"
  ;Compute the priorities before goals get formulated.
  (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))
  (goal (class PRODUCTION-MAINTAIN) (id ?maintain-id) (mode SELECTED))
  (not (goal (class FILL-RS) (mode FORMULATED)))
  ;MPS CEs
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  ;The MPS can mount a ring which needs more bases than currently available.
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps r ?ring-color&~RING_NONE
                  rn ?ring-num&:(neq ?rs-before ?ring-num)))
  (wm-fact (key domain fact rs-sub args? minuend ?ring-num subtrahend ?rs-before difference ?rs-diff))

  ;(TODO: make the mps-state  a precond of the put-slide to save traviling time)

  (wm-fact (key order meta wp-for-order args? wp ?order-wp ord ?order))
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&:(neq ?complexity C0)))
  ;The order requires this ring and the started workpiece does not
  ;have it mounted yet.
  (or (and (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring-color))
           (wm-fact (key domain fact wp-ring1-color args? wp ?order-wp col RING_NONE))
      )
      (and (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring-color))
           (wm-fact (key domain fact wp-ring1-color args? wp ?order-wp col ~RING_NONE))
           (wm-fact (key domain fact wp-ring2-color args? wp ?order-wp col RING_NONE))
      )
      (and (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring-color))
           (wm-fact (key domain fact wp-ring1-color args? wp ?order-wp col ~RING_NONE))
           (wm-fact (key domain fact wp-ring2-color args? wp ?order-wp col ~RING_NONE))
           (wm-fact (key domain fact wp-ring3-color args? wp ?order-wp col RING_NONE))
      )
  )
  ;This is unnecessary as the order was already started.
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
    (value ?qd&:(> ?qr ?qd)))
  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))

  ;TODO: add time considerations to only add a higher priority if it makes sense
  =>
  (printout debug "RS " ?mps " needs an additional base for " ?order " given " ?order-wp " (prio +2)" crlf)
  (if (not (do-for-fact
            ((?wmf wm-fact))
            (eq ?wmf:key (create$ evaluated fact rs-fill-priority args? m ?mps))
            (modify ?wmf (value 2))))
    then
      (assert
        (wm-fact (key evaluated fact rs-fill-priority args? m ?mps) (value 4)))
  )
)


(defrule goal-production-increase-priority-to-prefill-rs-for-unstarted-order
" Add a priority increase of +1 for goals that pre-fill a ring station which
  requires additional bases such that an available order (that was not started
  yet) can use them for the first ring.
  The second and third rings of available orders are not considered as
  priority increases for those can be computed once the order is started.
"
  ;compute the priorities before goals get formulated.
  (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))

  (goal (id ?maintain-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (class FILL-RS) (mode FORMULATED)))
  ;MPS CEs
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  ;The MPS can mount a ring which needs more bases than currently available.
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps r ?ring1-color&~RING_NONE rn ?ring-num))
  (wm-fact (key domain fact rs-sub args? minuend ?ring-num subtrahend ?rs-before difference ?rs-diff))

 ;(TODO: make the mps-state  a precond of the put-slide to save traviling time)

  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&:(neq ?complexity C0)))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd&:(> ?qr ?qd)))
  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  (not (wm-fact (key evaluated fact rs-fill-priority args? m ?mps) (value 2)))
  ;TODO: add time considerations to only add a higher priority if it makes sense
  =>
  (printout debug "RS " ?mps " needs an additional base for " ?order " (prio +1)" crlf)
  (if (not (do-for-fact
            ((?wmf wm-fact))
            (eq ?wmf:key (create$ evaluated fact rs-fill-priority args? m ?mps))
      (modify ?wmf (value 1))))
    then
      (assert
        (wm-fact (key evaluated fact rs-fill-priority args? m ?mps) (value 2)))
  )
)


(defrule goal-production-create-get-base-to-fill-rs
  "Fill the ring station with a fresh base from the base station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?maintain-id) (class PREPARE-RINGS) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS-RS CEs (a cap carrier can be used to fill a RS later)
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN))
  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  (domain-object (name ?bs-side&:(or (eq ?bs-side INPUT) (eq ?bs-side OUTPUT))) (type mps-side))

  (wm-fact (key domain fact order-base-color args? ord ?any-order col ?base-color))
  ; Formulate the goal only if it is not already formulated (prevents doubling
  ; the goals due to matching with RS-1 and RS-2)
  (not (goal (class GET-BASE-TO-FILL-RS) (params robot ?robot
                                          bs ?bs
                                          bs-side ?bs-side
                                          base-color ?
                                          wp ?spawned-wp)))
  =>
  (printout t "Goal " GET-BASE-TO-FILL-RS " formulated" crlf)
  (bind ?distance (node-distance (str-cat ?bs - (if (eq ?bs-side INPUT) then I else O))))
  (assert (goal (id (sym-cat GET-BASE-TO-FILL-RS- (gensym*)))
                (class GET-BASE-TO-FILL-RS)
                (priority  (+ ?*PRIORITY-PREFILL-RS-WITH-FRESH-BASE* (goal-distance-prio ?distance)))
                (parent ?maintain-id) (sub-type SIMPLE)
                             (params robot ?robot
                                     bs ?bs
                                     bs-side ?bs-side
                                     base-color ?base-color
                                     wp ?spawned-wp
                                     )
                            (required-resources ?spawned-wp)
  ))
)


(defrule goal-production-create-get-shelf-to-fill-rs
  "Get a capcarrier from a shelf to feed it later."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?maintain-id) (class PREPARE-RINGS) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS-RS CEs (a cap carrier can be used to fill a RS later)
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?cs spot ?spot))
  ; Formulate the goal only if it is not already formulated (prevents doubling
  ; the goals due to matching with RS-1 and RS-2)
  (not (goal (class GET-SHELF-TO-FILL-RS) (parent ?maintain-id)
             (params robot ?robot cs ?cs wp ?wp spot ?spot
                                     )))
  =>
  (printout t "Goal " GET-SHELF-TO-FILL-RS " formulated" crlf)
  (bind ?distance (node-distance (str-cat ?mps -I)))
  (assert (goal (id (sym-cat GET-SHELF-TO-FILL-RS- (gensym*)))
                (class GET-SHELF-TO-FILL-RS)
                (priority (+ ?*PRIORITY-PREFILL-RS* (goal-distance-prio ?distance)))
                (parent ?maintain-id) (sub-type SIMPLE)
                             (params robot ?robot
                                     cs ?cs
                                     wp ?wp
                                     spot ?spot
                                     )
                             (required-resources ?wp)
  ))
)


(defrule goal-production-create-prefill-ring-station
  ;Fill a ring station with the currently holding workpiece.
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PREPARE-RINGS) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact wp-usable args? wp ?wp))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-state args? m ?mps s ?state&~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  ;CCs don't have a base color. Hence, models base with UNKOWN color
  ; (not (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color)))
  =>
  ;Check if this ring station should be filled with increased priority.
  (bind ?priority-increase 0)
  (do-for-all-facts ((?prio wm-fact)) (and (wm-key-prefix ?prio:key (create$ evaluated fact rs-fill-priority))
                                           (eq (wm-key-arg ?prio:key m) ?mps))
      (if (< ?priority-increase ?prio:value)
         then
          (bind ?priority-increase ?prio:value)
      ))
  ;
  (if (eq ?state DOWN)
    then
      (bind ?priority-increase (- ?priority-increase 1))
  )
  (bind ?distance (node-distance (str-cat ?mps -I)))
  (printout t "Goal " FILL-RS " formulated" crlf)
  (assert (goal (id (sym-cat FILL-RS- (gensym*)))
                (class FILL-RS) (sub-type SIMPLE)
                (priority (+ ?*PRIORITY-PREFILL-RS* ?priority-increase (goal-distance-prio ?distance)))
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        rs-before ?rs-before
                        rs-after ?rs-after
                )
                (required-resources ?mps ?wp)
  ))
)


(defrule goal-production-create-discard-unknown
  "Discard a base which is not needed."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class NO-PROGRESS) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;TODO: Model state IDLE
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  ;only discard if ring stations have at least two bases loaded
  ;(wm-fact (key domain fact rs-filled-with args? m ?mps n TWO|THREE))
  ;question: or would be more correct to create it and later
  ;  reject it because its not useful
  =>
  (printout t "Goal " DISCARD-UNKNOWN " formulated" crlf)
  (assert (goal (id (sym-cat DISCARD-UNKNOWN- (gensym*)))
                (class DISCARD-UNKNOWN) (sub-type SIMPLE)
                (priority ?*PRIORITY-DISCARD-UNKNOWN*)
                (parent ?production-id)
                (params robot ?robot
                        wp ?wp
                )
                (required-resources ?wp)
  ))
  ; (assert (goal-already-tried DISCARD-UNKNOWN))
)


(defrule goal-production-create-produce-c0
" Produce a C0 product: Get the correct base and mount the right cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))
  (goal (id ?urgent) (class URGENT) (mode FORMULATED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side INPUT)))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (domain-object (name ?bs-side&:(or (eq ?bs-side INPUT) (eq ?bs-side OUTPUT))) (type mps-side))
  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  ;To-Do: Model the bs active-side
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key order meta competitive args? ord ?order)
           (value ?competitive))

  (wm-fact (key config rcll competitive-order-priority) (value ?comp-prio))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
	(value ?qd&:(> ?qr ?qd)))
  (wm-fact (key refbox order ?order delivery-begin) (type UINT)
	(value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-AHEAD-TIME*))))
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
	(value ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-LATEST-TIME*))))
  ;Active Order CEs
  ;This order complexity is not produced exclusively while another exclusive
  ;complexity order is already started
  (not (and (wm-fact (key order meta wp-for-order args? wp ?ord-wp ord ?any-order))
            (wm-fact (key domain fact order-complexity args? ord ?any-order com ?other-complexity))
            (wm-fact (key config rcll exclusive-complexities) (values $?other-exclusive&:(member$ (str-cat ?other-complexity) ?other-exclusive)))
            (wm-fact (key config rcll exclusive-complexities) (values $?exclusive&:(member$ (str-cat ?complexity) ?exclusive)))))
  (or (and (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
           (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN))
           (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
           (not (wm-fact (key order meta wp-for-order args? wp ?any-ord-wp ord ?order))))
      (and (wm-fact (key domain fact holding args? r ?robot wp ?spawned-wp))
           (wm-fact (key domain fact wp-base-color args? wp ?spawned-wp col ?base-color))
           (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
           (wm-fact (key domain fact wp-cap-color args? wp ?wp cap-color CAP_NONE))))
  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  (test (eq ?complexity C0))
  (not (goal (class PRODUCE-C0)
             (parent ?parent)
             (params robot ?robot
                     bs ?bs
                     bs-side ?bs-side $?
                     mps ?mps $?
                     order ?order
                     wp ?spawned-wp
             )
  ))
  =>
  (bind ?required-resources ?order ?spawned-wp)
  ;If this order complexity should be produced exclusively ...
  (if (any-factp ((?exclusive-complexities wm-fact))
        (and (wm-key-prefix ?exclusive-complexities:key (create$ config rcll exclusive-complexities))
             (neq FALSE (member$ (str-cat ?complexity) ?exclusive-complexities:values))))
    then
      ;... then an exclusive order token is required.
      (bind ?required-resources ?mps ?order ?spawned-wp PRODUCE-EXCLUSIVE-COMPLEXITY)
      (printout t "Goal " PRODUCE-C0 " formulated, it needs the PRODUCE-EXCLUSIVE-COMPLEXITY token" crlf)
    else
      (printout t "Goal " PRODUCE-C0 " formulated" crlf))
  (bind ?parent ?production-id)
  (bind ?priority-decrease 0)
  (if (and (eq ?comp-prio "HIGH") ?competitive)
    then
     (bind ?parent ?urgent))
  (if (eq ?comp-prio "LOW")
    then
      (bind ?priority-decrease 1))
  (bind ?distance (node-distance (str-cat ?bs - (if (eq ?bs-side INPUT) then I else O))))
  (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
                (class PRODUCE-C0) (sub-type SIMPLE)
                (priority (+ (- ?*PRIORITY-PRODUCE-C0* ?priority-decrease) (goal-distance-prio ?distance)))
                (parent ?parent)
                (params robot ?robot
                        bs ?bs
                        bs-side ?bs-side
                        bs-color ?base-color
                        mps ?mps
                        cs-color ?cap-color
                        order ?order
                        wp ?spawned-wp
                )
                (required-resources (sym-cat ?mps -INPUT) ?required-resources)
  ))
)


(defrule goal-production-create-mount-first-ring
" Start a higher order product by getting the base and mounting the first ring.
  The workpiece remains in the output of the used ring station after
  successfully finishing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))

  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args?         r ?robot))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args?       m ?mps-rs t RS))
  (wm-fact (key domain fact mps-state args?      m ?mps-rs s ~BROKEN))
  (wm-fact (key domain fact mps-team args?       m ?mps-rs col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps-rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps-rs r ?ring1-color&~RING_NONE rn ?bases-needed))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
  (not (wm-fact (key domain fact rs-prepared-color args?  m ?mps-rs col ?some-col)))
  (not (wm-fact (key domain fact wp-at args? wp ?wp-rs m ?mps-rs side INPUT)))
  (wm-fact (key domain fact mps-type args? m ?other-rs&~?mps-rs t RS))
  (wm-fact (key domain fact mps-team args? m ?other-rs col ?team-color))
  ; There is at least one other rs side, except for the target input, that
  ; is free (because occupying all 4 sides at once can cause deadlocks)
  (or (wm-fact (key domain fact mps-side-free args? m ?mps-rs side OUTPUT))
      (wm-fact (key domain fact mps-side-free args? m ?other-mps side ?any-side)))
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args?  m ?mps-bs t BS))
  (wm-fact (key domain fact mps-team args?  m ?mps-bs col ?team-color))
  (domain-object (name ?bs-side&:(or (eq ?bs-side INPUT) (eq ?bs-side OUTPUT))) (type mps-side))
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color) (value ?qd&:(> ?qr ?qd)))
  ;Active Order CEs
  ;No one started this order already
  (or (and (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
           (wm-fact (key domain fact mps-state args? m ?mps-bs s ~BROKEN&~DOWN))
           (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
           (not (wm-fact (key order meta wp-for-order args? wp ?any-ord-wp ord ?order))))
      (and (wm-fact (key domain fact holding args? r ?robot wp ?spawned-wp))
           (wm-fact (key domain fact wp-base-color args? wp ?spawned-wp col ?base-color))
           (wm-fact (key order meta wp-for-order args? wp ?spawned-wp ord ?order))
           (wm-fact (key domain fact wp-ring1-color args? wp ?spawned-wp col RING_NONE))))
  ;This order complexity is not produced exclusively while another exclusive
  ;complexity order is already started
  (not (and (wm-fact (key order meta wp-for-order args? wp ?ord-wp&~?spawned-wp ord ?any-order))
            (wm-fact (key domain fact order-complexity args? ord ?any-order com ?other-complexity))
            (wm-fact (key config rcll exclusive-complexities) (values $?other-exclusive&:(member$ (str-cat ?other-complexity) ?other-exclusive)))
            (wm-fact (key config rcll exclusive-complexities) (values $?exclusive&:(member$ (str-cat ?complexity) ?exclusive)))))
  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  (test (neq ?complexity C0))
  ; Strategy CEs
  (not (goal (class MOUNT-FIRST-RING)
             (parent ?production-id)
             (params robot ?robot $?
                     bs-side ?bs-side $?
                     order ?order
                     wp ?spawned-wp)))
  (not (wm-fact (key strategy keep-mps-side-free args? m ?mps-rs side INPUT $?)))
  =>
  (bind ?required-resources ?order ?spawned-wp)
  ;If this order complexity should be produced exclusively ...
  (if (any-factp ((?exclusive-complexities wm-fact))
        (and (wm-key-prefix ?exclusive-complexities:key (create$ config rcll exclusive-complexities))
             (neq FALSE (member$ (str-cat ?complexity) ?exclusive-complexities:values))))
    then
      ;... then an exclusive order token is required.
      (bind ?required-resources ?mps-rs ?order ?spawned-wp PRODUCE-EXCLUSIVE-COMPLEXITY)
      (printout t "Goal " MOUNT-FIRST-RING " formulated, it needs the PRODUCE-EXCLUSIVE-COMPLEXITY token" crlf)
    else
      (printout t "Goal " MOUNT-FIRST-RING " formulated" crlf))
  (bind ?distance (node-distance (str-cat ?mps-bs - (if (eq ?bs-side INPUT) then I else O))))
  (assert (goal (id (sym-cat MOUNT-FIRST-RING- (gensym*)))
                (class MOUNT-FIRST-RING) (sub-type SIMPLE)
                (priority (+ ?*PRIORITY-MOUNT-FIRST-RING* (goal-distance-prio ?distance)))
                (parent ?production-id)
                (params robot ?robot
                        bs ?mps-bs
                        bs-side ?bs-side
                        bs-color ?base-color
                        mps ?mps-rs
                        ring-color ?ring1-color
                        rs-before ?bases-filled
                        rs-after ?bases-remain
                        rs-req ?bases-needed
                        order ?order
                        wp ?spawned-wp
                )
                (required-resources (sym-cat ?mps-rs -INPUT) ?required-resources)
  ))
)


;TODO: Do we need this?
(deffunction trac-ring-mount-time (?complexity ?rings)
  "Determine time to mount the remaining rings plus cap"
  (bind ?max-rings (eval (sub-string 2 3 (str-cat ?complexity))))
  (return (+ (* (- ?max-rings ?rings) ?*PRODUCE-RING-AHEAD-TIME*) ?*PRODUCE-CAP-AHEAD-TIME*))
)


(defrule goal-production-create-mount-next-ring
" Mount the next ring on a CX product:
   - Take the started workpiece from the ring station output.
   - Bring it to the ring station that can mount the next ring.
  The workpiece remains in the output of the used ring station after
  successfully finishing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class INTERMEDEATE-STEPS) (id ?maintain-id) (mode FORMULATED))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args?         r ?robot))

  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args?       m ?mps-rs t RS))
  (wm-fact (key domain fact mps-state args?      m ?mps-rs s ~BROKEN))
  (wm-fact (key domain fact mps-team args?       m ?mps-rs col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps-rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps-rs r ?next-ring-color&~RING_NONE rn ?bases-needed))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
  (not (wm-fact (key domain fact rs-prepared-color args?  m ?mps-rs col ?some-col)))

  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&C2|C3))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?order-ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?order-ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?order-ring3-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color) (value ?qd&:(> ?qr ?qd)))
  ;WP CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))

  ;The workpiece misses a ring
  (test (or
            (and (eq ?wp-ring1-color ?order-ring1-color)
                 (eq ?wp-ring2-color ?order-ring2-color)
                 (neq ?wp-ring3-color ?order-ring3-color)
                 (eq ?next-ring-color ?order-ring3-color))
            (and (eq ?wp-ring1-color ?order-ring1-color)
                 (neq ?wp-ring2-color ?order-ring2-color)
                 (eq ?next-ring-color ?order-ring2-color))))
  (or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
           (wm-fact (key domain fact wp-at args? wp ?wp m ?prev-rs side OUTPUT)))
      (and (wm-fact (key domain fact holding args? r ?robot wp ?wp))
           (wm-fact (key domain fact mps-type args? m ?prev-rs t RS))))
  (not (wm-fact (key domain fact wp-at args? wp ?wp-rs&:(neq ?wp-rs ?wp) m ?mps-rs side INPUT)))
  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  ; Strategy CEs
  (not (wm-fact (key strategy keep-mps-side-free
                 args? m ?mps-rs side INPUT cause ~?wp)))
  (not (goal (class MOUNT-NEXT-RING)
             (parent ?maintain-id)
             (params robot ?robot $?
                     wp ?wp $?
                     order ?order)))
  =>
  (bind ?ring-pos (member$ RING_NONE (create$ ?wp-ring1-color ?wp-ring2-color ?wp-ring3-color)))
  (bind ?curr-ring-color (nth$ ?ring-pos (create$ ?order-ring1-color ?order-ring2-color ?order-ring3-color)))
  (printout t "Goal " MOUNT-NEXT-RING " formulated (Ring " ?ring-pos")" crlf)
  (assert (goal (id (sym-cat MOUNT-NEXT-RING- (gensym*)))
                (class MOUNT-NEXT-RING) (priority (+ ?ring-pos ?*PRIORITY-MOUNT-NEXT-RING*))
                (parent ?maintain-id) (sub-type SIMPLE)
                (params robot ?robot
                        prev-rs ?prev-rs
                        prev-rs-side OUTPUT
                        wp ?wp
                        rs ?mps-rs
                        ring1-color ?order-ring1-color
                        ring2-color ?order-ring2-color
                        ring3-color ?order-ring3-color
                        curr-ring-color ?curr-ring-color
                        ring-pos (int-to-sym ?ring-pos)
                        rs-before ?bases-filled
                        rs-after ?bases-remain
                        rs-req ?bases-needed
                        order ?order
                )
                (required-resources (sym-cat ?mps-rs -INPUT) (sym-cat ?prev-rs -OUTPUT) ?wp)
  ))
)


(defrule goal-production-create-produce-c1
" Produce a C1 product: Get the workpiece with the mounted ring and mount
  a cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.

  Note that the produce-c1, produce-c2, produce-c3 goal creation is
  deliberately split into separate rules. This is done for readability and
  to leave the option open to customize the strategy for CX products in
  the future.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?maintain-id) (class INTERMEDEATE-STEPS) (mode FORMULATED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side INPUT)))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;WP CEs
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  ;Order CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com C1))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd&:(> ?qr ?qd)))
  (or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
           (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side OUTPUT)))
      (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
  (not (goal (class PRODUCE-CX)
             (parent ?maintain-id)
             (params robot ?robot
                     wp ?wp $?
                     mps ?mps $?
                     order ?order)))
  =>
  (printout t "Goal " PRODUCE-CX " formulated" crlf)
  (assert (goal (id (sym-cat PRODUCE-CX- (gensym*))) (class PRODUCE-CX)
                (priority ?*PRIORITY-PRODUCE-C1*) (sub-type SIMPLE)
                                (parent ?maintain-id)
                                (params robot ?robot
                                        wp ?wp
                                        rs ?rs
                                        mps ?mps
                                        cs-color ?cap-color
                                        order ?order
                                )
                                (required-resources (sym-cat ?mps -INPUT) (sym-cat ?rs -OUTPUT) ?wp)
  ))
)


(defrule goal-production-create-produce-c2
" Produce a C2 product: Get the workpiece with the mounted ring and mount
  a cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class INTERMEDEATE-STEPS) (id ?maintain-id) (mode FORMULATED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side INPUT)))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;WP CEs
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  ;Order CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com C2))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd&:(> ?qr ?qd)))
  (or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
           (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side OUTPUT)))
      (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
  (not (goal (class PRODUCE-CX)
             (parent ?maintain-id)
             (params robot ?robot
                     wp ?wp $?
                     mps ?mps $?
                     order ?order)))
  =>
  (printout t "Goal " PRODUCE-CX " (C2) formulated" crlf)
  (assert (goal (id (sym-cat PRODUCE-CX- (gensym*))) (class PRODUCE-CX)
                (priority ?*PRIORITY-PRODUCE-C2*) (sub-type SIMPLE)
                (parent ?maintain-id)
                (params robot ?robot
                        wp ?wp
                        rs ?rs
                        mps ?mps
                        cs-color ?cap-color
                        order ?order
                )
                (required-resources (sym-cat ?mps -INPUT) (sym-cat ?rs -OUTPUT) ?wp)
  ))
)


(defrule goal-production-create-produce-c3
" Produce a C3 product: Get the workpiece with the mounted ring and mount
  a cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class INTERMEDEATE-STEPS) (id ?maintain-id) (mode FORMULATED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side INPUT)))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;WP CEs
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  ;Order CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com C3))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
  (value ?qd&:(> ?qr ?qd)))
  (or (and (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
           (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side OUTPUT)))
      (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
  (not (goal (class PRODUCE-CX)
                 (parent ?maintain-id)
                 (params robot ?robot
                         wp ?wp $?
                         mps ?mps $?
                         order ?order)))
  =>
  (printout t "Goal " PRODUCE-CX " (C3) formulated" crlf)
  (assert (goal (id (sym-cat PRODUCE-CX- (gensym*))) (class PRODUCE-CX)
                (priority ?*PRIORITY-PRODUCE-C3*) (sub-type SIMPLE)
                (parent ?maintain-id)
                (params robot ?robot
                        wp ?wp
                        rs ?rs
                        mps ?mps
                        cs-color ?cap-color
                        order ?order
                )
                (required-resources (sym-cat ?mps -INPUT) (sym-cat ?rs -OUTPUT) ?wp)
  ))
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
  (retract ?t)
)


(defrule goal-production-create-discard-failed-put-slide
" Discard the currently held workpiece after filling it to a ring station
  failed too often
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class NO-PROGRESS) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;To-Do: Model state IDLE
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  ?t <- (wm-fact (key monitoring action-retried args? r ?self a wp-put-slide-cc m ?mps wp ?wp)
                (value ?tried&:(>= ?tried ?*MAX-RETRIES-PICK*)))

  =>
  (printout t "Goal " DISCARD-UNKNOWN " formulated" crlf)
 (assert (goal (id (sym-cat DISCARD-UNKNOWN- (gensym*)))
                (class DISCARD-UNKNOWN) (sub-type SIMPLE)
                (priority ?*PRIORITY-RESET*)
                (parent ?production-id)
                (params robot ?robot
                        wp ?wp
                )
                (required-resources ?wp)
  ))
  (retract ?t)
)


(defrule goal-production-create-deliver
  "Deliver a fully produced workpiece."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class DELIVER-PRODUCTS) (mode FORMULATED))
  (goal (id ?urgent) (class URGENT) (mode FORMULATED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS-DS CEs
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  ;MPS-CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;WP-CEs
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  ;Order-CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  ;note: could be moved to rejected checks
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd&:(> ?qr ?qd)))
  (wm-fact (key refbox order ?order delivery-begin) (type UINT)
           (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*))))
  (or (and (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
           (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp))))
      (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
  (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))
  (wm-fact (key config rcll competitive-order-priority) (value ?comp-prio))
  (not (goal (class DELIVER)
             (parent ?parent)
             (params robot ?robot $?
                     order ?order
                     wp ?wp
                     ds ?ds
                     ds-gate ?gate $?)))
  =>
  (printout t "Goal " DELIVER " formulated" crlf)
  (bind ?parent ?production-id)
  (bind ?priority-decrease 0)
  (if (and (eq ?comp-prio "HIGH") ?competitive)
    then
      (bind ?parent ?urgent))
  (if (eq ?comp-prio "LOW")
    then
      (bind ?priority-decrease 1))
  (assert (goal (id (sym-cat DELIVER- (gensym*)))
                (class DELIVER) (sub-type SIMPLE)
                (priority (- ?*PRIORITY-DELIVER* ?priority-decrease))
                (parent ?parent)
                (params robot ?robot
                        mps ?mps
                        order ?order
                        wp ?wp
                        ds ?ds
                        ds-gate ?gate
                        base-color ?base-color
                        ring1-color ?ring1-color
                        ring2-color ?ring2-color
                        ring3-color ?ring3-color
                        cap-color ?cap-color
                )
                (required-resources (sym-cat ?mps -OUTPUT) ?order ?wp)
  ))
)

(defrule goal-production-wait-for-mps-processing
" If a mps is ready to process (IDLE and not wp at input) drive to output
  and wait for this mps
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class WAIT-FOR-PROCESS) (mode FORMULATED))

  (not (wm-fact (key domain fact holding args? r ?robot wp ?)))

  (wm-fact (key domain fact self args? r ?robot))
  (or (and (wm-fact (key domain fact at args? r ?robot m ?mps side WAIT))
           (domain-object (type waitpoint) (name ?waitpoint&:(and (eq ?mps ?waitpoint) (eq (str-length (str-cat ?waitpoint)) 10)))))
      (and (domain-object (type waitpoint) (name ?waitpoint&:(eq (str-length (str-cat ?waitpoint)) 10)))
           (wm-fact (key domain fact at args? r ?robot m ?mps side ?side))
           (not (domain-object (type waitpoint) (name ?w2&:(and (eq ?w2 ?mps) (eq (str-length (str-cat ?w2)) 10))))))
  )
  =>
  (assert
    (goal (id (sym-cat WAIT-FOR-MPS-PROCESS- (gensym*)))
          (class WAIT-FOR-MPS-PROCESS)
          (sub-type SIMPLE)
          (parent ?production-id)
          (priority ?*PRIORITY-WAIT-MPS-PROCESS*)
          (params robot ?robot
                  pos ?waitpoint)
          (required-resources WAIT-PROCESS ?waitpoint)
    )
  )
)


(defrule goal-production-mps-handling-create-prepare-goal
  "Prepare and model processing of a mps"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?pg <- (goal (id ?mps-handling-id) (class MPS-HANDLING-MAINTAIN) (mode SELECTED))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;Requested process CEs
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
  ?wp-for-order <- (wm-fact (key order meta wp-for-order
                                 args? wp ?wp ord ?order)
                            (value TRUE))
  (not (wm-fact (key domain fact wp-usable args? wp ?wp)))
  =>
  (retract ?wp-for-order)
  (delayed-do-for-all-facts ((?wm wm-fact))
    (and (wm-key-prefix ?wm:key (create$ wp meta))
         (eq (wm-key-arg ?wm:key wp) ?wp))
    (retract ?wm)
  )
  (printout debug "WP " ?wp " no longer tied to Order " ?order " because it is
    not usable anymore" crlf)
)
