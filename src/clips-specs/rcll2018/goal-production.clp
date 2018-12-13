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
  ; Number of retrying enter-field
  ; until succeeding it manually
  ?*ENTER-FIELD-RETRIES* = 1
  ?*MAX-RETRIES-PICK* = 2
  ?*MAX-RETRIES-PUT-SLIDE* = 2


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
  ?*PRIORITY-PREFILL-RS* = 40 ;This priority can be increased by up to +2
  ?*PRIORITY-PREFILL-RS-WITH-FRESH-BASE* = 30
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

	?*ENTER-FIELD-RETRIES* = 3
)


(defrule goal-reasoner-create-acquire-token-spawning-master
" If no one is spawning master. Try to become the spawning master

  The spawning master creates the facts for new workpieces anyd capcarriers.
  Those can be introduced to the world during the game e.g. through dispensing
  at the base station or refilling of a shelf.
"
  (domain-facts-loaded)
  (domain-object (name SPAWNING-MASTER) (type master-token))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (not (goal (class ACQUIRE-TOKEN) (params token-name SPAWNING-MASTER)))
  (not (mutex (name SPAWNING-MASTER) (state LOCKED)))
  =>
  (assert (goal (id (sym-cat ACQUIRE-TOKEN- (gensym*)))
                    (class ACQUIRE-TOKEN)
		    (params token-name SPAWNING-MASTER)))
)


(defrule goal-reasoner-create-beacon-maintain
" The parent goal for beacon signals. Allows formulation of
  goals that periodically communicate with the refbox.
"
  (not (goal (class BEACONMAINTAIN)))
  =>
  (assert (goal (id (sym-cat BEACONMAINTAIN- (gensym*)))
                (class BEACONMAINTAIN) (type MAINTAIN)))
)


(defrule goal-reasoner-create-beacon-achieve
" Send a beacon signal whenever at least one second has elapsed since it
  last one got sent.
"
  ?g <- (goal (id ?maintain-id) (class BEACONMAINTAIN) (mode SELECTED))
  (not (goal (class BEACONACHIEVE)))
  (time $?now)
  ; TODO: make interval a constant
  (goal-meta (goal-id ?maintain-id)
             (last-achieve $?last&:(timeout ?now ?last 1)))
  =>
  (assert (goal (id (sym-cat BEACONACHIEVE- (gensym*)))
                (class BEACONACHIEVE) (parent ?maintain-id)))
)


(defrule goal-reasoner-create-wp-spawn-maintain
  "Maintain Spawning if the spawning-master token is held"
  (domain-facts-loaded)
 (not (goal (class WPSPAWN-MAINTAIN)))
 (mutex (name SPAWNING-MASTER) (state LOCKED) (locked-by ?locked-by))
 (wm-fact (key domain fact self args? r ?self&:(eq ?self (sym-cat ?locked-by))))
 (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
 =>
 (assert (goal (id (sym-cat WPSPAWN-MAINTAIN- (gensym*)))
               (class WPSPAWN-MAINTAIN) (type MAINTAIN)))
)


(defrule goal-reasoner-create-wp-spawn-achieve
  "Spawn a WP for each robot, if you are the spawn-master"
  ?g <- (goal (id ?maintain-id) (class WPSPAWN-MAINTAIN) (mode SELECTED))
  (not (goal (class WPSPAWN-ACHIEVE)))
  (time $?now)
  ; TODO: make interval a constant
  (goal-meta (goal-id ?maintain-id)
             (last-achieve $?last&:(timeout ?now ?last 1)))
  (domain-object (name ?robot) (type robot))
  (not
    (and
    (domain-object (name ?wp) (type workpiece))
    (wm-fact (key domain fact wp-spawned-for args? wp ?wp r ?robot)))
  )
  (mutex (name SPAWNING-MASTER) (state LOCKED) (locked-by ?locked-by))
  (wm-fact (key domain fact self args? r ?self&:(eq ?self (sym-cat ?locked-by))))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  =>
  (assert (goal (id (sym-cat WPSPAWN-ACHIEVE- (gensym*)))
                (class WPSPAWN-ACHIEVE) (parent ?maintain-id)
                (params robot ?robot)))
)


(defrule goal-reasoner-create-refill-shelf-maintain
" The parent goal to refill a shelf. Allows formulation of goals to refill
  a shelf only if the game is in the production phase and the domain is loaded.
  Only the spawning-master is in charge of handling shelf refills.
"
  (domain-facts-loaded)
  (not (goal (class REFILL-SHELF-MAINTAIN)))
  (mutex (name SPAWNING-MASTER) (state LOCKED) (locked-by ?locked-by))
  (wm-fact (key domain fact self args? r ?self&:(eq ?self (sym-cat ?locked-by))))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  =>
  (assert (goal (id (sym-cat REFILL-SHELF-MAINTAIN- (gensym*)))
                (class REFILL-SHELF-MAINTAIN) (type MAINTAIN)))
)


(defrule goal-reasoner-create-refill-shelf-achieve
  "Refill a shelf whenever it is empty."
  ?g <- (goal (id ?maintain-id) (class REFILL-SHELF-MAINTAIN) (mode SELECTED))
  (not (goal (class REFILL-SHELF-ACHIEVE)))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  (mutex (name SPAWNING-MASTER) (state LOCKED) (locked-by ?locked-by))
  (wm-fact (key domain fact self args? r ?self&:(eq ?self (sym-cat ?locked-by))))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  =>
  (assert (goal (id (sym-cat REFILL-SHELF-ACHIEVE- (gensym*)))
                (class REFILL-SHELF-ACHIEVE)
                (parent ?maintain-id)
                (params mps ?mps)))
)


(defrule goal-reasoner-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (final TRUE))
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name (sym-cat ?waitzone:name)) (type waitpoint))
      (domain-fact (name location-free) (param-values (sym-cat ?waitzone:name) WAIT))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
)


(defrule goal-reasoner-create-goal-production-maintain
" The parent production goal. Allows formulation of
  production goals only if the proper game state selected
  and the domain got loaded. Other production goals are
  formulated as sub-goals of this goal.
"
  (domain-facts-loaded)
  (not (goal (class PRODUCTION-MAINTAIN)))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (NavGraphWithMPSGeneratorInterface (final TRUE))
  (wm-fact (key navgraph waitzone generated))
  =>
  (assert (goal (id (sym-cat PRODUCTION-MAINTAIN- (gensym*)))
                (class PRODUCTION-MAINTAIN) (type MAINTAIN)))
)


(defrule goal-reasoner-create-wait
  "Keep waiting at one of the waiting positions."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key domain fact self args? r ?self))
  (domain-object (type waitpoint) (name ?waitpoint))
  (wm-fact (key domain fact at args? r ?self m ?waitpoint side WAIT))
  =>
  (printout t "Goal " WAIT " formulated" crlf)
  (assert (goal (id (sym-cat WAIT- (gensym*)))
               (class WAIT)
               (priority ?*PRIORITY-WAIT*)
               (parent ?production-id)
               (params r ?self
                       point ?waitpoint)
               (required-resources ?waitpoint)
  ))
)


(defrule goal-reasoner-create-go-wait
  "Drive to a waiting position and wait there."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key domain fact self args? r ?self))
  (domain-object (type waitpoint) (name ?waitpoint))
  (domain-fact (name location-free) (param-values ?waitpoint WAIT))
  =>
  (printout t "Goal " GO-WAIT " formulated" crlf)
  (assert (goal (id (sym-cat GO-WAIT- (gensym*)))
                (class GO-WAIT)
                (priority  ?*PRIORITY-GO-WAIT*)
                (parent ?production-id)
                (params r ?self
                        point ?waitpoint
                )
                (required-resources ?waitpoint)
  ))
)


(defrule goal-reasoner-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class ENTER-FIELD-ACHIEVE)))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  ; (NavGraphGeneratorInterface (final TRUE))
  ; (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (assert (goal (id (sym-cat ENTER-FIELD-ACHIEVE- (gensym*)))
                (class ENTER-FIELD-ACHIEVE)))
)


(defrule goal-reasoner-create-fill-cap
" Fill a cap into a cap station.
  Use a capcarrier from the corresponding shelf to feed it into a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
  ;MPS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~DOWN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?any-cap-color)))
  (not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side ?any-side)))
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
                        (and (wm-key-prefix ?wp-for-order:key (create$ evaluated fact wp-for-order))
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
  (assert (goal (id (sym-cat FILL-CAP- (gensym*)))
                (class FILL-CAP)
                (priority (+ ?priority-increase ?*PRIORITY-PREFILL-CS*))
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        cc ?cc
                )
                (required-resources ?mps)
  ))
)


(defrule goal-reasoner-create-clear-rs-from-expired-product
  "Remove an unfinished product from the output of a ring station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class PRODUCTION-MAINTAIN) (id ?maintain-id) (mode SELECTED))
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
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))

  ;TODO: Discuss strategy, throwing away expired products is usually not desired.
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
    (value ?end&:(< ?end (nth$ 1 ?game-time))))
  =>
  (printout t "Goal " CLEAR-MPS " ("?mps") formulated" crlf)
  (assert (goal (id (sym-cat CLEAR-MPS- (gensym*))) (class CLEAR-MPS)
                (priority ?*PRIORITY-CLEAR-RS*)
                (parent ?maintain-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side OUTPUT
                )
                (required-resources ?wp)
  ))
)


(defrule goal-reasoner-create-clear-cs-for-capless-carriers
" Remove a capless capcarrier from the output of a cap station after
  retrieving a cap from it.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
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
  (assert (goal (id (sym-cat CLEAR-MPS- (gensym*)))
                (class CLEAR-MPS)
                (priority ?*PRIORITY-CLEAR-CS*)
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side OUTPUT
                )
                (required-resources ?wp)
  ))
)


(defrule goal-reasoner-create-clear-bs
  "Remove a workpiece from the base station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
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
                (class CLEAR-MPS)
                (priority ?*PRIORITY-CLEAR-BS*)
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side ?side
                )
                (required-resources ?mps ?wp)
  ))
)


(defrule goal-reasoner-clear-cs-from-expired-product
  "Remove a finished product from a cap station after it's deadline passed."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
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
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))

  (wm-fact (key refbox order ?order delivery-end) (type UINT)
           (value ?end&:(< ?end (nth$ 1 ?game-time))))
  =>
  (printout t "Goal " CLEAR-MPS " (" ?mps ") formulated" crlf)
  (assert (goal (id (sym-cat CLEAR-MPS- (gensym*)))
                (class CLEAR-MPS)
                (priority ?*PRIORITY-CLEAR-CS*)
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side OUTPUT
                )
                (required-resources ?wp)
  ))
)


(defrule goal-reasoner-increase-priority-to-prefill-rs-for-started-order
" Add a priority increase of +2 for goals that pre-fill a ring station which
  requires additional bases such that the production of a started product
  can be continued.
"
  ;Compute the priorities before goals get formulated.
  (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))
  (goal (class PRODUCTION-MAINTAIN) (id ?maintain-id) (mode SELECTED))
  (not (goal (class FILL-RS|FILL-RS-FROM-BS) (mode FORMULATED)))
  ;MPS CEs
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  ;The MPS can mount a ring which needs more bases than currently available.
  (wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring-color rn ?ring-num&:(neq ?rs-before ?ring-num)))
  (wm-fact (key domain fact rs-sub args? minuend ?ring-num subtrahend ?rs-before difference ?rs-diff))

  ;(TODO: make the mps-state  a precond of the put-slide to save traviling time)

  (wm-fact (key evaluated fact wp-for-order args? wp ?order-wp ord ?order))
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
  (printout warn "RS " ?mps " needs an additional base for " ?order " given " ?order-wp " (prio +2)" crlf)
  (if (not (do-for-fact
            ((?wmf wm-fact))
            (eq ?wmf:key (create$ evaluated fact rs-fill-priority args? m ?mps))
            (modify ?wmf (value 2))))
    then
      (assert
        (wm-fact (key evaluated fact rs-fill-priority args? m ?mps) (value 2)))
  )
)


(defrule goal-reasoner-increase-priority-to-prefill-rs-for-unstarted-order
" Add a priority increase of +1 for goals that pre-fill a ring station which
  requires additional bases such that an available order (that was not started
  yet) can use them for the first ring.
  The second and third rings of available orders are not considered as
  priority increases for those can be computed once the order is started.
"
  ;compute the priorities before goals get formulated.
  (declare (salience (+ 1 ?*SALIENCE-GOAL-FORMULATE*)))

  (goal (id ?maintain-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (not (goal (class FILL-RS|FILL-RS-FROM-BS) (mode FORMULATED)))
  ;MPS CEs
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  ;The MPS can mount a ring which needs more bases than currently available.
  (wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring1-color rn ?ring-num))
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
  (printout warn "RS " ?mps " needs an additional base for " ?order " (prio +1)" crlf)
  (if (not (do-for-fact
            ((?wmf wm-fact))
            (eq ?wmf:key (create$ evaluated fact rs-fill-priority args? m ?mps))
      (modify ?wmf (value 1))))
    then
      (assert
        (wm-fact (key evaluated fact rs-fill-priority args? m ?mps) (value 1)))
  )
)


(defrule goal-reasoner-create-prefill-ring-station-from-base-station
  "Fill the ring station with a fresh base from the base station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?maintain-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~DOWN&~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (not (wm-fact (key domain fact wp-at args? wp ?some-wp m ?bs side ?any-side)))
  (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN))
  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))

  (wm-fact (key domain fact order-base-color args? ord ?any-order col ?base-color))
  =>
  ;Check if this ring station should be filled with increased priority.
  (bind ?priority-increase 0)
  (do-for-all-facts ((?prio wm-fact)) (and (wm-key-prefix ?prio:key (create$ evaluated fact rs-fill-priority))
                                        (eq (wm-key-arg ?prio:key m) ?mps))
      (if (< ?priority-increase ?prio:value)
         then
          (bind ?priority-increase ?prio:value)
      )
  )
  (printout warn "Goal " FILL-RS-FROM-BS " formulated" crlf)
  (assert (goal (id (sym-cat FILL-RS-FROM-BS- (gensym*)))
                (class FILL-RS-FROM-BS)
                (priority (+ ?priority-increase ?*PRIORITY-PREFILL-RS-WITH-FRESH-BASE*))
                (parent ?maintain-id)
                             (params robot ?robot
                                     mps ?mps
                                     bs ?bs
                                     bs-side INPUT
                                     base-color ?base-color
                                     rs-before ?rs-before
                                     rs-after ?rs-after
                                     wp ?spawned-wp
                                     )
                            (required-resources ?mps ?spawned-wp)
  ))
)


(defrule goal-reasoner-create-prefill-ring-station-from-shelf
  "Fill a ring station with a capcarrier from a shelf."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?maintain-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~DOWN&~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?cs spot ?spot))
  =>
  ;Check if this ring station should be filled with increased priority.
  (bind ?priority-increase 0)
  (do-for-all-facts ((?prio wm-fact)) (and (wm-key-prefix ?prio:key (create$ evaluated fact rs-fill-priority))
                                           (eq (wm-key-arg ?prio:key m) ?mps))
      (if (< ?priority-increase ?prio:value)
         then
          (bind ?priority-increase ?prio:value)
      ))
  (printout warn "Goal " FILL-RS-FROM-SHELF " formulated" crlf)
  (assert (goal (id (sym-cat FILL-RS-FROM-SHELF- (gensym*)))
                (class FILL-RS-FROM-SHELF)
                (priority (+ ?priority-increase ?*PRIORITY-PREFILL-RS*))
                (parent ?maintain-id)
                             (params robot ?robot
                                     mps ?mps
                                     cs ?cs
                                     wp ?wp
                                     spot ?spot
                                     rs-before ?rs-before
                                     rs-after ?rs-after
                                     )
                             (required-resources ?mps)
  ))
)


(defrule goal-reasoner-create-prefill-ring-station
  ;Fill a ring station with the currently holding workpiece.
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact wp-usable args? wp ?wp))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~DOWN&~BROKEN))
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
  (printout t "Goal " FILL-RS " formulated" crlf)
  (assert (goal (id (sym-cat FILL-RS- (gensym*)))
                (class FILL-RS)
                (priority (+ ?*PRIORITY-PREFILL-RS* ?priority-increase))
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


(defrule goal-reasoner-create-discard-unknown
  "Discard a base which is not needed."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
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
                (class DISCARD-UNKNOWN)
                (priority ?*PRIORITY-DISCARD-UNKNOWN*)
                (parent ?production-id)
                (params robot ?robot
                        wp ?wp
                )
                (required-resources ?wp)
  ))
  ; (assert (goal-already-tried DISCARD-UNKNOWN))
)


(defrule goal-reasoner-create-produce-c0
" Produce a C0 product: Get the correct base and mount the right cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side ?any-side)))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (domain-object (name ?bs-side) (type mps-side))
  (domain-fact (name location-free) (param-values ?bs ?bs-siide))
  (not (wm-fact (key domain fact wp-at args? wp ?some-wp m ?bs side ?any-side)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN))
  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  ;To-Do: Model the bs active-side
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
	(value ?qd&:(> ?qr ?qd)))
  (wm-fact (key refbox order ?order-id delivery-begin) (type UINT)
	(value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-AHEAD-TIME*))))
  (wm-fact (key refbox order ?order-id delivery-end) (type UINT)
	(value ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-C0-LATEST-TIME*))))
  ;Active Order CEs
  ;This order complexity is not produced exclusively while another exclusive
  ;complexity order is already started
  (not (and (wm-fact (key evaluated fact wp-for-order args? wp ?ord-wp ord ?any-order))
            (wm-fact (key domain fact order-complexity args? ord ?any-order com ?other-complexity))
            (wm-fact (key config rcll exclusive-complexities) (values $?other-exclusive&:(member$ (str-cat ?other-complexity) ?other-exclusive)))
            (wm-fact (key config rcll exclusive-complexities) (values $?exclusive&:(member$ (str-cat ?complexity) ?exclusive)))))
  ;No one started this order already
  ;TODO: for multi-agent
  ;	 Model old agents constraints
  ;	 (in-production 0)
  ;	 (in-delivery ?id&:(> ?qr (+ ?qd ?id)))
  (not (wm-fact (key evaluated fact wp-for-order args? wp ?any-ord-wp ord ?order)))

  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  (test (eq ?complexity C0))
  =>
  (bind ?required-resources ?mps ?order ?spawned-wp)
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
  (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
                (class PRODUCE-C0)
                (priority ?*PRIORITY-PRODUCE-C0*)
                (parent ?production-id)
                (params robot ?robot
                        bs ?bs
                        bs-side ?bs-side
                        bs-color ?base-color
                        mps ?mps
                        cs-color ?cap-color
                        order ?order
                        wp ?spawned-wp
                )
                (required-resources ?required-resources)
  ))
)


(defrule goal-reasoner-create-mount-first-ring
" Start a higher order product by getting the base and mounting the first ring.
  The workpiece remains in the output of the used ring station after
  successfully finishing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))

  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args?         r ?robot))
  (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args?       m ?mps-rs t RS))
  (wm-fact (key domain fact mps-state args?      m ?mps-rs s ~BROKEN))
  (wm-fact (key domain fact mps-team args?       m ?mps-rs col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps-rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec args?   m ?mps-rs r ?ring1-color rn ?bases-needed))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
  (not (wm-fact (key domain fact rs-prepared-color args?  m ?mps-rs col ?some-col)))
  (not (wm-fact (key domain fact wp-at args? wp ?wp-rs m ?mps-rs side ?any-rs-side)))
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args?  m ?mps-bs t BS))
  (wm-fact (key domain fact mps-state args? m ?mps-bs s ~BROKEN))
  (wm-fact (key domain fact mps-team args?  m ?mps-bs col ?team-color))
  (not (wm-fact (key domain fact wp-at args? wp ?bs-wp m ?mps-bs side ?any-bs-side)))
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color) (value ?qd&:(> ?qr ?qd)))
  ;Active Order CEs
  ;This order complexity is not produced exclusively while another exclusive
  ;complexity order is already started
  (not (and (wm-fact (key evaluated fact wp-for-order args? wp ?ord-wp ord ?any-order))
            (wm-fact (key domain fact order-complexity args? ord ?any-order com ?other-complexity))
            (wm-fact (key config rcll exclusive-complexities) (values $?other-exclusive&:(member$ (str-cat ?other-complexity) ?other-exclusive)))
            (wm-fact (key config rcll exclusive-complexities) (values $?exclusive&:(member$ (str-cat ?complexity) ?exclusive)))))
  ;No one started this order already
  ;TODO: for multi-agent
  ;	 Model old agents constraints
  ;	 (in-production 0)
  ;	 (in-delivery ?id&:(> ?qr (+ ?qd ?id)))"
  (not (wm-fact (key evaluated fact wp-for-order args? wp ?any-ord-wp ord ?order)))
  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  (test (neq ?complexity C0))
  =>
  (bind ?required-resources ?mps-rs ?order ?spawned-wp)
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
  (assert (goal (id (sym-cat MOUNT-FIRST-RING- (gensym*)))
                (class MOUNT-FIRST-RING)
                (priority ?*PRIORITY-MOUNT-FIRST-RING*)
                (parent ?production-id)
                (params robot ?robot
                        bs ?mps-bs
                        bs-side OUTPUT
                        bs-color ?base-color
                        mps ?mps-rs
                        ring-color ?ring1-color
                        rs-before ?bases-filled
                        rs-after ?bases-remain
                        rs-req ?bases-needed
                        order ?order
                        wp ?spawned-wp
                )
                (required-resources ?required-resources)
  ))
)


;TODO: Do we need this?
(deffunction trac-ring-mount-time (?complexity ?rings)
  "Determine time to mount the remaining rings plus cap"
  (bind ?max-rings (eval (sub-string 2 3 (str-cat ?complexity))))
  (return (+ (* (- ?max-rings ?rings) ?*PRODUCE-RING-AHEAD-TIME*) ?*PRODUCE-CAP-AHEAD-TIME*))
)


(defrule goal-reasoner-create-mount-next-ring
" Mount the next ring on a CX product:
   - Take the started workpiece from the ring station output.
   - Bring it to the ring station that can mount the next ring.
  The workpiece remains in the output of the used ring station after
  successfully finishing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class PRODUCTION-MAINTAIN) (id ?maintain-id) (mode SELECTED))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args?         r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))

  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args?       m ?mps-rs t RS))
  (wm-fact (key domain fact mps-state args?      m ?mps-rs s ~BROKEN))
  (wm-fact (key domain fact mps-team args?       m ?mps-rs col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps-rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec args?   m ?mps-rs r ?next-ring-color rn ?bases-needed))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
  (not (wm-fact (key domain fact rs-prepared-color args?  m ?mps-rs col ?some-col)))
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&C3))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?order-ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?order-ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?order-ring3-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color) (value ?qd&:(> ?qr ?qd)))
  ;WP CEs
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?prev-rs side OUTPUT))
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
                 (eq ?next-ring-color ?order-ring2-color))
            (and (neq ?wp-ring1-color ?order-ring1-color)
                 (eq ?next-ring-color ?order-ring1-color))))
  (not (wm-fact (key domain fact wp-at args? wp ?wp-rs&:(neq ?wp-rs ?wp) m ?mps-rs side ?any-rs-side)))
  (wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  =>
  (bind ?ring-pos (member$ RING_NONE (create$ ?wp-ring1-color ?wp-ring2-color ?wp-ring3-color)))
  (bind ?curr-ring-color (nth$ ?ring-pos (create$ ?order-ring1-color ?order-ring2-color ?order-ring3-color)))
  (printout t "Goal " MOUNT-NEXT-RING " formulated (Ring " ?ring-pos")" crlf)
  (assert (goal (id (sym-cat MOUNT-NEXT-RING- (gensym*)))
                (class MOUNT-NEXT-RING) (priority (+ ?ring-pos ?*PRIORITY-MOUNT-NEXT-RING*))
                (parent ?maintain-id)
                (params robot ?robot
                        prev-rs ?prev-rs
                        prev-rs-side OUTPUT
                        wp ?wp
                        rs ?mps-rs
                        ring1-color ?order-ring1-color
                        ring2-color ?order-ring2-color
                        ring3-color ?order-ring3-color
                        curr-ring-color ?curr-ring-color
                        ring-pos ?ring-pos
                        rs-before ?bases-filled
                        rs-after ?bases-remain
                        rs-req ?bases-needed
                        order ?order
                )
                (required-resources ?wp ?mps-rs)
  ))
)


(defrule goal-reasoner-create-produce-c1
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
  (goal (id ?maintain-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side ?side-cs)))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;WP CEs
  (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side OUTPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  ;Order CEs
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com C1))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd&:(> ?qr ?qd)))
  =>
  (printout t "Goal " PRODUCE-CX " formulated" crlf)
  (assert (goal (id (sym-cat PRODUCE-CX- (gensym*))) (class PRODUCE-CX)
                (priority ?*PRIORITY-PRODUCE-C1*)
                                (parent ?maintain-id)
                                (params robot ?robot
                                        wp ?wp
                                        rs ?rs
                                        mps ?mps
                                        cs-color ?cap-color
                                        order ?order
                                )
                                (required-resources ?mps ?wp)
  ))
)


(defrule goal-reasoner-create-produce-c2
" Produce a C2 product: Get the workpiece with the mounted ring and mount
  a cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class PRODUCTION-MAINTAIN) (id ?maintain-id) (mode SELECTED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side ?side-cs)))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;WP CEs
  (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side OUTPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  ;Order CEs
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
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
  =>
  (printout t "Goal " PRODUCE-CX " (C2) formulated" crlf)
  (assert (goal (id (sym-cat PRODUCE-CX- (gensym*))) (class PRODUCE-CX)
                (priority ?*PRIORITY-PRODUCE-C2*)
                (parent ?maintain-id)
                (params robot ?robot
                        wp ?wp
                        rs ?rs
                        mps ?mps
                        cs-color ?cap-color
                        order ?order
                )
                (required-resources ?mps ?wp)
  ))
)


(defrule goal-reasoner-create-produce-c3
" Produce a C3 product: Get the workpiece with the mounted ring and mount
  a cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (class PRODUCTION-MAINTAIN) (id ?maintain-id) (mode SELECTED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?mps side ?side-cs)))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;WP CEs
  (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side OUTPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  ;Order CEs
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
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
  =>
  (printout t "Goal " PRODUCE-CX " (C3) formulated" crlf)
  (assert (goal (id (sym-cat PRODUCE-CX- (gensym*))) (class PRODUCE-CX)
                (priority ?*PRIORITY-PRODUCE-C3*)
                (parent ?maintain-id)
                (params robot ?robot
                        wp ?wp
                        rs ?rs
                        mps ?mps
                        cs-color ?cap-color
                        order ?order
                )
                (required-resources ?wp ?mps)
  ))
)


(defrule goal-reasoner-create-reset-mps
" Reset an mps to restore a consistent world model after getting a workpiece
  from it failed too often.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key domain fact self args? r ?self))
  ?t <- (wm-fact (key monitoring action-retried args? r ?self a wp-get m ?mps wp ?wp)
                (value ?tried&:(>= ?tried ?*MAX-RETRIES-PICK*)))
  =>
  (printout t "Goal " RESET-MPS " formulated" crlf)
  (assert (goal (id (sym-cat RESET-MPS- (gensym*)))
                (class RESET-MPS) (priority  ?*PRIORITY-RESET*)
                (parent ?production-id)
                (params r ?self
                        m ?mps
                )
                (required-resources ?mps)
  ))
  (retract ?t)
)


(defrule goal-reasoner-create-discard-failed-put-slide
" Discard the currently held workpiece after filling it to a ring station
  failed too often
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
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
                (class DISCARD-UNKNOWN)
                (priority ?*PRIORITY-RESET*)
                (parent ?production-id)
                (params robot ?robot
                        wp ?wp
                )
                (required-resources ?wp)
  ))
  (retract ?t)
)


(defrule goal-reasoner-create-deliver
  "Deliver a fully produced workpiece."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PRODUCTION-MAINTAIN) (mode SELECTED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  ;MPS-DS CEs
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  ;MPS-CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;WP-CEs
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  ;Order-CEs
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
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
  =>
  (printout t "Goal " DELIVER " formulated" crlf)
  (assert (goal (id (sym-cat DELIVER- (gensym*)))
                (class DELIVER)
                (priority ?*PRIORITY-DELIVER*)
                (parent ?production-id)
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
                (required-resources ?order ?wp)
  ))
)


(defrule goal-reasoner-evaluate-failed-enter-field
  "HACK: Stop trying to enter the field when it failed a few times."
   ?g <- (goal (id ?gid) (class ENTER-FIELD-ACHIEVE)
               (mode FINISHED) (outcome FAILED))
  ?pa <- (plan-action (goal-id ?gid) (state FAILED) (action-name enter-field))
  ?gm <- (goal-meta (goal-id ?gid) (num-tries ?num-tries))
  =>
  (printout t "Goal '" ?gid "' has failed, evaluating" crlf)

  (if (= ?num-tries ?*ENTER-FIELD-RETRIES*) then
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(modify ?g (mode DISPATCHED) (outcome UNKNOWN))
        else
	(bind ?num-tries (+ 1 ?num-tries))
	(modify ?gm (num-tries ?num-tries) (max-tries ?*ENTER-FIELD-RETRIES*))
	(modify ?g (mode EVALUATED))
  )
)


(defrule goal-reasoner-evaluate-production-maintain
  "Clean up all rs-fill-priorities facts when the production maintenance goal
   fails."
  ?g <- (goal (id ?goal-id) (class PRODUCTION-MAINTAIN) (parent nil)
              (mode FINISHED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
  ?t <- (wm-fact (key monitoring action-retried args? r ?self a ?an m ?mps wp ?wp)
                 (value ?tried&:(>= ?tried ?*MAX-RETRIES-PICK*)))
  =>
  (printout t "Goal '" ?goal-id "' has been " ?outcome ", evaluating" crlf)
  (retract ?t)
  (do-for-all-facts ((?prio wm-fact)) (wm-key-prefix ?prio:key (create$ evaluated fact rs-fill-priority))
   (retract ?prio))
  (modify ?g (mode EVALUATED))
)


(defrule goal-reasoner-evaluate-completed-produce-c0-and-mount-first-ring
" Bind a workpiece to the order it belongs to.

  Workpieces that got dispensed during PRODUCE-C0 and MOUNT-FIRST-RING get
  tied to their order independent of the goal outcome as long as they are
  still usable.
"
  ?g <- (goal (id ?goal-id) (class PRODUCE-C0|MOUNT-FIRST-RING)
              (parent ?parent-id)
              (mode FINISHED) (outcome ?outcome)
              (params $?params))
 ?gm <- (goal-meta (goal-id ?parent-id))
 (plan (goal-id ?goal-id) (id ?plan-id))
 ?p <-(plan-action
         (plan-id ?plan-id)
         (action-name bs-dispense)
         (param-names r m side wp basecol)
         (param-values ?robot ?bs ?bs-side ?wp ?base-color))
 (time $?now)
 (wm-fact (key domain fact wp-usable args? wp ?wp))
 (wm-fact (key domain fact self args? r ?robot))
 =>
 (bind ?order (get-param-by-arg ?params order))
 (printout t "Goal '" ?goal-id "' has been completed, Evaluating" crlf)
 (assert (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order) (type BOOL) (value TRUE)))
 (modify ?g (mode EVALUATED))
 (modify ?gm (last-achieve ?now))
)


(defrule goal-reasoner-evaluate-completed-subgoal-refill-shelf
" Create the domain objects and wm-facts corresponding to the freshly spawned
  capcarriers when the REFILL-SHELF-ACHIEVE goal finishes successfully.
"
  ?g <- (goal (class REFILL-SHELF-ACHIEVE) (parent ?parent-id)
              (mode FINISHED) (outcome COMPLETED)
              (params mps ?mps))
  ?p <- (goal (id ?parent-id))
  (wm-fact (key domain fact cs-color args? m ?mps col ?col))
  =>
  (if (eq ?col CAP_GREY)
     then
     (bind ?cc1 (sym-cat CCG (random-id)))
     (bind ?cc2 (sym-cat CCG (random-id)))
     (bind ?cc3 (sym-cat CCG (random-id)))
     else
     (bind ?cc1 (sym-cat CCB (random-id)))
     (bind ?cc2 (sym-cat CCB (random-id)))
     (bind ?cc3 (sym-cat CCB (random-id)))
   )
   (assert
     (domain-object (name ?cc1) (type cap-carrier))
     (domain-object (name ?cc2) (type cap-carrier))
     (domain-object (name ?cc3) (type cap-carrier))
     (wm-fact (key domain fact wp-cap-color args? wp ?cc1 col ?col) (type BOOL) (value TRUE))
     (wm-fact (key domain fact wp-cap-color args? wp ?cc2 col ?col) (type BOOL) (value TRUE))
     (wm-fact (key domain fact wp-cap-color args? wp ?cc3 col ?col) (type BOOL) (value TRUE))
     (wm-fact (key domain fact wp-on-shelf args? wp ?cc1 m ?mps spot LEFT) (type BOOL) (value TRUE))
     (wm-fact (key domain fact wp-on-shelf args? wp ?cc2 m ?mps spot MIDDLE) (type BOOL) (value TRUE))
     (wm-fact (key domain fact wp-on-shelf args? wp ?cc3 m ?mps spot RIGHT) (type BOOL) (value TRUE))
   )
   (modify ?g (mode EVALUATED))
)


(defrule goal-reasoner-evaluate-completed-subgoal-wp-spawn
" Create the domain objects and wm-facts corresponding to the freshly spawned
  workpieces when the WP-SPAWN-ACHIEVE goal finishes successfully.
"
  ?g <- (goal (id ?goal-id) (class WPSPAWN-ACHIEVE) (parent ?parent-id)
              (mode FINISHED) (outcome COMPLETED)
              (params robot ?robot))
  ?p <- (goal (id ?parent-id) (class WPSPAWN-MAINTAIN))
  ?m <- (goal-meta (goal-id ?parent-id))
  (time $?now)
  =>
  (printout debug "Goal '" ?goal-id "' (part of '" ?parent-id
    "') has been completed, Evaluating" crlf)
  (bind ?wp-id (sym-cat WP (random-id)))
  (assert
    (domain-object (name ?wp-id) (type workpiece))
    (wm-fact (key domain fact wp-unused args? wp ?wp-id) (type BOOL) (value TRUE))
    (wm-fact (key domain fact wp-cap-color args? wp ?wp-id col CAP_NONE) (type BOOL) (value TRUE))
    (wm-fact (key domain fact wp-ring1-color args? wp ?wp-id col RING_NONE) (type BOOL) (value TRUE))
    (wm-fact (key domain fact wp-ring2-color args? wp ?wp-id col RING_NONE) (type BOOL) (value TRUE))
    (wm-fact (key domain fact wp-ring3-color args? wp ?wp-id col RING_NONE) (type BOOL) (value TRUE))
    (wm-fact (key domain fact wp-base-color args? wp ?wp-id col BASE_NONE) (type BOOL) (value TRUE))
    (wm-fact (key domain fact wp-spawned-for args? wp ?wp-id r ?robot) (type BOOL) (value TRUE))
  )
  (modify ?g (mode EVALUATED))
  (modify ?m (last-achieve ?now))
)


(defrule goal-reasoner-evaluate-cleanup-evaluated-wp-for-order-facts
  "Unbind a workpiece from it's order when it can not be used anymore."
  ?wp-for-order <- (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order) (value TRUE))
  (not (wm-fact (key domain fact wp-usable args? wp ?wp)))
  =>
  (retract ?wp-for-order)
  (printout debug "WP " ?wp " no longer tied to Order " ?order " because it is not usable anymore" crlf)
)
