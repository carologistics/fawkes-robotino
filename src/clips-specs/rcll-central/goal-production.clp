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

; =================== administrative goals (no robot action) ========================

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

; todo (tristan): maybe don't need this (see slack)
(defrule goal-production-create-wp-spawn-maintain
  "Maintain Spawning"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (wm-fact (key refbox phase) (value PRODUCTION))
  (not (goal (class WP-SPAWN-MAINTAIN)))
 =>
  (bind ?goal (goal-tree-assert-run-endless WP-SPAWN-MAINTAIN 1))
  (modify ?goal (required-resources wp-spawn)
                (params frequency 1 retract-on-REJECTED)
                (verbosity QUIET))
)

(defrule goal-production-create-wp-spawn-achieve
  "Spawn a WP for each robot"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (time $?now)
  ?g <- (goal (id ?maintain-id) (class WP-SPAWN-MAINTAIN) (mode SELECTED))
  (not (goal (class SPAWN-WP)))
  (not (goal (class SPAWN-SS-C0)))
  (wm-fact (key central agent robot args? r ?robot))  
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

(defrule goal-production-navgraph-compute-wait-positions-finished
  "Add the waiting points to the domain once their generation is finished."
  (NavGraphWithMPSGeneratorInterface (final TRUE))
  (not (NavGraphWithMPSGeneratorInterface (final ~TRUE)))
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

; ============================= mps handling ===============================

(defrule goal-production-create-mps-handling-maintain
" The parent mps handling goal. Allows formulation of
  mps handling goals, if requested by a production goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (not (goal (class MPS-HANDLING-MAINTAIN)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  =>
  (goal-tree-assert-run-endless MPS-HANDLING-MAINTAIN 1)
)


(defrule goal-production-mps-handling-create-prepare-goal
  "Prepare and model processing of a mps"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  ?pg <- (goal (id ?mps-handling-id) (class MPS-HANDLING-MAINTAIN) (mode SELECTED))
  ;Requested process CEs
  ;Separate conditions apply for delivery stations
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
  ;Requested process CEs
  ;Separate conditions apply for delivery stations
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
                (parent ?mps-handling-id)
                (params m ?mps ord ?order)
                (required-resources ?resources)
  ))
  (modify ?pg (mode EXPANDED))
)

; ============================= Enter-field ===============================

(defrule goal-production-create-enter-field
  "Enter the field (drive outside of the starting box)."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (not (goal (class ENTER-FIELD)))
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE))
  (wm-fact (key refbox team-color) (value ?team-color))
  (NavGraphGeneratorInterface (final TRUE))
  ;Check if robot has entered the field already
  (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD " formulated" crlf)
  (assert (goal (id (sym-cat ENTER-FIELD- (gensym*)))
                (class ENTER-FIELD) (sub-type SIMPLE)
                (params r ?robot team-color ?team-color)))
)

;Diasbled due to the tendency of robot 2 and 3 failing to enter
;(defrule goal-production-hack-failed-enter-field
;  "HACK: Stop trying to enter the field when it failed a few times."
  ; TODO-GM: this was after 3 tries, now its instantly
;  ?g <- (goal (id ?gid) (class ENTER-FIELD)
;               (mode FINISHED) (outcome FAILED))
;  ?pa <- (plan-action (goal-id ?gid) (state FAILED) (action-name enter-field))
;  =>
;  (printout t "Goal '" ?gid "' has failed, evaluating" crlf)
;  (modify ?pa (state EXECUTION-SUCCEEDED))
;  (modify ?g (mode DISPATCHED) (outcome UNKNOWN))
;)

; ============================= Production goals ===============================

(defrule goal-production-create-production-maintain
" The parent production goal. Allows formulation of
  production goals only if the proper game state selected
  and the domain got loaded. Other production goals are
  formulated as sub-goals of this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (domain-facts-loaded)
  (wm-fact (key central agent robot args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (class PRODUCTION-MAINTAIN) (params $?frequency robot ?robot $?params)))
  (wm-fact (key refbox phase) (value PRODUCTION))
  (wm-fact (key game state) (value RUNNING))
  (NavGraphWithMPSGeneratorInterface (final TRUE))
  (wm-fact (key navgraph waitzone generated))
  =>
  (printout t "Creating production-maintain goal for " ?robot crlf)
  (bind ?goal (goal-tree-assert-run-endless PRODUCTION-MAINTAIN 1))
  (modify ?goal (params frequency 1 robot ?robot))
)

(defrule goal-production-create-fill-cap
" Fill a cap into a cap station.
  Use a capcarrier from the corresponding shelf to feed it into a cap station."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class PREPARE-CAPS) (params robot ?robot) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
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
  (assert (goal (id (sym-cat FILL-CAP- (gensym*)))
                (class FILL-CAP) (sub-type SIMPLE)
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        cc ?cc
                )
                (required-resources (sym-cat ?mps -INPUT) ?cc)
  ))
)

(defrule goal-production-create-produce-c0
" Produce a C0 product: Get the correct base and mount the right cap on it.
  The produced workpiece stays in the output of the used cap station after
  successfully executing this goal.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class INTERMEDEATE-STEPS) (params robot ?robot) (mode FORMULATED))
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
  (test (eq ?complexity C0))
  ;(wm-fact (key config rcll allowed-complexities) (values $?allowed&:(member$ (str-cat ?complexity) ?allowed)))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
	            (value ?qd&:(> ?qr ?qd)))
  ;Active Order CEs
  (or (and (wm-fact (key domain fact wp-spawned-for args? wp ?spawned-wp r ?robot))
           (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN))
           (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
           (not (wm-fact (key order meta wp-for-order args? wp ?any-ord-wp ord ?order))))
      (and (wm-fact (key domain fact holding args? r ?robot wp ?spawned-wp))
           (wm-fact (key domain fact wp-base-color args? wp ?spawned-wp col ?base-color))
           (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
           (wm-fact (key domain fact wp-cap-color args? wp ?wp cap-color CAP_NONE))))
  
  =>
  (printout error "GOAL-C0 formulated" crlf)
  (bind ?required-resources ?order ?spawned-wp)
  (assert (goal (id (sym-cat PRODUCE-C0- (gensym*)))
                (class PRODUCE-C0) (sub-type SIMPLE)
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
                (required-resources (sym-cat ?mps -INPUT) ?required-resources)
  ))
)


(defrule goal-production-create-clear-cs-for-capless-carriers
" Remove a capless capcarrier from the output of a cap station after
  retrieving a cap from it.
"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class CLEAR) (params robot ?robot) (mode FORMULATED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
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
                (class CLEAR-MPS) (sub-type SIMPLE)
                (parent ?production-id)
                (params robot ?robot
                        mps ?mps
                        wp ?wp
                        side OUTPUT
                )
                (required-resources (sym-cat ?mps -OUTPUT) ?wp)
  ))
)


(defrule goal-production-create-deliver
  "Deliver a fully produced workpiece."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class DELIVER-PRODUCTS) (params robot ?robot) (mode FORMULATED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  (wm-fact (key refbox team-color) (value ?team-color))
  ;MPS-DS CEs
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  ;MPS-CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS|SS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;WP-CEs
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?ds side INPUT)))
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
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
           (value ?qd&:(> ?qr ?qd)))
  (or (and (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
           (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp))))
      (wm-fact (key domain fact holding args? r ?robot wp ?wp)))
  (not (wm-fact (key wp meta wait-for-delivery args? wp ?wp wait-for ?)))
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
  (assert (goal (id (sym-cat DELIVER- (gensym*)))
                (class DELIVER) (sub-type SIMPLE)
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
                (required-resources (sym-cat ?mps -OUTPUT) ?order ?wp (sym-cat ?ds -INPUT))
  ))
)


; ======================= No-progress (busy waiting) goals ==========================
(defrule goal-production-create-wait
  "Keep waiting at one of the waiting positions."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class NO-PROGRESS) (params robot ?robot) (mode FORMULATED))
  (domain-object (type waitpoint) (name ?waitpoint))
  (wm-fact (key domain fact at args? r ?robot m ?waitpoint&:
               (eq (str-length (str-cat ?waitpoint)) 10) side WAIT))
  =>
  (printout t "Goal " WAIT " formulated" crlf)
  (assert (goal (id (sym-cat WAIT- (gensym*)))
               (class WAIT) (sub-type SIMPLE)
               (parent ?production-id)
               (params r ?robot
                       point ?waitpoint)
               (required-resources ?waitpoint)
  ))
)


(defrule goal-production-create-go-wait
  "Drive to a waiting position and wait there."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id ?production-id) (class NO-PROGRESS) (params robot ?robot) (mode FORMULATED))
  (goal (id ?urgent) (class URGENT) (mode FORMULATED))
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
                (parent ?production-id)
                (params r ?robot
                        point ?waitpoint
                )
                (required-resources ?waitpoint)
  ))
)
