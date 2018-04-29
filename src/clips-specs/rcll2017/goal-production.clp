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
  ?*ENTER-FIELD-RETRIES* = 10
  ?*MAX-RETRIES-PICK* = 2
  ?*MAX-RETRIES-PUT-SLIDE* = 2


  ; production order priorities
  ?*PRIORITY-GO-WAIT-HACK* = 200
  ?*PRIORITY-FIND-MISSING-MPS* = 110
  ?*PRIORITY-DELIVER* = 100
  ?*PRIORITY-RESET* = 98
  ?*PRIORITY-CLEAR-BS* = 97
  ?*PRIORITY-PRODUCE-CX* = 95
  ?*PRIORITY-PRODUCE-C0* = 90
  ?*PRIORITY-ADD-ADDITIONAL-RING* = 85
  ?*PRIORITY-ADD-FIRST-RING* = 80
  ?*PRIORITY-CLEAR-CS* = 70
  ?*PRIORITY-CLEAR-RS* = 55
  ?*PRIORITY-PREFILL-CS* = 50
  ?*PRIORITY-PREFILL-RS-WITH-HOLDING-BASE* = 45
  ?*PRIORITY-PREFILL-RS* = 40
  ?*PRIORITY-ADD-ADDITIONAL-RING-WAITING* = 20
  ?*PRIORITY-DISCARD-UNKNOWN* = 10
  ?*PRIORITY-NOTHING-TO-DO* = -1
  ;ToDo:The proirites are copied from old agent
  ;     for the current moment. Filter out uneeded
  ;     later. For now needed for refrence.

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

; ## Maintain beacon sending
(defrule goal-reasoner-create-beacon-maintain
  (not (goal (id BEACONMAINTAIN)))
  =>
  (assert (goal (id BEACONMAINTAIN) (type MAINTAIN)))
)

(defrule goal-reasoner-create-beacon-achieve
  ?g <- (goal (id BEACONMAINTAIN) (mode SELECTED))
  (not (goal (id BEACONACHIEVE)))
  (time $?now)
  ; TODO: make interval a constant
  (goal-meta (goal-id BEACONMAINTAIN)
    (last-achieve $?last&:(timeout ?now ?last 1)))
  =>
  (assert (goal (id BEACONACHIEVE) (parent BEACONMAINTAIN)))
)


; ## Maintain wp-spawning
(defrule goal-reasoner-create-wp-spawn-maintain
 (domain-facts-loaded)
 (not (goal (id WPSPAWN-MAINTAIN)))
 =>
 (assert (goal (id WPSPAWN-MAINTAIN) (type MAINTAIN)))
)

(defrule goal-reasoner-create-wp-spawn-achieve
  ?g <- (goal (id WPSPAWN-MAINTAIN) (mode SELECTED))
  (not (goal (id WPSPAWN-ACHIEVE)))
  (time $?now)
  ; TODO: make interval a constant
  (goal-meta (goal-id WPSPAWN-MAINTAIN)
  (last-achieve $?last&:(timeout ?now ?last 1)))
  (wm-fact (key domain fact self args? r ?robot))
  (not (and
    (domain-object (name ?wp) (type workpiece))
    (wm-fact (key domain fact wp-spawned-by args? wp ?wp r ?robot))))
  =>
  (assert (goal (id WPSPAWN-ACHIEVE) (parent WPSPAWN-MAINTAIN)
                                     (params robot ?robot)))
)

(defrule goal-reasoner-create-refill-shelf-maintain
  (domain-facts-loaded)
  (not (goal (id REFILL-SHELF-MAINTAIN)))
  =>
  (assert (goal (id REFILL-SHELF-MAINTAIN) (type MAINTAIN)))
)

(defrule goal-reasoner-create-refill-shelf-achieve
  ?g <- (goal (id REFILL-SHELF-MAINTAIN) (mode SELECTED))
  (not (goal (id REFILL-SHELF-ACHIEVE)))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (not (wm-fact (key domain fact wp-on-shelf args? wp ?wp m ?mps spot ?spot)))
  =>
  (assert (goal (id REFILL-SHELF-ACHIEVE) (parent REFILL-SHELF-MAINTAIN)
                                          (params mps ?mps)))
)

(defrule navgraph-compute-wait-positions-finished
  (NavGraphWithMPSGeneratorInterface (final TRUE))
=>
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (do-for-all-facts ((?waitzone navgraph-node)) (str-index "WAIT-" ?waitzone:name)
    (assert
      (domain-object (name ?waitzone:name) (type location))
      (domain-fact (name location-free) (param-values (sym-cat ?waitzone:name) WAIT))
      (wm-fact (key navgraph waitzone args? name (sym-cat ?waitzone:name)) (is-list TRUE) (type INT) (values (nth$ 1 ?waitzone:pos) (nth$ 2 ?waitzone:pos)))
    )
  )
  (assert (wm-fact (key navgraph waitzone generated) (type BOOL) (value TRUE)))
)

; ## Maintain production
(defrule goal-reasoer-create-goal-production-maintain
  "The parent production goal. Allowes formulation of
  production goals only if proper game state selected
  and domain loaded. Other production goals are
  formulated as sub-goals of this goal"
  (domain-facts-loaded)
  (not (goal (id PRODUCTION-MAINTAIN)))
  (wm-fact (key refbox phase) (type UNKNOWN) (value PRODUCTION))
  (wm-fact (key game state) (type UNKNOWN) (value RUNNING))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (NavGraphWithMPSGeneratorInterface (final TRUE))
  (wm-fact (key navgraph waitzone generated))
  =>
  (assert (goal (id PRODUCTION-MAINTAIN) (type MAINTAIN)))
)

(defrule goal-reasoner-create-go-wait-hack
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key domain fact self args? r ?self))
  (wm-fact (key domain fact location-free args? l ?waitpoint side WAIT))
  (or (wm-fact (key domain fact at args? r ?self m ?another-point&:(neq ?waitpoint ?another-point) side WAIT))
      (wm-fact (key domain fact at args? r ?self m START side INPUT)))
  (wm-fact (key config rcll master) (value ?master&:(neq ?self (sym-cat ?master))))
  =>
  (printout t "Goal " GO-WAIT " formulated" crlf)
  (assert (goal (id GO-WAIT) (priority  ?*PRIORITY-GO-WAIT-HACK*)
                              (parent PRODUCTION-MAINTAIN)
                              (params r ?self
                                      point ?waitpoint
                                      )))
  ; (assert (goal-already-tried FILL-CAP))
)

(defrule goal-reasoner-create-enter-field
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
  (wm-fact (key refbox state) (value RUNNING))
  (wm-fact (key refbox phase) (value PRODUCTION|EXPLORATION))
  ; (NavGraphGeneratorInterface (final TRUE))
  ; (not (wm-fact (key domain fact entered-field args? r ?robot)))
  =>
  (printout t "Goal " ENTER-FIELD-ACHIEVE " formulated" crlf)
  (assert (goal (id ENTER-FIELD-ACHIEVE) ))
)

(defrule goal-reasoner-create-fill-cap-goal
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~DOWN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op RETRIEVE_CAP))
  (not (wm-fact (key domain fact wp-at args? wp ?wp-a m ?mps side ?any-side)))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))
  (not (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
  =>
  (printout t "Goal " FILL-CAP " formulated" crlf)
  (assert (goal (id FILL-CAP) (priority ?*PRIORITY-PREFILL-CS*)
                              (parent PRODUCTION-MAINTAIN)
                              (params robot ?robot
                                      mps ?mps
                                      cc ?cc
                                      )))
  ; (assert (goal-already-tried FILL-CAP))
)

(defrule goal-reasoner-create-clear-cs
  "Remove an unknown base from CS after retrieving a cap from it."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  ;Maybe add a check for the base_color
  ; (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s READY-AT-OUTPUT))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))
  =>
  (printout t "Goal " CLEAR-CS " formulated" crlf)
  (assert (goal (id CLEAR-CS) (priority ?*PRIORITY-CLEAR-CS*)
                              (parent PRODUCTION-MAINTAIN)
                              (params robot ?robot
                                      mps ?mps
                                      wp ?wp
                                      )))
  ; (assert (goal-already-tried CLEAR-CS))
)


(defrule goal-reasoner-clear-cs-from-expired-product
  "Remove an unknown base from CS after retrieving a cap from it."
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot Conditions
  (wm-fact (key domain fact self args? r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?some-wp)))
  ;MPS Condition
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~DOWN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;Order conditions
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key refbox order ?order-id delivery-end) (type UINT)
    (value ?end&:(< ?end (nth$ 1 ?game-time))))
  =>
  (printout t "Goal " CLEAR-CS " formulated" crlf)
  (assert (goal (id CLEAR-CS) (priority ?*PRIORITY-CLEAR-CS*)
                              (parent PRODUCTION-MAINTAIN)
                              (params robot ?robot
                                      mps ?mps
                                      wp ?wp
                                      )))
)


(defrule goal-reasoner-create-prifill-ring-station
  "Insert a base with unknown color in a RS for preparation"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact wp-usable args? wp ?wp))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  ;TODO: make the mps-state  a precond of the put-slid to save traviling time
  (wm-fact (key domain fact mps-state args? m ?mps s ~DOWN&~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TOW))
  ;CCs don't have a base color. Hence, models base with UNKOWN color
  ; (not (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color)))
  =>
  (printout t "Goal " FILL-RS " formulated" crlf)
  (assert (goal (id FILL-RS) (priority ?*PRIORITY-PREFILL-RS*)
                             (parent PRODUCTION-MAINTAIN)
                             (params robot ?robot
                                     mps ?mps
                                     wp ?wp
                                     rs-before ?rs-before
                                     rs-after ?rs-after
                                     )))
  ; (assert (goal-already-tried FILL-RS))
)

(defrule goal-reasoner-create-discard-unknown
  "Discard a base which is not needed if no RS can be pre-filled"
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;To-Do: Model state IDLE
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  ;only discard if ring stations have at least two bases loaded
  (wm-fact (key domain fact rs-filled-with args? m ?mps n TWO|THREE))
  ;question: or would be more correct to create it and later
  ;  reject it because its not useful
  =>
  (printout t "Goal " DISCARD-UNKNOWN " formulated" crlf)
  (assert (goal (id DISCARD-UNKNOWN) (priority ?*PRIORITY-DISCARD-UNKNOWN*)
                                     (parent PRODUCTION-MAINTAIN)
                                     (params robot ?robot
                                             wp ?wp
                                             )))
  ; (assert (goal-already-tried DISCARD-UNKNOWN))
)

(defrule goal-reasoner-create-produce-c0
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  ;Robot CEs
  (wm-fact (key domain fact self args? r ?robot))
  ;MPS-CS CEs
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~READY-AT-OUTPUT))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
  (wm-fact (key domain fact cs-buffered args? m ?mps col ?cap-color))
  (wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (not (wm-fact (key domain fact wp-at args? wp ?some-wp m ?mps side ?any-side)))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
  (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN&~DOWN&~READY-AT-OUTPUT))
  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  ;To-Do: Model the bs active-side
  ;Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com C0))
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
  ;TODO for multi-agent
  ;	Model old agents constraints
  ; 	(in-production 0)
  ; 	(in-delivery ?id&:(> ?qr (+ ?qd ?id)))
  =>
  (printout t "Goal " PRODUCE-C0 " formulated" crlf)
  (assert (goal (id PRODUCE-C0) (priority ?*PRIORITY-PRODUCE-C0*)
                                (parent PRODUCTION-MAINTAIN)
                                (params robot ?robot
                                        bs ?bs
                                        bs-side INPUT
                                        bs-color ?base-color
                                        mps ?mps
                                        cs-color ?cap-color
                                        order ?order
                                        )))
  ; (assert (goal-already-tried PRODUCE-C0))
)


(deffunction tac-ring-mount-time (?complexity ?rings)
  "Determine time to mount the remaining rings plus cap"
  (bind ?max-rings (eval (sub-string 2 3 (str-cat ?complexity))))
  (return (+ (* (- ?max-rings ?rings) ?*PRODUCE-RING-AHEAD-TIME*) ?*PRODUCE-CAP-AHEAD-TIME*))
)


(defrule goal-reasoner-create-mount-first-ring
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))

  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;Robot CEs
  (wm-fact (key domain fact self args?         r ?robot))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
  ;MPS-RS CEs
  (wm-fact (key domain fact mps-type args?       m ?mps-rs t RS))
  (wm-fact (key domain fact mps-state args?      m ?mps-rs s ~BROKEN))
  (wm-fact (key domain fact mps-team args?       m ?mps-rs col ?team-color))
  (wm-fact (key domain fact rs-filled-with args? m ?mps-rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec args?   m ?mps-rs r ?ring-color rn ?bases-needed))
  (wm-fact (key domain fact minuend ?bases-filled
                            subtrahend ?bases-needed
                            difference ?bases-remain&ZERO|ONE|TWO|THREE))
  (not (wm-fact (key domain fact rs-prepared-for args?  m ?mps-rs col ?some-col)))
  (not (wm-fact (key domain fact wp-at args? wp ?wp-rs m ?mps-rs side ?any-rs-side)))
  ;TODO think a lot about the old CE
  ; (incoming $?i&~:(member$ PROD_RING ?i))
  ; and what does it emply for the new locking
  ;MPS-BS CEs
  (wm-fact (key domain fact mps-type args?  m ?mps-bs t BS))
  (wm-fact (key domain fact mps-state args? m ?mps-bs s ~BROKEN))
  (wm-fact (key domain fact mps-team args?  m ?mps-bs col ?team-color))
  (not (wm-fact (key domain fact wp-at args? wp ?bs-wp m ?mps-bs side ?any-bs-side)))
  ;Order CEs
  (not (wm-fact (key evaluated fact wp-for-order args? wp ?ord-wp ord ?order)))

  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color) (value ?qd&:(> ?qr ?qd)))
  ; (wm-fact (key refbox order ?order delivery-begin) (type UINT)
    ; (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) (tac-ring-mount-time ?complexity 0)))))
  ; (wm-fact (key refbox order ?order delivery-end) (type UINT)
    ; (value ?end&:(> ?end (+ (nth$ 1 ?game-time) ?*PRODUCE-CX-LATEST-TIME*))))
  ;TODO for multi-agent
  ; Model old agents constraints
  ;  (in-production 0))
  ;  (in-delivery ?id&:(> ?qr (+ ?qd ?id)))
  (wm-fact (key config rcll allowed-produce-complexity) (values ?complexity))
  =>
  (printout t "Goal " MOUNT-FIRST-RING " formulated" crlf)
  (assert (goal (id MOUNT-FIRST-RING) (priority ?*PRIORITY-DELIVER*)
                             (parent PRODUCTION-MAINTAIN)
                             (params robot ?robot
                                        bs ?mps-bs
                                        bs-side OUTPUT
                                        bs-color ?base-color
                                        mps ?mps-rs
                                        ring-color ?ring-color
                                        rs-before ?bases-filled
                                        rs-after ?bases-remain
                                        rs-req ?bases-needed
                                        order ?order
                                        )))
  )



(defrule goal-reasoner-create-reset-mps
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key domain fact self args? r ?self))
  ?t <- (wm-fact (key monitoring action-retried args? r ?self a wp-get m ?mps wp ?wp)
                (value ?tried&:(>= ?tried ?*MAX-RETRIES-PICK*)))
  =>
  (printout t "Goal " RESET-MPS " formulated" crlf)
  (assert (goal (id RESET-MPS) (priority  ?*PRIORITY-RESET*)
                              (parent PRODUCTION-MAINTAIN)
                              (params r ?self
                                      m ?mps
                                      )))
  (retract ?t)
)

(defrule goal-reasoner-create-discard-failed-put-slide
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  (wm-fact (key refbox team-color) (value ?team-color))
  ;To-Do: Model state IDLE
  (wm-fact (key domain fact self args? r ?robot))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  (wm-fact (key domain fact mps-type args? m ?mps t RS))
  ?t <- (wm-fact (key monitoring action-retried args? r ?self a wp-put-slide-cc m ?mps wp ?wp)
                (value ?tried&:(>= ?tried ?*MAX-RETRIES-PICK*)))
  =>
  (printout t "Goal " DISCARD-UNKNOWN " formulated" crlf)
  (assert (goal (id DISCARD-UNKNOWN) (priority ?*PRIORITY-RESET*)
                                     (parent PRODUCTION-MAINTAIN)
                                     (params robot ?robot
                                             wp ?wp
                                             )))
  (retract ?t)
  ; (assert (goal-already-tried DISCARD-UNKNOWN))
)




(defrule goal-reasoner-create-deliver
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (goal (id PRODUCTION-MAINTAIN) (mode SELECTED))
  ;To-Do: Model state IDLE|wait-and-look-for-alternatives
  (wm-fact (key domain fact self args? r ?robot))
  ;CEs for MPS-DS
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  ;CEs for MPS-CS
  (wm-fact (key domain fact mps-type args? m ?mps t CS))
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))

  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))

  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
  (not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  ;note: could be moved to rejected checks
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
    (value ?qd&:(> ?qr ?qd)))
  (wm-fact (key refbox order ?order delivery-begin) (type UINT)
    (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*))))
  ;TODO for multi-agent
  ; Model old agents constraints
  ;  (in-production ?ip&:(> ?ip 0))
  ;  (in-delivery ?id)

  ;On evaluation of delivery. Update the quanetities deliverd
  ;Delete the evaluated wp-for-order
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  ;note: could be moved to rejected checks
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
    (value ?qd&:(> ?qr ?qd)))
  (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key refbox order ?order-id delivery-begin) (type UINT)
    (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*))))
  ;TODO for multi-agent
  ; Model old agents constraints
  ;  (in-production ?ip&:(> ?ip 0))
  ;  (in-delivery ?id)
  =>
  (printout t "Goal " DELIVER " formulated" crlf)
  (assert (goal (id DELIVER) (priority ?*PRIORITY-DELIVER*)
                             (parent PRODUCTION-MAINTAIN)
                             (params robot ?robot
                                     mps ?mps
                                     order ?order
                                     wp ?wp
                                     ds ?ds
                                     ds-gate ?gate
                                     base-color ?base-color
                                     cap-color ?cap-color
                                     )))
  ; (assert (goal-already-tried DELIVER))
)

; ## Goal Evaluation
 (defrule goal-reasoner-evaluate-failed-enter-field
   ?g <- (goal (id ENTER-FIELD-ACHIEVE) (mode FINISHED) (outcome FAILED))
  ?pa <- (plan-action (goal-id ENTER-FIELD-ACHIEVE) (status FAILED) (action-name enter-field))
  ?gm <- (goal-meta (goal-id ENTER-FIELD-ACHIEVE) (num-tries ?num-tries))
  =>
  (printout t "Goal '" ENTER-FIELD-ACHIEVE"' has failed, Evaluating" crlf)

  (if (= ?num-tries ?*ENTER-FIELD-RETRIES*) then
	(modify ?pa (status EXECUTION-SUCCEEDED))
	(modify ?g (mode DISPATCHED) (outcome UNKNOWN))
        else
	(bind ?num-tries (+ 1 ?num-tries))
	(modify ?gm (num-tries ?num-tries) (max-tries ?*ENTER-FIELD-RETRIES*))
	(modify ?g (mode EVALUATED))
  )
)

(defrule goal-reasoner-evaluate-production-maintain
  ?g <- (goal (id PRODUCTION-MAINTAIN) (parent nil) (mode FINISHED) (outcome ?outcome))
  ?gm <- (goal-meta (goal-id ?goal-id) (num-tries ?num-tries))
  ?t <- (wm-fact (key monitoring action-retried args? r ?self a ?an m ?mps wp ?wp)
                (value ?tried&:(>= ?tried ?*MAX-RETRIES-PICK*)))
  =>
  (printout t "Goal '" ?goal-id "' has been " ?outcome ", evaluating" crlf)
  (retract ?t)
  (modify ?g (mode EVALUATED))
)

(defrule goal-reasoner-evaluate-completed-subgoal-produce-c0
  ?g <- (goal (id PRODUCE-C0) (parent ?parent-id)
              (mode FINISHED) (outcome COMPLETED)
              (params $?params))
 ?gm <- (goal-meta (goal-id ?parent-id))
 (plan (goal-id PRODUCE-C0)
   (id ?plan-id))
 ?p <-(plan-action
         (plan-id ?plan-id)
         (action-name bs-dispense)
         (param-names r m side wp basecol)
         (param-values ?robot ?bs ?bs-side ?wp ?base-color))
 (time $?now)
 (wm-fact (key domain fact self args? r ?robot))
 =>
 (bind ?order (get-param-by-arg ?params order))
 (printout t "Goal '" PRODUCE-C0 "' has been completed, Evaluating" crlf)
 (assert (wm-fact (key evaluated fact wp-for-order args? wp ?wp ord ?order) (value TRUE)))
 (modify ?g (mode EVALUATED))
 (modify ?gm (last-achieve ?now))
)

(defrule goal-reasoner-evaluate-completed-subgoal-refill-shelf
  ?g <- (goal (id REFILL-SHELF-ACHIEVE) (parent REFILL-SHELF-MAINTAIN)
             (mode FINISHED) (outcome COMPLETED)
             (params mps ?mps))
  ?p <- (goal(id REFILL-SHELF-MAINTAIN))
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
     (wm-fact (key domain fact wp-unused args? wp ?cc1) (value TRUE))
     (wm-fact (key domain fact wp-unused args? wp ?cc2) (value TRUE))
     (wm-fact (key domain fact wp-unused args? wp ?cc3) (value TRUE))
     (wm-fact (key domain fact wp-cap-color args? wp ?cc1 col ?col) (value TRUE))
     (wm-fact (key domain fact wp-cap-color args? wp ?cc2 col ?col) (value TRUE))
     (wm-fact (key domain fact wp-cap-color args? wp ?cc3 col ?col) (value TRUE))
     (wm-fact (key domain fact wp-on-shelf args? wp ?cc1 m ?mps spot LEFT) (value TRUE))
     (wm-fact (key domain fact wp-on-shelf args? wp ?cc2 m ?mps spot MIDDLE) (value TRUE))
     (wm-fact (key domain fact wp-on-shelf args? wp ?cc3 m ?mps spot RIGHT) (value TRUE))
   )
   (modify ?g (mode EVALUATED))
)


(defrule goal-reasoner-evaluate-completed-subgoal-wp-spawn
  ?g <- (goal (id WPSPAWN-ACHIEVE) (parent WPSPAWN-MAINTAIN)
            (mode FINISHED) (outcome COMPLETED)
            (params robot ?robot))
  ?p <- (goal (id WPSPAWN-MAINTAIN))
  ?m <- (goal-meta (goal-id WPSPAWN-MAINTAIN))
  (time $?now)
  (wm-fact (key domain fact self args? r ?robot))
  =>
  (printout debug "Goal '" WPSPAWN-ACHIEVE "' (part of '" WPSPAWN-MAINTAIN
    "') has been completed, Evaluating" crlf)
     (bind ?wp-id (sym-cat WP (random-id)))
  (assert
    (domain-object (name ?wp-id) (type workpiece))
    (wm-fact (key domain fact wp-unused args? wp ?wp-id) (value TRUE))
    (wm-fact (key domain fact wp-cap-color args? wp ?wp-id col CAP_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring1-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring2-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-ring3-color args? wp ?wp-id col RING_NONE) (value TRUE))
    (wm-fact (key domain fact wp-base-color args? wp ?wp-id col BASE_NONE) (value TRUE))
    (wm-fact (key domain fact wp-spawned-by args? wp ?wp-id r ?robot) (value TRUE))
  )
  (modify ?g (mode EVALUATED))
  (modify ?m (last-achieve ?now))
)
