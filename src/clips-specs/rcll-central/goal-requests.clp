;---------------------------------------------------------------------------
;  goal-requests.clp - Handle requests for production support goals.
;
;  Created: Tue 05 Jul 2022 18:00:00 CET
;  Copyright  2022  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

; -------------------- Buffer Goals ------------------------

(defrule goal-request-accept-buffer-offers
  "If there is an unfulfilled buffer request, use a buffer offer."
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status OPEN assigned-to))
  ?request-offer <- (wm-fact (key request offer buffer args? $? col ?cap-col) (values assigned-to ?buffer-goal-id ?instruct-goal-id))
  =>
  (modify ?request (values status ACTIVE assigned-to ?buffer-goal-id ?instruct-goal-id))
  (retract ?request-offer)
)

(defrule goal-request-assert-buffer-goal
  "If there is an unfulfilled buffer request, create buffer goal."
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status OPEN assigned-to))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  (goal (class SUPPORT-ROOT) (id ?root-id))
  ; and there is no offer
  (not (wm-fact (key request offer buffer args? $? col ?cap-col) (values assigned-to $?)))
  =>
  (bind ?buffer-goal (goal-production-assert-buffer-cap ?cs ?cap-col ?order-id))
  (bind ?instruct-goal (goal-production-assert-instruct-cs-buffer-cap ?cs ?cap-col ?order-id))
  (modify ?request (values status ACTIVE assigned-to (fact-slot-value ?buffer-goal id) (fact-slot-value ?instruct-goal id)))
  (modify ?buffer-goal (parent ?root-id) (priority ?prio))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority ?prio))
)

(defrule goal-request-remap-buffer-offers
  "If there is an unfulfilled buffer request, use a buffer offer."
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to ?buffer-goal-id ?instruct-goal-id))
  ?request-offer <- (wm-fact (key request offer buffer args? $? col ?cap-col) (values assigned-to ?offer-buffer-goal-id ?offer-instruct-goal-id))

  ?buffer-goal <- (goal (id ?buffer-goal-id) (mode FORMULATED))
  ?instruct-goal <- (goal (id ?instruct-goal-id) (mode FORMULATED))

  ?offer-buffer-goal <- (goal (id ?offer-buffer-goal-id))
  ?offer-instruct-goal <- (goal (id ?offer-instruct-goal-id))
  =>
  (modify ?request (values status ACTIVE assigned-to ?offer-buffer-goal-id ?offer-instruct-goal-id))
  (modify ?offer-buffer-goal (priority ?prio))
  (modify ?offer-instruct-goal (priority ?prio))
  (retract ?request-offer ?buffer-goal ?instruct-goal)
)

(defrule goal-request-map-buffer-cap-completed-mount-cap
  "If a MOUNT-CAP goal is completed and the buffer request for the same order is not mapped
   to a fitting BUFFER-CAP goal, create the mapping."
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to ?buffer-goal-id ?instruct-goal-id))
  (goal (id ?buffer-goal-id) (mode RETRACTED) (outcome COMPLETED))
  (goal (id ?instruct-goal-id) (mode RETRACTED) (outcome COMPLETED))

  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED))
  (goal-meta (goal-id ?mount-cap-goal) (order-id ?order-id))
  =>
  (modify ?request (values status COMPLETED assigned-to ?buffer-goal-id ?instruct-goal-id))
)

(defrule goal-request-remove-buffer-failed
  ;an order root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))

  ;it has a request for buffering
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to ?buffer-goal-id ?instruct-goal-id))

  ;the instruct goal for mounting the cap was not used yet
  (goal (id ?instruct-mount-goal-id) (class INSTRUCT-CS-MOUNT-CAP) (mode FORMULATED))
  (goal-meta (goal-id ?instruct-mount-goal-id) (order-id ?order-id))

  ;the buffer cap request was completed
  (goal (id ?buffer-goal-id) (mode RETRACTED) (outcome COMPLETED))
  (goal (id ?instruct-goal-id) (mode RETRACTED) (outcome COMPLETED))
  =>
  (assert (wm-fact (key request offer buffer args? ord ?order-id col ?cap-col) (values assigned-to ?buffer-goal-id ?instruct-goal-id)))
  (retract ?request)
)

; -------------------- Payment and Discard Goals ------------------------

(defrule goal-request-accept-pay-offers
  "If there are unfulfilled payment offer, use it."
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status OPEN assigned-to))
  ?request-offer <- (wm-fact (key request offer pay args? $? m ?rs $?) (values assigned-to $?payment-goals))
  =>
  (modify ?request (values status ACTIVE assigned-to ?payment-goals))
  (retract ?request-offer)
)

(defrule goal-request-accept-discard-offers
  "If there are unfulfilled discard offer, use it."
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio-discard) (values status OPEN assigned-to))
  ?request-offer <- (wm-fact (key request offer discard args? $? cs ?cs $?) (values assigned-to $?discard-goals))
  =>
  (modify ?request (values status ACTIVE assigned-to ?discard-goals))
  (retract ?request-offer)
)

(defrule goal-request-assert-pay-with-base-goal
  "If there is an unfulfilled payment request, create a pay-with-base-goal."
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio) (values status OPEN assigned-to))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  ;and there is no payment offer
  (not (wm-fact (key request offer pay args? $? m ?rs $?) (values assigned-to $?)))
  =>
  (bind ?wp-base-pay (sym-cat BASE-PAY- (gensym*)))
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-base ?wp-base-pay ?bs INPUT ?rs INPUT ?order-id))
  (bind ?instruct-goal (goal-production-assert-instruct-bs-dispense-base ?wp-base-pay BASE_RED INPUT ?order-id ?bs))
  (assert
      (domain-object (name ?wp-base-pay) (type workpiece))
      (domain-fact (name wp-unused) (param-values ?wp-base-pay))
      (wm-fact (key domain fact wp-base-color args? wp ?wp-base-pay col BASE_NONE) (type BOOL) (value TRUE))
  )
  (modify ?request (values status ACTIVE assigned-to (fact-slot-value ?payment-goal id) (fact-slot-value ?instruct-goal id)))
  (modify ?payment-goal (parent ?root-id) (priority ?prio))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority ?prio))
)

(defrule goal-request-assert-discard
  "If there is an unfulfilled discard request, create a discard goal."
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio) (values status OPEN assigned-to))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  ;and there is no discard offer
  (not (wm-fact (key request offer discard args? $? cs ?cs $?) (values assigned-to $?)))
  =>
  (bind ?discard-goal (goal-production-assert-discard UNKNOWN ?cs OUTPUT ?order-id))
  (bind ?instruct-goal (goal-production-assert-instruct-ds-discard UNKNOWN ?ds))
  (modify ?request (values status ACTIVE assigned-to (fact-slot-value ?discard-goal id) (fact-slot-value ?instruct-goal id)))
  (modify ?discard-goal (parent ?root-id) (priority ?prio))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority ?prio))
)

(defrule goal-request-remap-discard-to-pay-with-cc
  "If there is an active request for discard and an active request for payment-with-base
  but the goals have not been started yet, we can remap them to a pay-with-cc goal."
  ?request-pay <- (wm-fact (key request pay args? ord ?order-id-pay m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))
  ?request-dis <- (wm-fact (key request discard args? ord ?order-id-discard cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))
  (goal (class SUPPORT-ROOT) (id ?root-id))

  ;the assigned goals have not been started yet
  (not (goal (id ?request-goal&:(or (member$ ?request-goal ?payment-goals) (member$ ?request-goal ?discard-goals))) (mode ~FORMULATED)))
  ;the mapping is currently on distinct discard and pay-with-base goals
  (goal (id ?discard-goal&:(member$ ?discard-goal ?discard-goals)) (class DISCARD) (params wp ?wp $?))
  (goal (id ?payment-goal&:(member$ ?payment-goal ?payment-goals)) (class PAY-FOR-RINGS-WITH-BASE))
  ;there is a buffer goal already running (so we can discard soon, to avoid slow-down in payments)
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs $?))
  ;but there is no other payment goal mapped to the same CS to avoid queuing
  (not (and
    (wm-fact (key request pay args? ord ? m ? ring ? seq ? prio ?) (values status ACTIVE assigned-to $?other-payment-goals))
    (goal (id ?other-payment-goal&:(member$ ?other-payment-goal ?other-payment-goals))
          (mode ~RETRACTED)
          (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
          (params $? wp-loc ?cs $?))
  ))
  =>
  (do-for-all-facts ((?goal goal) (?goal-meta goal-meta))
    (and (or (member$ ?goal:id ?payment-goals) (member$ ?goal:id ?discard-goals)) (eq ?goal:id ?goal-meta:goal-id))
    (retract ?goal)
    (retract ?goal-meta)
  )

  (bind ?pay-with-cc-goal (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs OUTPUT ?rs INPUT ?order-id-pay))
  (modify ?request-pay (values status ACTIVE assigned-to (fact-slot-value ?pay-with-cc-goal id)))
  (modify ?request-dis (values status ACTIVE assigned-to (fact-slot-value ?pay-with-cc-goal id)))
  (modify ?pay-with-cc-goal (parent ?root-id) (priority ?prio-discard))
)

(defrule goal-request-map-pay-completed
  "If there is a completed mount ring goal that used up a payment associatable with a fulfilled request, mark the request as completed."
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))
  (goal (class MOUNT-RING) (id ?mount-ring-goal) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?rs $? ring-color ?ring-color))
  (goal-meta (goal-id ?mount-ring-goal) (order-id ?order-id))
  ; the goals associated with the request have been completed
  (not (goal (id ?id&:(member$ ?id ?payment-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
  ; the payments have actually been consumed
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order-id))
  (wm-fact (key wp meta prev-step args? wp ?wp) (value ?prev-step))
  (test
    (or
        (eq ?prev-step ?ring)
        (eq ?prev-step CAP)
        (and (eq ?prev-step RING1) (eq ?ring RING1))
        (and (eq ?prev-step RING2) (or (eq ?ring RING1) (eq ?ring RING2)))
        (and (eq ?prev-step RING3) (or (eq ?ring RING1) (eq ?ring RING2) (eq ?ring RING3)))
    )
  )
  =>
  (modify ?request (values status COMPLETED assigned-to ?payment-goals))
)

(defrule goal-request-map-discard-completed
  "If there is a completed mount cap goal or discard goal, mark the associated request as completed"
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))
  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?cs $?))
  (goal-meta (goal-id ?mount-cap-goal) (order-id ?order-id))
  ; the goals associated with the request have been completed
  (not (goal (id ?id&:(member$ ?id ?discard-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
  =>
  (modify ?request (values status COMPLETED assigned-to ?discard-goals))
)

(defrule goal-request-remove-payment-failed
  ;an order root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))

  ;it has a request for discarding
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))

  ;the associated goals were completed
  (not (goal (id ?id&:(member$ ?id ?payment-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
  =>
  (assert (wm-fact (key request offer pay args? ord ?order-id m ?rs) (values assigned-to ?payment-goals)))
  (retract ?request)
)

(defrule goal-request-remove-discard-failed
  ;an order root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))

  ;it has a request for discarding
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))

  ;the associated goals were completed
  (not (goal (id ?id&:(member$ ?id ?discard-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
  =>
  (assert (wm-fact (key request offer discard args? ord ?order-id cs ?cs) (values assigned-to ?discard-goals)))
  (retract ?request)
)


; -------------------- Handling of broken machines ------------------------

(defrule goal-request-buffer-reset-broken-mps
  "Assuming the breaking of the machine leads to graceful termination of all goals, we can simply reset the requests"
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to $?buffer-goals))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (wm-fact (key domain fact mps-state args? m ?cs s BROKEN))
  (goal (id ?buffer-goal&:(member$ ?buffer-goal ?buffer-goals)) (mode ~FORMULATED))
  =>
  (modify ?request (values status OPEN assigned-to))
  (delayed-do-for-all-facts ((?g goal))
    (member$ ?g:id ?buffer-goals)
    (modify ?g (outcome FAILED) (mode RETRACTED))
  )
)

(defrule goal-request-discard-reset-broken-mps
  "Assuming the breaking of the machine leads to graceful termination of all goals, we can simply reset the requests"
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))
  (wm-fact (key domain fact mps-state args? m ?cs s BROKEN))
  (goal (id ?discard-goal&:(member$ ?discard-goal ?discard-goals)) (mode ~FORMULATED))
  =>
  (modify ?request (values status OPEN assigned-to))
)

(defrule goal-request-pay-reset-broken-mps
  "Assuming the breaking of the machine leads to graceful termination of all goals, we can simply reset the requests"
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))
  (wm-fact (key domain fact mps-state args? m ?rs s BROKEN))
  (goal (id ?payment-goal&:(member$ ?payment-goal ?payment-goals)) (mode ~FORMULATED))
  =>
  (modify ?request (values status OPEN assigned-to))
)


; -------------------- Clean-up Goals ------------------------

(defrule goal-request-handler-clean-up-holding
  " If there is an orphaned WP create a handling request."
  (domain-object (name ?orphaned-wp) (type workpiece))

  (wm-fact (key domain fact holding args? r ?robot wp ?orphaned-wp))
  ; it is orphaned, i.e. there is no clean-up request and it does not occur in any goal
  (not (goal (mode ~RETRACTED) (params $? ?orphaned-wp $?)))
  (not (wm-fact (key request clean-up args? wp ?orphaned-wp $?)))
  =>
  (assert (wm-fact (key request clean-up args? wp ?orphaned-wp  mps HOLDING side HOLDING ) (values status OPEN assigned-to)))
)

(defrule goal-request-handler-clean-up-at-machine
  " If there is an orphaned WP at a machine create a handling request."
  (domain-object (name ?orphaned-wp) (type workpiece))

  (wm-fact (key domain fact wp-at args? wp ?orphaned-wp m ?mps side ?side))
  ; it is orphaned, i.e. there is no clean-up request and it does not occur in any goal
  (not (goal (mode ~RETRACTED) (params $? ?orphaned-wp $?)))
  (not (wm-fact (key request clean-up args? wp ?orphaned-wp  $?)))
  =>
  (assert (wm-fact (key request clean-up args? wp ?orphaned-wp mps ?mps side ?side) (values status OPEN assigned-to)))
)

(defrule goal-request-handler-clean-up-activate-pay
  " If there is an orphaned WP and there are empty slides create a payment goal.
  "
  ?request <- (wm-fact (key request clean-up args? wp ?wp mps ?mps side ?side) (values status OPEN assigned-to))
  (domain-fact (name rs-filled-with) (param-values ?rs&:(< (compute-rs-total-payments ?rs) 3)))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  =>
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-base ?wp ?mps ?side ?rs INPUT nil))
  (modify ?request (values status ACTIVE assigned-to (fact-slot-value ?payment-goal id)))
  (modify ?payment-goal (parent ?root-id) (priority 100))
)

(defrule goal-request-handler-clean-up-activate-discard
  " If there is an orphaned WP and there are no empty slides create a discard goal.
  "
  ?request <- (wm-fact (key request clean-up args? wp ?wp mps ?mps side ?side) (values status OPEN assigned-to))
  (not (domain-fact (name rs-filled-with) (param-values ?rs&:(< (compute-rs-total-payments ?rs) 2))))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  =>
  (bind ?discard-goal (goal-production-assert-discard ?wp ?mps ?side nil))
  (bind ?instruct-goal (goal-production-assert-instruct-ds-discard ?wp ?ds))
  (modify ?request (values status ACTIVE assigned-to (fact-slot-value ?discard-goal id) (fact-slot-value ?instruct-goal id)))
  (modify ?discard-goal (parent ?root-id) (priority 100))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority 100))
)


(defrule goal-request-handler-convert-clean-up
  "If the clean-up goals for payment were finished, convert them to an offer "
  ?request <- (wm-fact (key request clean-up args? wp ?wp mps ?mps side ?side) (values status ACTIVE assigned-to $?clean-up-goals))

  ;the associated goals were completed
  (goal (class PAY-FOR-RINGS-WITH-BASE) (id ?id&:(member$ ?id ?clean-up-goals)) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?rs $?))
  =>
  (retract ?request)
  (assert (wm-fact (key request offer pay args? ord nil m ?rs) (values assigned-to ?clean-up-goals)))
)




; -------------------- Block Requests ------------------------
(defrule goal-request-deactivate-block
  " If there is a block request and it either timed out or the WP was moved to
    the DS, deactivate the block request.
  "
  (wm-fact (key refbox game-time) (values ?now $?))
  ?br <- (wm-fact (key request block args? mps ?ds wp ?wp com ?com ord ?order) (values ACTIVE ?st ?duration))

  (or
    (wm-fact (key domain fact wp-at args? wp ?wp m ?ds side INPUT)) ;wp arrived at DS
    (test (> (- ?now ?st) ?duration))
    (not (wm-fact (key domain fact wp-usable args? wp ?wp))) ;the wp was flushed away or consumed
  )
  =>
  (modify ?br (values INACTIVE))
)

(defrule goal-request-block-ds-almost-finished-product
  " If there is a product that is soon going to be completed,
    issue a request to keep the the delivery station free, if possible.
  "
  ;get wp and order information
  (domain-object (name ?wp) (type workpiece))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (or
    (wm-fact (key wp meta next-step args? wp ?wp) (value DELIVER))
    (wm-fact (key wp meta next-machine args? wp ?wp) (value ?ds))
  )
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))

  ;either the wp is already at the output of a machine or the output is clear
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (or
    (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side OUTPUT))
    (and
      (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side INPUT))
      (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?cs side OUTPUT)))
    )
  )
  (wm-fact (key refbox game-time) (values ?now $?))
  (not (wm-fact (key request block args? mps ?ds wp ?wp com ?com ord ?order) (values $?)))
  =>
  (assert (wm-fact (key request block args? mps ?ds wp ?wp com ?com ord ?order) (values ACTIVE ?now ?*BLOCK-DURATION-DS*)))
)

(defrule goal-request-block-rs-almost-finished-product
  " If there is a product that is soon going to a RS,
    issue a request to keep the the RS free.
  "
  ;get wp and order information
  (domain-object (name ?wp) (type workpiece))
  (wm-fact (key wp meta next-step args? wp ?wp) (value ?ring&RING1|RING2|RING3))
  (wm-fact (key wp meta next-machine args? wp ?wp) (value ?rs))
  (wm-fact (key wp meta prev-machine args? wp ?wp) (value ?prev-mps))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))

  ;the wp is already at the output of the previous machine
  (wm-fact (key domain fact wp-at args? wp ?wp m ?prev-mps side OUTPUT))
  ;the input side of the target RS is free
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?rs side INPUT)))

  (wm-fact (key refbox game-time) (values ?now $?))
  (not (wm-fact (key request block args? mps ?rs wp ?wp com ?com ord ?order) (values $? ?ring)))
  =>
  (assert (wm-fact (key request block args? mps ?rs wp ?wp com ?com ord ?order) (values ACTIVE ?now ?*BLOCK-DURATION-RS* ?ring )))
)

(defrule goal-request-block-cs-almost-finished-product
  " If there is a product that is soon going to need a CS,
    issue a request to keep the the CS free.
  "
  ;get wp and order information
  (domain-object (name ?wp) (type workpiece))
  (wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
  (wm-fact (key wp meta next-machine args? wp ?wp) (value ?cs))
  (wm-fact (key wp meta prev-machine args? wp ?wp) (value ?prev-mps))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))

  ;the wp is already at the output of the previous machine
  (wm-fact (key domain fact wp-at args? wp ?wp m ?prev-mps side OUTPUT))
  ;the input side of the target CS is free and there is a cap buffered
  (not (wm-fact (key domain fact wp-at args? wp ?any-wp m ?cs side INPUT)))
  (wm-fact (key domain fact cs-buffered args? m ?cs col ?any-cap-color))

  (wm-fact (key refbox game-time) (values ?now $?))
  (not (wm-fact (key request block args? mps ?cs wp ?wp com ?com ord ?order) (values $?)))
  =>
  (assert (wm-fact (key request block args? mps ?cs wp ?wp com ?com ord ?order) (values ACTIVE ?now ?*BLOCK-DURATION-CS*)))
)
