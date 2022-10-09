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


; handle incoming requests

(defrule goal-request-assert-buffer-goal
  "If there is an unfulfilled buffer request, create buffer goal."
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (value OPEN))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  =>
  (bind ?buffer-goal (goal-production-assert-buffer-cap ?cs ?cap-col ?order-id))
  (modify ?request (value ACTIVE))
  (modify ?buffer-goal (parent ?root-id) (priority ?prio))
)

(defrule goal-request-assert-pay-with-cap-carrier
  "If there is an unfulfilled discard and payment request of the same goal,
   create a pay with cap-carrier goal to handle both requests."
  ?request-discard <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio-discard) (value OPEN))
  ?request-payment <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (value OPEN))
  ; to avoid potential delays by not being able to remove the cap-carrier, only pair the last payment request
  ; with the discard request
  (not (wm-fact (key request pay args? ord ?order-id $? seq ?other-seq&:(> 0 (str-compare (str-cat ?other-seq) (str-cat ?seq))) $?) (value nil)))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  =>
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs UNKNOWN ?rs INPUT ?order-id))
  (modify ?request-discard (value ACTIVE))
  (modify ?request-payment (value ACTIVE))
  (modify ?payment-goal (parent ?root-id) (priority ?prio-payment))
)

(defrule goal-request-assert-pay-with-cap-carrier-from-active-requests
  "If there is a formulated discard goal and a new payment request comes in
   or a formulated payment goal and a new discard request comes in,
   combine them into one pay-with-cap-carrier goal."
  (goal (class SUPPORT-ROOT) (id ?root-id))
  ?request-discard <- (wm-fact (key request discard args? ord ?order-discard cs ?cs prio ?prio-discard) (value ?value-discard))
  ?request-payment <- (wm-fact (key request pay args? ord ?order-payment m ?rs ring ?ring seq ?seq prio ?prio-payment) (value ?value-payment))
  ?goal <- (goal (id ?goal-id) (class ?goal-class) (mode FORMULATED) (params $?goal-params))

  ;there is no payment request that is open and has a higher sequence number (occurs later)
  (not (wm-fact (key request pay args? $? seq ?other-seq&:(> 0 (str-compare (str-cat ?other-seq) (str-cat ?seq))) $?) (value OPEN)))

  ;only do a swap if there is already a buffer-cap running and no cap-carrier payment goal to avoid deadlocks
  (goal (class BUFFER-CAP) (mode DISPATCHED))
  (not (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER) (mode ~RETRACTED)))

  ;one request must be active, one must be open and the goal class must match
  (test
    (or
        (and (eq ?value-discard ACTIVE) (eq ?value-payment OPEN) (eq ?goal-class DISCARD))
        (and (eq ?value-discard OPEN) (eq ?value-payment ACTIVE) (eq ?goal-class PAY-FOR-RINGS-WITH-BASE))
    )
  )
  =>
  ;if the goal is a PAY-FOR-RINGS-WITH-BASE goal, retract the corresponding instruct goal too
  (if (eq ?goal-class PAY-FOR-RINGS-WITH-BASE) then
    (bind ?base-wp (get-param-by-arg ?goal-params wp))
    (do-for-fact ((?instruct-goal goal))
      (and
        (eq ?instruct-goal:class INSTRUCT-BS-DISPENSE-BASE)
        (member$ ?base-wp ?instruct-goal:params)
      )
      (retract ?instruct-goal)
    )
  )

  ;retract the goal to be replaced
  (retract ?goal)

  ;assert the new payment goal
  (bind ?new-goal (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs UNKNOWN ?rs INPUT ?order-payment))
  (modify ?request-discard (value ACTIVE))
  (modify ?request-payment (value ACTIVE))

  (bind ?prio ?prio-payment)
  (if (> 0 (str-compare (str-cat ?prio-discard) (str-cat ?prio-payment))) then
    (bind ?prio ?prio-discard)
  )
  (modify ?new-goal (parent ?root-id) (priority ?prio))
)

(defrule goal-request-assert-discard-goal
  "If there is a discard request that is not paired with a goal yet, create a discard goal."
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio) (value OPEN))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  =>
  (bind ?discard-goal (goal-production-assert-discard UNKNOWN ?cs OUTPUT ?order-id))
  (modify ?request (value ACTIVE))
  (modify ?discard-goal (parent ?root-id) (priority ?prio))
)

(defrule goal-request-assert-payment-goal
  "If there is a payment request that is not paired with a goal yet,
   create a pay-with-base goal and a base dispense goal"
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio) (value OPEN))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  =>
  (bind ?wp-base-pay (sym-cat BASE-PAY- (gensym*)))
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-base ?wp-base-pay ?bs INPUT ?rs INPUT ?order-id))
  (modify ?request (value ACTIVE))
  (modify ?payment-goal (parent ?root-id) (priority ?prio))

  (bind ?instruct-goal (goal-production-assert-instruct-bs-dispense-base ?wp-base-pay BASE_RED INPUT ?order-id ?bs))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority ?prio))

  (assert
      (domain-object (name ?wp-base-pay) (type workpiece))
      (domain-fact (name wp-unused) (param-values ?wp-base-pay))
      (wm-fact (key domain fact wp-base-color args? wp ?wp-base-pay col BASE_NONE) (type BOOL) (value TRUE))
  )
)

; take over existing request offers

(defrule goal-request-accept-buffer-offer
  "If there is an unfulfilled buffer request, use a buffer offer."
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (value ACTIVE))
  ?request-offer <- (wm-fact (key request offer buffer args? $? col ?cap-col) (value ?buffer-goal-id))
  ?buffer-goal <- (goal (class BUFFER-CAP) (mode FORMULATED) (params $? cap-color ?cap-col $?))
  =>
  (modify ?request (value ?buffer-goal-id))
  (retract ?request-offer ?buffer-goal)
)

(defrule goal-request-accept-pay-with-cc-offer
  "If there is a discard and pay request that is not paired with a goal yet, use a pay-with-cc offer."
  ?request-discard <- (wm-fact (key request discard args? ord ?order-id cs ?cs $?) (value ACTIVE))
  ?request-pay <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq $?) (value ACTIVE))
  ?request-offer <- (wm-fact (key request offer pay-with-cc args? $? cs ?cs rs ?rs) (value ?payment-goal-id))
  ?pay-goal <- (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER) (mode FORMULATED) (params $? wp-loc ?cs $? target-mps ?rs $?))
  =>
  (modify ?request-discard (value ?payment-goal-id))
  (modify ?request-pay (value ?payment-goal-id))
  (retract ?request-offer ?pay-goal)
)

(defrule goal-request-accept-discard-offer
  "If there is a discard request that is not paired with a goal yet, use a discard offer."
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio) (value ACTIVE))
  ?request-offer <- (wm-fact (key request offer discard args? $? cs ?cs) (value ?discard-goal-id))
  ?discard-goal <- (goal (class DISCARD) (mode FORMULATED) (params $? wp-loc ?cs $?))
  =>
  (modify ?request (value ?discard-goal-id))
  (retract ?request-offer ?discard-goal)
)

(defrule goal-request-accept-payment-offer
  "If there is a payment request that is not paired with a goal yet, "
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio) (value ACTIVE))
  ?request-offer <- (wm-fact (key request offer pay args? $? m ?rs $?) (value ?payment-goal-id))
  ?payment-goal <- (goal (class PAY-FOR-RINGS-WITH-BASE) (mode FORMULATED) (params $? target-mps ?rs $?))
  =>
  (modify ?request (value ?payment-goal-id))
  (retract ?request-offer ?payment-goal)
)

; map requests to goals upon completion of order assembly steps

(defrule goal-request-map-buffer-cap-completed-mount-cap
  "If a MOUNT-CAP goal is completed and the buffer request for the same order is not mapped
   to a fitting BUFFER-CAP goal, create the mapping."
  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED))
  (goal-meta (goal-id ?mount-cap-goal) (order-id ?order-id))
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col $?) (value ACTIVE))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))

  (goal (class BUFFER-CAP) (id ?buffer-cap-goal) (mode RETRACTED) (outcome COMPLETED) (params target-mps ?cs $?))
  (not (wm-fact (key request buffer args? $? cs ?cs $?) (value ?buffer-cap-goal)))
  =>
  (modify ?request (value ?buffer-cap-goal))
)

(defrule goal-request-map-discard-completed-mount-cap-to-discard-goal
  "If a MOUNT-CAP goal is completed and the discard request for the same order is not mapped
   to a fitting DISCARD goal, for products that require no ring payments, create the mapping."
  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED))
  (goal-meta (goal-id ?mount-cap-goal) (order-id ?order-id))
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs $?) (value ACTIVE))

  (goal (class DISCARD) (id ?discard-goal) (mode RETRACTED) (outcome COMPLETED) (params $? wp-loc ?cs $?))
  (not (wm-fact (key request discard args? $? cs ?cs $?) (value ?discard-goal)))
  =>
  (modify ?request (value ?discard-goal))
)

(defrule goal-request-map-discard-completed-mount-cap-to-payment-goal
  "If a MOUNT-CAP goal is completed and the discard request for the same order is not mapped
   to a fitting discard or pay-with-carrier goal yet, create aa mapping to a pay with carrier goal."
  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED))
  (goal-meta (goal-id ?mount-cap-goal) (order-id ?order-id))
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs $?) (value ACTIVE))

  (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER) (id ?pay-goal) (mode RETRACTED) (outcome COMPLETED) (params  $? wp-loc ?cs $?))
  (not (wm-fact (key request discard args? $? cs ?cs $?) (value ?pay-goal)))
  =>
  (modify ?request (value ?pay-goal))
)

(defrule goal-request-map-pay-completed-mount-ring
  "If a MOUNT-RING goal that requires payment is completed and a pay request for the same order
   on the same machine is not mapped to a fitting payment goal yet, create the mapping."
  (goal (class MOUNT-RING) (id ?mount-ring-goal) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?rs $? ring-color ?ring-color))
  (goal-meta (goal-id ?mount-ring-goal) (order-id ?order-id))
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring $?) (value ACTIVE))

  (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER|PAY-FOR-RINGS-WITH-BASE) (id ?pay-goal) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?rs $?))
  (not (wm-fact (key request pay args? $? m ?rs $?) (value ?pay-goal)))

  ;only match on those mount-ring goal completions that actually needed payments in the last step
  (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring-color rn ONE|TWO))
  (wm-fact (key wp meta prev-step args? wp ?wp) (value ?ring))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order-id))
  =>
  (modify ?request (value ?pay-goal))
)

; remap requests if order tree root fails

(defrule goal-request-remove-completed-buffer-order-failed
  ;an order root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))

  ;the instruct goal was not used yet
  (goal (id ?instruct-goal-id) (class INSTRUCT-CS-MOUNT-CAP) (mode FORMULATED))
  (goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))

  ;the buffer cap request was completed
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col $?) (value ?buffer-goal-id))
  (goal (id ?buffer-goal-id) (class BUFFER-CAP) (mode FINISHED) (outcome COMPLETED))
  =>
  (assert (wm-fact (key request offer buffer args? ord ?order-id col ?cap-col) (value ?buffer-goal-id)))
  (retract ?request)
)

(defrule goal-request-remove-completed-pay-with-cc-order-failed
  ;an order root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))

  ;the instruct goal was not used yet
  (goal (id ?instruct-goal-id) (class INSTRUCT-RS-MOUNT-RING) (mode FORMULATED))
  (goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))
  (plan-action (id 2) (goal-id ?instruct-goal-id) (action-name ?instruct-action-id))
  (goal (id ?instruct-mount-id) (class INSTRUCT-CS-MOUNT-CAP) (mode FORMULATED))
  (goal-meta (goal-id ?instruct-mount-id) (order-id ?order-id))

  ;the payment request was completed
  ?request-discard <- (wm-fact (key request discard args? ord ?order-id cs ?cs $?) (value ?payment-goal-id))
  ?request-payment <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq $?) (value ?payment-goal-id))
  (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER) (id ?pay-goal) (mode RETRACTED) (outcome COMPLETED))

  ;match ring with payment goals
  (test
    (or
      (and (eq ?instruct-action-id rs-mount-ring1) (eq ?ring RING1))
      (and (eq ?instruct-action-id rs-mount-ring2) (eq ?ring RING2))
      (and (eq ?instruct-action-id rs-mount-ring3) (eq ?ring RING3))
    )
  )
  =>
  (assert (wm-fact (key request offer pay-with-cc args? ord ?order-id rs ?rs cs ?cs ring ?ring ?seq) (value ?payment-goal-id)))
  (retract ?request-discard ?request-payment)
)


(defrule goal-request-remove-completed-discard-order-failed
  ;an order root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))

  ;the instruct goal was not used yet
  (goal (id ?instruct-goal-id) (class INSTRUCT-CS-MOUNT-CAP) (mode FORMULATED))
  (goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))

  ;the discard request was completed
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs $?) (value ?discard-goal-id))
  (goal (id ?discard-goal-id) (class DISCARD) (mode FINISHED) (outcome COMPLETED))
  =>
  (assert (wm-fact (key request offer discard args? ord ?order-id cs ?cs) (value ?discard-goal-id)))
  (retract ?request)
)

(defrule goal-request-remove-completed-payment-order-failed
  ;an order root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))

  ;the instruct goal was not used yet
  (goal (id ?instruct-goal-id) (class INSTRUCT-RS-MOUNT-RING) (mode FORMULATED))
  (goal-meta (goal-id ?instruct-goal-id) (order-id ?order-id))
  (plan-action (id 2) (goal-id ?instruct-goal-id) (action-name ?instruct-action-id))

  ;the payment request was completed
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq $?) (value ?payment-goal-id))
  (goal (class PAY-FOR-RINGS-WITH-BASE) (id ?pay-goal) (mode RETRACTED) (outcome COMPLETED))

  ;match ring with payment goals
  (test
    (or
      (and (eq ?instruct-action-id rs-mount-ring1) (eq ?ring RING1))
      (and (eq ?instruct-action-id rs-mount-ring2) (eq ?ring RING2))
      (and (eq ?instruct-action-id rs-mount-ring3) (eq ?ring RING3))
    )
  )
  =>
  (assert (wm-fact (key request offer pay args? ord ?order-id m ?rs ring ?ring seq ?seq) (value ?payment-goal-id)))
  (retract ?request)
)
