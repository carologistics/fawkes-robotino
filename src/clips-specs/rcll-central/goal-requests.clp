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
  ?request-discard <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prioDiscard) (value OPEN))
  ?request-payment <- (wm-fact (key request pay args? ord ?order-id m ?rs seq ?seq prio ?prioPayment) (value OPEN))
  ; to avoid potential delays by not being able to remove the cap-carrier, only pair the last payment request
  ; with the discard request
  (not (wm-fact (key request pay args? ord ?order-id $? seq ?other-seq&:(> 0 (str-compare (str-cat ?other-seq) (str-cat ?seq))) $?) (value nil)))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  =>
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs UNKNOWN ?rs INPUT ?order-id))
  (modify ?request-discard (value ACTIVE))
  (modify ?request-payment (value ACTIVE))
  (modify ?payment-goal (parent ?root-id) (priority ?prioPayment))
)

(defrule goal-request-assert-pay-with-cap-carrier-from-active-requests
  "If there is a formulated discard goal and a new payment request comes in
   or a formulated payment goal and a new discard request comes in,
   combine them into one pay-with-cap-carrier goal."
  (goal (class SUPPORT-ROOT) (id ?root-id))
  ?request-discard <- (wm-fact (key request discard args? ord ?orderDiscard cs ?cs prio ?prioDiscard) (value ?valueDiscard))
  ?request-payment <- (wm-fact (key request pay args? ord ?orderPayment m ?rs seq ?seq prio ?prioPayment) (value ?valuePayment))
  ?goal <- (goal (id ?goalId) (class ?goalClass) (mode FORMULATED) (params $?goalParams))

  ;there is no payment request that is open and has a higher sequence number (occurs later)
  (not (wm-fact (key request pay args? $? seq ?other-seq&:(> 0 (str-compare (str-cat ?other-seq) (str-cat ?seq))) $?) (value OPEN)))

  ;only do a swap if there is already a buffer-cpa running and no cap-carrier payment goal to avoid deadlocks
  (goal (class BUFFER-CAP) (mode DISPATCHED))
  (not (goal (class PAY-FOR-RINGS-WITH-CAP-CARRIER) (mode ~RETRACTED)))

  ;one request must be active, one must be open and the goal class must match
  (test
    (or
        (and (eq ?valueDiscard ACTIVE) (eq ?valuePayment OPEN) (eq ?goalClass DISCARD))
        (and (eq ?valueDiscard OPEN) (eq ?valuePayment ACTIVE) (eq ?goalClass PAY-FOR-RINGS-WITH-BASE))
    )
  )
  =>
  ;if the goal is a PAY-FOR-RINGS-WITH-BASE goal, retract the corresponding instruct goal too
  (if (eq ?goalClass PAY-FOR-RINGS-WITH-BASE) then
    (bind ?base-wp (get-param-by-arg ?goalParams wp))
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
  (bind ?new-goal (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs UNKNOWN ?rs INPUT ?orderPayment))
  (modify ?request-discard (value ACTIVE))
  (modify ?request-payment (value ACTIVE))

  (bind ?prio ?prioPayment)
  (if (> 0 (str-compare (str-cat ?prioDiscard) (str-cat ?prioPayment))) then
    (bind ?prio ?prioDiscard)
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
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs seq ?seq prio ?prio) (value OPEN))
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
