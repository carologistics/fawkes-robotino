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
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (value nil))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  =>
  (bind ?buffer-goal (goal-production-assert-buffer-cap ?cs ?cap-col ?order-id))
  (modify ?request (value (fact-slot-value ?buffer-goal id)))
  (modify ?buffer-goal (parent ?root-id) (priority ?prio))
)

(defrule goal-request-assert-pay-with-cap-carrier
  "If there is an unfulfilled discard and payment request of the same goal,
   create a pay with cap-carrier goal to handle both requests."
  ?request-discard <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prioDiscard) (value nil))
  ?request-payment <- (wm-fact (key request pay args? ord ?order-id m ?rs seq ?seq prio ?prioPayment) (value nil))
  ; to avoid potential delays by not being able to remove the cap-carrier, only pair the last payment request
  ; with the discard request
  (not (wm-fact (key request pay args? ord ?order-id $? seq ?other-seq&:(> 0 (str-compare (str-cat ?other-seq) (str-cat ?seq))) $?) (value nil)))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  =>
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs UNKNOWN ?rs INPUT ?order-id))
  (modify ?request-discard (value (fact-slot-value ?payment-goal id)))
  (modify ?request-payment (value (fact-slot-value ?payment-goal id)))
  (modify ?payment-goal (parent ?root-id) (priority ?prioPayment))
)

(defrule goal-request-assert-discard-goal
  "If there is a discard request that is not paired with a goal yet, create a discard goal."
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio) (value nil))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  =>
  (bind ?discard-goal (goal-production-assert-discard UNKNOWN ?cs OUTPUT ?order-id))
  (modify ?request (value (fact-slot-value ?discard-goal id)))
  (modify ?discard-goal (parent ?root-id) (priority ?prio))
)

(defrule goal-request-assert-payment-goal
  "If there is a payment request that is not paired with a goal yet,
   create a pay-with-base goal and a base dispense goal"
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs seq ?seq prio ?prio) (value nil))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  =>
  (bind ?wp-base-pay (sym-cat BASE-PAY- (gensym*)))
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-base ?wp-base-pay ?bs INPUT ?rs INPUT ?order-id))
  (modify ?request (value (fact-slot-value ?payment-goal id)))
  (modify ?payment-goal (parent ?root-id) (priority ?prio))

  (bind ?instruct-goal (goal-production-assert-instruct-bs-dispense-base ?wp-base-pay BASE_RED INPUT ?order-id ?bs))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority ?prio))

  (assert
      (domain-object (name ?wp-base-pay) (type workpiece))
      (domain-fact (name wp-unused) (param-values ?wp-base-pay))
      (wm-fact (key domain fact wp-base-color args? wp ?wp-base-pay col BASE_NONE) (type BOOL) (value TRUE))
  )
)
