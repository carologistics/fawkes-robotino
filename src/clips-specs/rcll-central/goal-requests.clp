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
  ?request <- (wm-fact (key request buffer args? prod ?product-id col ?cap-col prio ?prio) (values status OPEN assigned-to))
  ?request-offer <- (wm-fact (key request offer buffer args? $? col ?cap-col) (values assigned-to ?buffer-goal-id ?instruct-goal-id))
  =>
  (modify ?request (values status ACTIVE assigned-to ?buffer-goal-id ?instruct-goal-id))
  (retract ?request-offer)
)

(defrule goal-request-assert-buffer-goal
  "If there is an unfulfilled buffer request, create buffer goal."
  ?request <- (wm-fact (key request buffer args? prod ?product-id col ?cap-col prio ?prio) (values status OPEN assigned-to))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  (goal (class SUPPORT-ROOT) (id ?root-id))
  (not (goal (class BUFFER-CAP) (params $? ?cs ?$) (mode ~RETRACTED)))
  =>
  (bind ?buffer-goal (goal-production-assert-buffer-cap ?cs ?cap-col nil))
  (bind ?instruct-goal (goal-production-assert-instruct-cs-buffer-cap ?cs ?cap-col nil))
  (modify ?buffer-goal (parent ?root-id))
  (modify ?instruct-goal (parent ?instruct-root-id))
)

(defrule goal-request-assign-request-to-buffer-goal
  "If there is an unfulfilled buffer request, create buffer goal."
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status OPEN assigned-to))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  ?buffer-goal <- (goal (id ?buffer-id) (class BUFFER-CAP) (params $? ?cs $?) (priority ?less-prio&:(< ?less-prio (string-to-field (str-cat ?prio)))))
  ?instruct-goal <- (goal (id ?instruct-id) (class INSTRUCT-CS-BUFFER-CAP) (params $? ?cs $?))
  ; and there is no offer
  (not (wm-fact (key request offer buffer args? $? col ?cap-col) (values assigned-to $?)))
  =>
  (bind ?buffer-goal (goal-production-assert-buffer-cap ?cs ?cap-col ?product-id))
  (bind ?instruct-goal (goal-production-assert-instruct-cs-buffer-cap ?cs ?cap-col ?product-id))
  (modify ?request (values status ACTIVE assigned-to (fact-slot-value ?buffer-goal id) (fact-slot-value ?instruct-goal id)))
  (modify ?buffer-goal (parent ?root-id) (priority ?prio))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority ?prio))
)

(defrule goal-request-remap-buffer-offers
  "If there is an unfulfilled buffer request, use a buffer offer."
  ?request <- (wm-fact (key request buffer args? prod ?product-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to ?buffer-goal-id ?instruct-goal-id))
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
  "If a MOUNT-CAP goal is completed and the buffer request for the same product is not mapped
   to a fitting BUFFER-CAP goal, create the mapping."
  ?request <- (wm-fact (key request buffer args? prod ?product-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to ?buffer-goal-id ?instruct-goal-id))
  (goal (id ?buffer-goal-id) (mode RETRACTED) (outcome COMPLETED))
  (goal (id ?instruct-goal-id) (mode RETRACTED) (outcome COMPLETED))

  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED))
  (goal-meta (goal-id ?mount-cap-goal) (product-id ?product-id))
  =>
  (modify ?request (values status COMPLETED assigned-to ?buffer-goal-id ?instruct-goal-id))
)

(defrule goal-request-remove-buffer-failed
  ;an product root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-product ?product-id&~nil))

  ;it has a request for buffering
  ?request <- (wm-fact (key request buffer args? prod ?product-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to ?buffer-goal-id ?instruct-goal-id))

  ;the instruct goal for mounting the cap was not used yet
  (goal (id ?instruct-mount-goal-id) (class INSTRUCT-CS-MOUNT-CAP) (mode FORMULATED))
  (goal-meta (goal-id ?instruct-mount-goal-id) (product-id ?product-id))

  ;the buffer cap request was completed
  (goal (id ?buffer-goal-id) (mode RETRACTED) (outcome COMPLETED))
  (goal (id ?instruct-goal-id) (mode RETRACTED) (outcome COMPLETED))
  =>
  (assert (wm-fact (key request offer buffer args? prod ?product-id col ?cap-col) (values assigned-to ?buffer-goal-id ?instruct-goal-id)))
  (retract ?request)
)

; -------------------- Payment and Discard Goals ------------------------

(defrule goal-request-accept-pay-offers
  "If there are unfulfilled payment offer, use it."
  ?request <- (wm-fact (key request pay args? prod ?product-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status OPEN assigned-to))
  ?request-offer <- (wm-fact (key request offer pay args? $? m ?rs $?) (values assigned-to $?payment-goals))
  =>
  (modify ?request (values status ACTIVE assigned-to ?payment-goals))
  (retract ?request-offer)
)

(defrule goal-request-accept-discard-offers
  "If there are unfulfilled discard offer, use it."
  ?request <- (wm-fact (key request discard args? prod ?product-id cs ?cs prio ?prio-discard) (values status OPEN assigned-to))
  ?request-offer <- (wm-fact (key request offer discard args? $? cs ?cs $?) (values assigned-to $?discard-goals))
  =>
  (modify ?request (values status ACTIVE assigned-to ?discard-goals))
  (retract ?request-offer)
)

(defrule goal-request-assert-pay-with-base-goal
  "If there is an unfulfilled payment request, create a pay-with-base-goal."
  ?request <- (wm-fact (key request pay args? prod ?product-id m ?rs ring ?ring seq ?seq prio ?prio) (values status OPEN assigned-to))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  (not (and (goal (id ?some-id) (class PAY-FOR-RINGS-WITH-BASE) (params $? target-mps ?rs $?) (mode ~RETRACTED))
       (goal (id ?some-other-id&:(neq ?some-id ?some-other-id)) (class PAY-FOR-RINGS-WITH-BASE) (params $? target-mps ?rs $?))))
  =>
  (bind ?wp-base-pay (sym-cat BASE-PAY- (gensym*)))
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-base ?wp-base-pay ?bs INPUT ?rs INPUT ?product-id))
  (bind ?instruct-goal (goal-production-assert-instruct-bs-dispense-base ?wp-base-pay (nth$ (random 1 3) (create$ BASE_RED BASE_BLACK BASE_SILVER)) INPUT nil ?bs))
  (assert
      (domain-object (name ?wp-base-pay) (type workpiece))
      (domain-fact (name wp-unused) (param-values ?wp-base-pay))
      (wm-fact (key domain fact wp-base-color args? wp ?wp-base-pay col BASE_NONE) (type BOOL) (value TRUE))
  )
  (modify ?payment-goal (parent ?root-id))
  (modify ?instruct-goal (parent ?instruct-root-id))
)

(defrule goal-request-assign-request-to-pay-with-base-goal
  "If there is an unfulfilled payment request, create a pay-with-base-goal."
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio) (values status OPEN assigned-to))
  ?pay-goal <- (goal (id ?pay-id) (class PAY-FOR-RINGS-WITH-BASE) (params wp ?wp $? target-mps ?rs) (priority ?less-prio&:(< ?less-prio (string-to-field (str-cat ?prio)))))
  ?pay-instruct-goal <- (goal (id ?pay-instruct-id) (class INSTRUCT-BS-DISPENSE-BASE) (params wp ?wp $?))
  ;and there is no payment offer
  (not (wm-fact (key request offer pay args? $? m ?rs $?) (values assigned-to $?)))
  =>
  (modify ?request (values status ACTIVE assigned-to ?pay-id ?pay-instruct-id))
  (modify ?pay-goal (priority ?prio))
  (modify ?pay-instruct-goal (priority ?prio))
)

(defrule goal-request-assert-discard
  "Create a discard goal for each cap station."
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))

  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

  (goal (class SUPPORT-ROOT) (id ?root-id))
  (goal (class INSTRUCTION-ROOT) (id ?instruct-root-id))
  (not (goal (class DISCARD) (params $? ?cs $?)))
  ;and there is no discard offer
  =>
  (bind ?discard-goal (goal-production-assert-discard UNKNOWN ?cs OUTPUT nil))
  (bind ?instruct-goal (goal-production-assert-instruct-ds-discard UNKNOWN ?ds))
  (do-for-all-facts ((?mtype domain-fact)) (and (eq ?mtype:name mps-type) (member$ RS ?mtype:param-values))
  (bind ?pay-goal-fact (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs OUTPUT (nth$ 1 ?mtype:param-values) INPUT nil))
  (modify ?pay-goal-fact (parent ?root-id))
  )
  (modify ?discard-goal (parent ?root-id))
  (modify ?instruct-goal (parent ?instruct-root-id))
)

(defrule goal-request-assign-request-to-discard-goal
  "If there is an unfulfilled discard request, create a discard goal."
  ?request <- (wm-fact (key request discard args? prod ?product-id cs ?cs prio ?prio) (values status OPEN assigned-to))

  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))

  ?discard-goal <- (goal (id ?discard-id) (class DISCARD) (params wp ?wp wp-loc ?cs $?) (priority ?less-prio&:(< ?less-prio (string-to-field (str-cat ?prio)))))
  ?instruct-goal <- (goal (id ?instruct-id) (class INSTRUCT-DS-DISCARD) (params wp ?wp target-mps ?ds))
  ;and there is no discard offer
  (not (wm-fact (key request offer discard args? $? cs ?cs $?) (values assigned-to $?)))
  =>
  (bind ?discard-goal (goal-production-assert-discard UNKNOWN ?cs OUTPUT ?product-id))
  (bind ?instruct-goal (goal-production-assert-instruct-ds-discard UNKNOWN ?ds))
  (modify ?request (values status ACTIVE assigned-to (fact-slot-value ?discard-goal id) (fact-slot-value ?instruct-goal id)))
  (modify ?discard-goal (parent ?root-id) (priority ?prio))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority ?prio))
)

(defrule goal-request-remap-discard-to-pay-with-cc
  "If there is an active request for discard and an active request for payment-with-base
  but the goals have not been started yet, we can remap them to a pay-with-cc goal."
  ?request-pay <- (wm-fact (key request pay args? prod ?product-id-pay m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))
  ?request-dis <- (wm-fact (key request discard args? prod ?product-id-discard cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))
  (goal (class SUPPORT-ROOT) (id ?root-id))

  ;the assigned goals have not been started yet
  (not (goal (id ?request-goal&:(or (member$ ?request-goal ?payment-goals) (member$ ?request-goal ?discard-goals))) (mode ~FORMULATED)))
  ;the mapping is currently on distinct discard and pay-with-base goals
  (goal (id ?discard-goal&:(member$ ?discard-goal ?discard-goals)) (class DISCARD) (params wp ?wp wp-loc ?cs $?) (priority ?more-prio&:(> ?more-prio 0.0)))
  (goal (id ?payment-goal&:(member$ ?payment-goal ?payment-goals)) (class PAY-FOR-RINGS-WITH-BASE) (params $? target-mps ?rs $?))
  ?pay-with-cc-fact <- (goal (id ?pay-with-cc-goal) (class PAY-FOR-RINGS-WITH-CAP-CARRIER) (params $? wp-loc ?cs $? ))
  ;there is a buffer goal already running (so we can discard soon, to avoid slow-down in payments)
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs $?))
  ;but there is no other payment goal mapped to the same CS to avoid queuing
  (not (and
    (wm-fact (key request pay args? prod ? m ? ring ? seq ? prio ?) (values status ACTIVE assigned-to $?other-payment-goals))
    (goal (id ?other-payment-goal&:(member$ ?other-payment-goal ?other-payment-goals))
          (mode ~RETRACTED)
          (class PAY-FOR-RINGS-WITH-CAP-CARRIER)
          (params $? wp-loc ?cs $?))
  ))
  =>
  (delayed-do-for-all-facts ((?goal goal))
  (or (member$ ?goal:id ?payment-goals) (member$ ?goal:id ?discard-goals))
    (modify ?goal (priority 0.0))
  )

  (bind ?pay-with-cc-goal (goal-production-assert-pay-for-rings-with-cap-carrier UNKNOWN ?cs OUTPUT ?rs INPUT ?product-id-pay))
  (modify ?request-pay (values status ACTIVE assigned-to (fact-slot-value ?pay-with-cc-goal id)))
  (modify ?request-dis (values status ACTIVE assigned-to (fact-slot-value ?pay-with-cc-goal id)))
  (modify ?pay-with-cc-goal (parent ?root-id) (priority ?prio-discard))
)

(defrule goal-request-map-pay-completed
  "If there is a completed mount ring goal that used up a payment associatable with a fulfilled request, mark the request as completed."
  ?request <- (wm-fact (key request pay args? prod ?product-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))
  (goal (class MOUNT-RING) (id ?mount-ring-goal) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?rs $? ring-color ?ring-color))
  (goal-meta (goal-id ?mount-ring-goal) (product-id ?product-id))
  ; the goals associated with the request have been completed
  (not (goal (id ?id&:(member$ ?id ?payment-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
  ; the payments have actually been consumed
  (wm-fact (key product meta wp-for-product args? wp ?wp prod ?product-id))
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
  ?request <- (wm-fact (key request discard args? prod ?product-id cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))
  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?cs $?))
  (goal-meta (goal-id ?mount-cap-goal) (product-id ?product-id))
  ; the goals associated with the request have been completed
  (not (goal (id ?id&:(member$ ?id ?discard-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
  =>
  (modify ?request (values status COMPLETED assigned-to ?discard-goals))
)

(defrule goal-request-remove-payment-failed
  ;an product root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-product ?product-id&~nil))

  ;it has a request for discarding
  ?request <- (wm-fact (key request pay args? prod ?product-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))

  ;the associated goals were completed
  (not (goal (id ?id&:(member$ ?id ?payment-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
  =>
  (assert (wm-fact (key request offer pay args? prod ?product-id m ?rs) (values assigned-to ?payment-goals)))
  (retract ?request)
)

(defrule goal-request-remove-discard-failed
  ;an product root has failed
  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
  (goal-meta (goal-id ?root-id) (root-for-product ?product-id&~nil))

  ;it has a request for discarding
  ?request <- (wm-fact (key request discard args? prod ?product-id cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))

  ;the associated goals were completed
  (not (goal (id ?id&:(member$ ?id ?discard-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
  =>
  (assert (wm-fact (key request offer discard args? prod ?product-id cs ?cs) (values assigned-to ?discard-goals)))
  (retract ?request)
)


; -------------------- Handling of broken machines ------------------------

(defrule goal-request-buffer-reset-broken-mps
  "Assuming the breaking of the machine leads to graceful termination of all goals, we can simply reset the requests"
  ?request <- (wm-fact (key request buffer args? prod ?product-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to $?buffer-goals))
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
  ?request <- (wm-fact (key request discard args? prod ?product-id cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))
  (wm-fact (key domain fact mps-state args? m ?cs s BROKEN))
  (goal (id ?discard-goal&:(member$ ?discard-goal ?discard-goals)) (mode ~FORMULATED))
  =>
  (modify ?request (values status OPEN assigned-to))
)

(defrule goal-request-pay-reset-broken-mps
  "Assuming the breaking of the machine leads to graceful termination of all goals, we can simply reset the requests"
  ?request <- (wm-fact (key request pay args? prod ?product-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))
  (wm-fact (key domain fact mps-state args? m ?rs s BROKEN))
  (goal (id ?payment-goal&:(member$ ?payment-goal ?payment-goals)) (mode ~FORMULATED))
  =>
  (modify ?request (values status OPEN assigned-to))
)


; -------------------- Handle orphaned workpieces ------------------------

(defrule goal-request-take-over-orphaned-workpiece
  "If there is an unhandled payment request that has associated pay-with-base
  goals update the parameters s.t. orphaned workpieces are used to pay."
  (wm-fact (key request pay args? prod ?product-id m ?rs $?) (values status ACTIVE assigned-to ?payment-goal-id ?instruct-goal-id))
  ?instruct-goal <- (goal (id ?instruct-goal-id) (class INSTRUCT-BS-DISPENSE-BASE) (mode FORMULATED))
  ?payment-goal <- (goal (id ?payment-goal-id) (class PAY-FOR-RINGS-WITH-BASE) (mode FORMULATED) (params wp ?wp wp-loc ?bs wp-side ?wp-side target-mps ?target target-side ?target-side))

  (wm-fact (key domain fact wp-at args? wp ?existing-wp m ?bs side ?any-side))
  (not (goal (mode ~RETRACTED) (params $? ?existing-wp $?)))

  ;gather wp facts for cleanup
  ?wp-fact <- (domain-object (name ?wp) (type workpiece))
  ?wp-unused-fact <- (domain-fact (name wp-unused) (param-values ?wp))
  ?wp-color-fact <- (wm-fact (key domain fact wp-base-color args? wp ?wp $?))
  =>
  ;modify the request-associated goals
  (modify ?payment-goal (params wp ?existing-wp wp-loc ?bs wp-side ?wp-side target-mps ?target target-side ?target-side))
  (modify ?instruct-goal (mode FINISHED) (outcome COMPLETED))
  ;clean up the wp facts
  (retract ?wp-fact ?wp-unused-fact ?wp-color-fact)
)
