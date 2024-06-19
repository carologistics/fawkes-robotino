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

(defrule goal-request-map-buffer-cap-completed-mount-cap
  "If a MOUNT-CAP goal is completed and the buffer request for the same order is not mapped
   to a fitting BUFFER-CAP goal, create the mapping."
  ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status ACTIVE))
  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED))
  (goal-meta (goal-id ?mount-cap-goal) (order-id ?order-id))
  =>
  (modify ?request (values status COMPLETED))
)

; -------------------- Payment and Discard Goals ------------------------

(defrule goal-request-map-pay-completed
  "If there is a completed mount ring goal that used up a payment associatable with a fulfilled request, mark the request as completed."
  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE))
  (goal (class MOUNT-RING) (id ?mount-ring-goal) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?rs $? ring-color ?ring-color))
  (goal-meta (goal-id ?mount-ring-goal) (order-id ?order-id))
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
  (modify ?request (values status COMPLETED))
)

(defrule goal-request-map-discard-completed
  "If there is a completed mount cap goal or discard goal, mark the associated request as completed"
  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio-discard) (values status ACTIVE ))
  (goal (class MOUNT-CAP) (id ?mount-cap-goal) (mode RETRACTED) (outcome COMPLETED) (params $? target-mps ?cs $?))
  (goal-meta (goal-id ?mount-cap-goal) (order-id ?order-id))
  =>
  (modify ?request (values status COMPLETED))
)

;(defrule goal-request-remove-payment-failed
;  ;an order root has failed
;  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
;  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))
;
;  ;it has a request for discarding
;  ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))
;
;  ;the associated goals were completed
;  (not (goal (id ?id&:(member$ ?id ?payment-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
;  =>
;  (assert (wm-fact (key request offer pay args? ord ?order-id m ?rs) (values assigned-to ?payment-goals)))
;  (retract ?request)
;)
;
;(defrule goal-request-remove-discard-failed
;  ;an order root has failed
;  (goal (id ?root-id) (mode FINISHED) (outcome FAILED))
;  (goal-meta (goal-id ?root-id) (root-for-order ?order-id&~nil))
;
;  ;it has a request for discarding
;  ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))
;
;  ;the associated goals were completed
;  (not (goal (id ?id&:(member$ ?id ?discard-goals)) (mode ~RETRACTED) (outcome ~COMPLETED)))
;  =>
;  (assert (wm-fact (key request offer discard args? ord ?order-id cs ?cs) (values assigned-to ?discard-goals)))
;  (retract ?request)
;)


; -------------------- Handling of broken machines ------------------------

; (defrule goal-request-buffer-reset-broken-mps
;   "Assuming the breaking of the machine leads to graceful termination of all goals, we can simply reset the requests"
;   ?request <- (wm-fact (key request buffer args? ord ?order-id col ?cap-col prio ?prio) (values status ACTIVE assigned-to $?buffer-goals))
;   (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
;   (wm-fact (key domain fact mps-state args? m ?cs s BROKEN))
;   (goal (id ?buffer-goal&:(member$ ?buffer-goal ?buffer-goals)) (mode ~FORMULATED))
;   =>
;   (modify ?request (values status OPEN assigned-to))
;   (delayed-do-for-all-facts ((?g goal))
;     (member$ ?g:id ?buffer-goals)
;     (modify ?g (outcome FAILED) (mode RETRACTED))
;   )
; )

; (defrule goal-request-discard-reset-broken-mps
;   "Assuming the breaking of the machine leads to graceful termination of all goals, we can simply reset the requests"
;   ?request <- (wm-fact (key request discard args? ord ?order-id cs ?cs prio ?prio-discard) (values status ACTIVE assigned-to $?discard-goals))
;   (wm-fact (key domain fact mps-state args? m ?cs s BROKEN))
;   (goal (id ?discard-goal&:(member$ ?discard-goal ?discard-goals)) (mode ~FORMULATED))
;   =>
;   (modify ?request (values status OPEN assigned-to))
; )
;
; (defrule goal-request-pay-reset-broken-mps
;   "Assuming the breaking of the machine leads to graceful termination of all goals, we can simply reset the requests"
;   ?request <- (wm-fact (key request pay args? ord ?order-id m ?rs ring ?ring seq ?seq prio ?prio-payment) (values status ACTIVE assigned-to $?payment-goals))
;   (wm-fact (key domain fact mps-state args? m ?rs s BROKEN))
;   (goal (id ?payment-goal&:(member$ ?payment-goal ?payment-goals)) (mode ~FORMULATED))
;   =>
;   (modify ?request (values status OPEN assigned-to))
; )


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
  (bind ?payment-goal (goal-production-assert-pay-for-rings-with-base ?wp ?rs nil))
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
  (bind ?discard-goal (goal-production-assert-discard ?wp ?mps nil))
  (bind ?instruct-goal (goal-production-assert-instruct-ds-discard ?wp ?ds))
  (modify ?request (values status ACTIVE assigned-to (fact-slot-value ?discard-goal id) (fact-slot-value ?instruct-goal id)))
  (modify ?discard-goal (parent ?root-id) (priority 100))
  (modify ?instruct-goal (parent ?instruct-root-id) (priority 100))
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
