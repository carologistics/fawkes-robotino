;---------------------------------------------------------------------------
;  production-strategy.clp - online strategic decision making for the rcll
;
;  Created: Tue 28 April 2019 14:22:17 CET
;  Copyright  2019  Tarik Viehmann <tarik.viehmann@rwth-aachen.de>
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
;
; ======================== Order Meta ========================================
; Provide additional information to each order like for example total and
; expected point gains, time estimates and info about the required production
; steps.


(defrule production-strategy-init-order-meta-facts
" Calculates the points for each production step, total points and number
  of rings needed for a posted order.
"
  ; Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-col))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  ; Ring Specs CEs
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps1 r ?col-r1 rn ?req1&:(neq ?req1 NA)))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps2 r ?col-r2 rn ?req2&:(neq ?req2 NA)))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps3 r ?col-r3 rn ?req3&:(neq ?req3 NA)))
  ; Order Meta CEs
  (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))
  (not (wm-fact (key order meta points-total args? ord ?order)))
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd-us))
  (wm-fact (key refbox order ?order quantity-delivered ~?team-color)
           (value ?qd-them))
=>
  (bind ?rings-needed (string-to-field (sub-string 2 2 (str-cat ?com))))
  (bind ?points-ring1 (+ (* (bool-to-int (> ?rings-needed 0))
                            (ring-req-points ?req1))
                         (* (bool-to-int (= ?rings-needed 1))
                            (last-ring-points ?com))))
  (bind ?points-ring2 (+ (* (bool-to-int (> ?rings-needed 1))
                            (ring-req-points ?req2))
                         (* (bool-to-int (= ?rings-needed 2))
                            (last-ring-points ?com))))
  (bind ?points-ring3 (+ (* (bool-to-int (> ?rings-needed 2))
                            (ring-req-points ?req3))
                         (* (bool-to-int (= ?rings-needed 3))
                            (last-ring-points ?com))))
  (bind ?points-cap ?*POINTS-MOUNT-CAP*)
  (bind ?points-delivery ?*POINTS-DELIVER*)
  (bind ?bases-needed (+ (sym-to-int ?req1)
                         (sym-to-int ?req2)
                         (sym-to-int ?req3)))
  (bind ?res (+ ?points-ring1 ?points-ring2 ?points-ring3 ?points-cap
               ?points-delivery))
  (bind ?res (finalize-points ?res ?competitive ?qr ?qd-us ?qd-them))
  (printout t "Order " ?order " gives " ?res " points in total." crlf)
  (if (> ?rings-needed 0)
    then
      (printout t "It needs " ?rings-needed " rings, they require to feed "
                  ?bases-needed " more bases into ring stations." crlf)
  )
  (assert (wm-fact (key order meta points-steps args? ord ?order) (type INT)
                   (is-list TRUE) (values (create$ ?points-ring1 ?points-ring2
                                                   ?points-ring3 ?points-cap
                                                   ?points-delivery)))
          (wm-fact (key order meta points-total args? ord ?order) (type INT)
                   (is-list FALSE) (value ?res))
          (wm-fact (key order meta points-current args? ord ?order) (type INT)
                   (is-list FALSE) (value 0))
          (wm-fact (key order meta bases-missing args? ord ?order)
                   (type INT) (is-list FALSE) (value ?bases-needed))
          (wm-fact (key order meta rings-missing args? ord ?order)
                   (type INT) (is-list FALSE) (value ?rings-needed))
          (wm-fact (key order meta estimated-points-next-step args? ord ?order)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key order meta estimated-time-next-step args? ord ?order)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key order meta estimated-points-total args? ord ?order)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key order meta estimated-time-steps args? ord ?order)
                   (type INT) (is-list TRUE)
                   (values
                     (create$ (* (bool-to-int (not (= 0 ?points-ring1)))
                                 ?*TIME-MOUNT-RING*)
                              (* (bool-to-int (not (= 0 ?points-ring2)))
                                 ?*TIME-MOUNT-RING*)
                              (* (bool-to-int (not (= 0 ?points-ring3)))
                                 ?*TIME-MOUNT-RING*)
                              ?*TIME-MOUNT-CAP*
                              ?*TIME-DELIVER*))))
)


(defrule production-strategy-current-points-for-started-order
" Calculates the point value of an intermediate product."
  ; WP CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-col-r3))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?wp-cap-col))
  ; Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-col))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd-us))
  (wm-fact (key refbox order ?order quantity-delivered ~?team-color)
           (value ?qd-them))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
           (value ?end&:(> ?end (nth 1 ?game-time))))
  ; Order Meta CEs
  (wm-fact (key order meta points-steps args? ord ?order)
           (values ?p-r1 ?p-r2 ?p-r3 ?p-cap $?))
  (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))
  ?pc <- (wm-fact (key order meta points-current args? ord ?order)
                  ; the current points have changed and the deadline has not
                  ; been met yet
                  (value ?p-curr&:(neq ?p-curr
                    (finalize-points
                       (+ (* (bool-to-int (eq ?wp-col-r1 ?col-r1)) ?p-r1)
                          (* (bool-to-int (eq ?wp-col-r2 ?col-r2)) ?p-r2)
                          (* (bool-to-int (eq ?wp-col-r3 ?col-r3)) ?p-r3)
                          (* (bool-to-int (eq ?wp-cap-col ?cap-col)) ?p-cap))
                       ?competitive ?qr ?qd-us ?qd-them))))
=>
  (bind ?res (finalize-points
               (+ (* (bool-to-int (eq ?wp-col-r1 ?col-r1)) ?p-r1)
                  (* (bool-to-int (eq ?wp-col-r2 ?col-r2)) ?p-r2)
                  (* (bool-to-int (eq ?wp-col-r3 ?col-r3)) ?p-r3)
                  (* (bool-to-int (eq ?wp-cap-col ?cap-col)) ?p-cap))
                ?competitive ?qr ?qd-us ?qd-them))
  (modify ?pc (value ?res))
  (printout t "WP " ?wp " for order " ?order " yields " ?res
              " points if it can be finished." crlf)
)


(defrule production-strategy-bases-missing-for-posted-order
" Calculates the amount of bases missing in any ring station that are required
  to produce a given order that has not been started yet.

  This does not take the consumption of additional bases used within other
  orders into account.
"
  (not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order)))
  ; Order CEs
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  ; Ring Specs CEs
  (wm-fact (key domain fact rs-ring-spec args? m ?mps1 r ?col-r1 rn ?req1))
  (wm-fact (key domain fact rs-ring-spec args? m ?mps2 r ?col-r2 rn ?req2))
  (wm-fact (key domain fact rs-ring-spec args? m ?mps3 r ?col-r3 rn ?req3))
  (wm-fact (key domain fact rs-filled-with args? m ?mps1 n ?cur1))
  (wm-fact (key domain fact rs-filled-with args? m ?mps2 n ?cur2))
  (wm-fact (key domain fact rs-filled-with args? m ?mps3 n ?cur3))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req1 subtrahend ?cur1 difference ?diff1))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req2 subtrahend ?cur2 difference ?diff2))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req3 subtrahend ?cur3 difference ?diff3))
  ; Order Meta CEs
  ?bm <- (wm-fact (key order meta bases-missing args? ord ?order)
                  (value ?rn&:(neq ?rn (+ (sym-to-int ?diff1)
                                          (sym-to-int ?diff2)
                                          (sym-to-int ?diff3)))))
=>
  (modify ?bm (value (+ (sym-to-int ?diff1) (sym-to-int ?diff2)
                                (sym-to-int ?diff3))))
)


(defrule production-strategy-bases-missing-for-started-order
" Calculates the amount of bases missing in any ring station that are required
  to finish an intermediate product for an order.

  This does not take the consumption of additional bases used within other
  orders into account.
"
  ; WP CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-col-r3))
  ; Order CEs
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  ; Ring Specs CEs
  (wm-fact (key domain fact rs-ring-spec args? m ?mps1 r ?col-r1 rn ?req1))
  (wm-fact (key domain fact rs-ring-spec args? m ?mps2 r ?col-r2 rn ?req2))
  (wm-fact (key domain fact rs-ring-spec args? m ?mps3 r ?col-r3 rn ?req3))
  (wm-fact (key domain fact rs-filled-with args? m ?mps1 n ?cur1))
  (wm-fact (key domain fact rs-filled-with args? m ?mps2 n ?cur2))
  (wm-fact (key domain fact rs-filled-with args? m ?mps3 n ?cur3))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req1 subtrahend ?cur1 difference ?diff1))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req2 subtrahend ?cur2 difference ?diff2))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req3 subtrahend ?cur3 difference ?diff3))
  ; Order Meta CEs
  ?om <- (wm-fact (key order meta bases-missing args? ord ?order)
                         (value ?rn&:(neq ?rn
                           (+ (* (sym-to-int ?diff1)
                                 (bool-to-int (neq ?wp-col-r1 ?col-r1)))
                              (* (sym-to-int ?diff2)
                                 (bool-to-int (neq ?wp-col-r2 ?col-r2)))
                              (* (sym-to-int ?diff3)
                                 (bool-to-int (neq ?wp-col-r3 ?col-r3)))))))
=>
  (bind ?res (+ (* (sym-to-int ?diff1)
                   (bool-to-int (neq ?wp-col-r1 ?col-r1)))
                (* (sym-to-int ?diff2)
                   (bool-to-int (neq ?wp-col-r2 ?col-r2)))
                (* (sym-to-int ?diff3)
                   (bool-to-int (neq ?wp-col-r3 ?col-r3)))))
  (modify ?om (value ?res))
  (printout t "WP " ?wp " for order " ?order " requires " ?res
              " additional base(s) to pay for the remaining ring(s)." crlf)
)


(defrule production-strategy-rings-missing-for-started-order
" calculates the amount of rings that have to be mounted to finish
  an intermediate product for an order.
"
  ; WP CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-col-r3))
  ; Order CEs
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  ; Order Meta CEs
  ?om <- (wm-fact (key order meta rings-missing args? ord ?order)
                  (value ?rm&:(neq ?rm
                    (+ (bool-to-int (neq ?wp-col-r1 ?col-r1))
                       (bool-to-int (neq ?wp-col-r2 ?col-r2))
                       (bool-to-int (neq ?wp-col-r3 ?col-r3))))))
=>
  (bind ?res (+ (bool-to-int (neq ?wp-col-r1 ?col-r1))
                (bool-to-int (neq ?wp-col-r2 ?col-r2))
                (bool-to-int (neq ?wp-col-r3 ?col-r3))))
  (modify ?om (value ?res))
  (if (> ?res 0)
    then
      (printout t "WP " ?wp " for order " ?order " needs " ?res
                  " more ring(s)." crlf)
    else
      (printout t "WP " ?wp " for order " ?order " has all rings now." crlf)
  )
)


(defrule production-strategy-estimate-first-step
" Estimate point gains and time bound to finish the first step.
"
  (not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order)))
  ; Order CEs
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  ?om1 <- (wm-fact (key order meta estimated-points-next-step args? ord ?order)
                   (value ?e-p))
  ?om2 <- (wm-fact (key order meta estimated-time-next-step args? ord ?order)
                   (value ?e-t))
  (test (or (neq ?e-p ?*POINTS-MOUNT-CAP*) (neq ?e-t ?*TIME-MOUNT-CAP*)))
=>
  (modify ?om1 (value ?*POINTS-MOUNT-CAP*))
  (modify ?om2 (value ?*TIME-MOUNT-CAP*))
)


(defrule production-strategy-estimate-next-step-mount-cap
" Estimate point gains and time bound to finish the next step.
  Next Step is to mount a cap.
"
  ; WP CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?col-r3))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  ; Order CEs
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  ; Order Meta CEs
  ?om1 <- (wm-fact (key order meta estimated-points-next-step args? ord ?order)
                   (value ?e-p))
  ?om2 <- (wm-fact (key order meta estimated-time-next-step args? ord ?order)
                   (value ?e-t))
  (test (or (neq ?e-p ?*POINTS-MOUNT-CAP*) (neq ?e-t ?*TIME-MOUNT-CAP*)))
=>
  (modify ?om1 (value ?*POINTS-MOUNT-CAP*))
  (modify ?om2 (value ?*TIME-MOUNT-CAP*))
)


(defrule production-strategy-estimate-next-step-mount-ring
" Estimate point gains and time bound to finish the next step.
  Next Step is to mount a ring.
"
  ; WP CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-col-r3))
  ; Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  ; Ring Specs CEs
  (wm-fact (key domain fact rs-ring-spec args? m ?mps r ?next-col rn ?req))
  ; next rings color needs ?req additional bases
  (test (or
            (and (eq ?wp-col-r1 ?col-r1)
                 (eq ?wp-col-r2 ?col-r2)
                 (neq ?wp-col-r3 ?col-r3)
                 (eq ?next-col ?col-r3))
            (and (eq ?wp-col-r1 ?col-r1)
                 (neq ?wp-col-r2 ?col-r2)
                 (eq ?next-col ?col-r2))
            (and (neq ?wp-col-r1 ?col-r1)
                 (eq ?next-col ?col-r1))))
  ; Order Meta CEs
  ?om1 <- (wm-fact (key order meta estimated-points-next-step args? ord ?order)
                   (value ?e-p))
  ?om2 <- (wm-fact (key order meta estimated-time-next-step args? ord ?order)
                   (value ?e-t))
  (test (or (neq ?e-p (+ (ring-req-points ?req)
                      (* (last-ring-points ?com)
                         (bool-to-int (and (eq ?wp-col-r1 ?col-r1)
                                           (eq ?wp-col-r2 ?col-r2)
                                           (eq ?wp-col-r3 ?col-r3))))))
            (neq ?e-t ?*TIME-MOUNT-RING*)))
=>
  (modify ?om1 (value (+ (ring-req-points ?req)
                         (* (last-ring-points ?com)
                            (bool-to-int (and (eq ?wp-col-r1 ?col-r1)
                                              (eq ?wp-col-r2 ?col-r2)
                                              (eq ?wp-col-r3 ?col-r3)))))))
  (modify ?om2 (value ?*TIME-MOUNT-RING*))
)


(defrule production-strategy-estimate-achievable-points-for-posted-order
" Calculates the points one may score when starting the order assuming the
  required production tasks are all performed within the upper bounds
  as specified in the globals.

  Additional points for prepared caps and supplied additional bases are not
  counted.
"
  (not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order)))
  ; Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-col))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  ; Time CEs
  (wm-fact (key refbox order ?order delivery-end) (value ?end&:(> ?end 0)))
  (wm-fact (key refbox game-time) (values ?game-time $?ms))
  ; Order Meta CEs
  (wm-fact (key order meta points-steps args? ord ?order) (values $?p-steps))
  (wm-fact (key order meta estimated-time-steps args? ord ?order)
           (values $?t-steps))
  (wm-fact (key order meta points-total args? ord ?order) (value ?p-total))
  (wm-fact (key order meta points-current args? ord ?order) (value ?p-curr))
  ?om <- (wm-fact (key order meta estimated-points-total args? ord ?order)
                  (value ?tpe&:(not (= ?tpe (estimate-achievable-points
                    ?p-steps
                    ?p-curr
                    ?t-steps
                    ?game-time
                    ?end)))))
=>
  (bind ?res (estimate-achievable-points
               ?p-steps
               ?p-curr
               ?t-steps
               ?game-time
               ?end))
  (printout t  "Order " ?order " is expected to score " ?res " out of "
               ?p-total " points" crlf)
  (modify ?om (value ?res))
)


(defrule production-strategy-estimate-points-for-started-order
" Calculates the points one may score when finishing a started order assuming
  the required production tasks are all performed within the upper bounds
  as specified in the globals.

  Additional points for prepared caps and supplied additional bases are not
  counted.
"
  ; Order CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-col-r3))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?wp-cap-col))
  ; Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-col))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  ; Time CEs
  (wm-fact (key refbox order ?order delivery-end) (value ?end&:(> ?end 0)))
  (wm-fact (key refbox game-time) (values ?game-time $?ms))
  ; Order Meta CEs
  (wm-fact (key order meta rings-missing args? ord ?order)
           (value ?rings-missing))
  (wm-fact (key order meta points-current args? ord ?order) (value ?p-curr))
  (wm-fact (key order meta points-total args? ord ?order) (value ?p-total))
  (wm-fact (key order meta points-steps args? ord ?order) (values $?p-steps))
  (wm-fact (key order meta estimated-time-steps args? ord ?order)
           (values $?t-steps))
  ?tpe <- (wm-fact (key order meta estimated-points-total args? ord ?order)
             (value ?ap&:(not (= ?ap
               (estimate-achievable-points
                 ?p-steps
                 ?p-curr
                 ?t-steps
                 ?game-time
                 ?end)))))
=>
  (bind ?res (estimate-achievable-points
               ?p-steps
               ?p-curr
               ?t-steps
               ?game-time
               ?end))
  (printout error "Order " ?order " can score us " ?res " out of " ?p-total
                  "  points in total." crlf)
  (modify ?tpe (value ?res))
)
