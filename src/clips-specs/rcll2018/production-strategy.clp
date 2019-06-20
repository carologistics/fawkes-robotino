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
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  ; Ring Specs CEs
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps1 r ?col-r1 rn ?req1&:(neq ?req1 NA)))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps2 r ?col-r2 rn ?req2&:(neq ?req2 NA)))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps3 r ?col-r3 rn ?req3&:(neq ?req3 NA)))
  ; Order Meta CEs
  (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))
  (not (wm-fact (key order meta points-max args? ord ?order)))
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
  (bind ?res (+ ?points-ring1 ?points-ring2 ?points-ring3 ?points-cap
               ?points-delivery))
  (bind ?res (finalize-points ?res ?competitive ?qr ?qd-us ?qd-them))
  (printout t "Order " ?order " gives " ?res " points in total." crlf)
  (assert (wm-fact (key order meta points-steps args? ord ?order) (type INT)
                   (is-list TRUE) (values (create$ ?points-ring1 ?points-ring2
                                                   ?points-ring3 ?points-cap
                                                   ?points-delivery)))
          (wm-fact (key order meta points-max args? ord ?order) (type INT)
                   (is-list FALSE) (value ?res))
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


(defrule production-strategy-init-wp-meta-facts
" Initializes facts to track for a given workpiece (started order)
   - number of missing rings
   - number of additional bases for missing rings
   - current points the workpiece already scored
   - estimated total points the workpiece can score
   - estimated points the workpiece will score in the next step
   - estimated time it will take to score the next points with the workpiece
"
  ; Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  ; Ring Specs CEs
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps1 r ?col-r1 rn ?req1&:(neq ?req1 NA)))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps2 r ?col-r2 rn ?req2&:(neq ?req2 NA)))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps3 r ?col-r3 rn ?req3&:(neq ?req3 NA)))
  ; Order Meta CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (not (wm-fact (key wp meta points-current args? wp ?wp)))
  (wm-fact (key order meta estimated-points-total args? ord ?order)
           (value ?ep-total))
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key refbox order ?order quantity-delivered ?team-color)
           (value ?qd-us))
  (wm-fact (key refbox order ?order quantity-delivered ~?team-color)
           (value ?qd-them))
=>
  (bind ?curr-step RING1)
  (if (eq ?com C0) then (bind ?curr-step CAP))
  (assert (wm-fact (key wp meta points-current args? wp ?wp) (type INT)
                   (is-list FALSE) (value 0))
          (wm-fact (key wp meta current-step args? wp ?wp)
                   (type SYMBOL) (is-list FALSE) (value ?curr-step))
          (wm-fact (key wp meta estimated-points-total args? wp ?wp)
                   (type INT) (is-list FALSE) (value ?ep-total)))
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
  ; WP Meta CEs
  ?pc <- (wm-fact (key wp meta points-current args? wp ?wp)
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


(defrule production-strategy-update-current-step
" Keeps track of the workpiece progress by checking which step has to be
  performed next.
  This information together with the point-steps and estimated-time-steps
  order meta facts can be used to obtain the point gain and estimated time
  consumption that the next step will cause.
"
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  ; WP CEs
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-col-r3))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?wp-cap-col))
  ; Order CEs
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  ; WP Meta CEs
  ?wm <- (wm-fact (key wp meta current-step args? wp ?wp)
                   (value ?curr-step))
  (test (or
          (and (not (eq ?wp-col-r1 ?col-r1)) (not (eq ?curr-step RING1)))
          (and (eq ?wp-col-r1 ?col-r1)
               (not (eq ?wp-col-r2 ?col-r2))
               (not (eq ?curr-step RING2)))
          (and (eq ?wp-col-r1 ?col-r1)
               (eq ?wp-col-r2 ?col-r2)
               (not (eq ?wp-col-r3 ?col-r3))
               (not (eq ?curr-step RING3)))
          (and (eq ?wp-col-r1 ?col-r1)
               (eq ?wp-col-r2 ?col-r2)
               (eq ?wp-col-r3 ?col-r3)
               (not (eq ?wp-cap-col ?cap-col))
               (not (eq ?curr-step CAP)))
          (and (eq ?wp-col-r1 ?col-r1)
               (eq ?wp-col-r2 ?col-r2)
               (eq ?wp-col-r3 ?col-r3)
               (eq ?wp-cap-col ?cap-col)
               (not (eq ?curr-step DELIVER)))
))
=>
  (bind ?new-step DELIVER)
  (if (not (eq ?wp-col-r1 ?col-r1))
    then
      (bind ?new-step RING1)
    else
      (if (not (eq ?wp-col-r2 ?col-r2))
        then
          (bind ?new-step RING2)
        else
          (if (not (eq ?wp-col-r3 ?col-r3))
            then
              (bind ?new-step RING3)
            else
              (if (not (eq ?wp-cap-col ?cap-col))
                then
                  (bind ?new-step CAP)
              )
          )
      )
  )
  (modify ?wm (value ?new-step))
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
  (wm-fact (key order meta points-max args? ord ?order) (value ?p-total))
  ?om <- (wm-fact (key order meta estimated-points-total args? ord ?order)
                  (value ?tpe&:(not (= ?tpe (estimate-achievable-points
                    ?p-steps
                    0
                    ?t-steps
                    ?game-time
                    ?end)))))
=>
  (bind ?res (estimate-achievable-points
               ?p-steps
               0
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
  (wm-fact (key order meta points-max args? ord ?order) (value ?p-total))
  (wm-fact (key order meta points-steps args? ord ?order) (values $?p-steps))
  (wm-fact (key order meta estimated-time-steps args? ord ?order)
           (values $?t-steps))
  ; WP Meta CEs
  (wm-fact (key wp meta points-current args? wp ?wp) (value ?p-curr))
  ?tpe <- (wm-fact (key wp meta estimated-points-total args? wp ?wp)
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
  (printout error "Workpiece " ?wp " for order " ?order " can score us " ?res
                  " out of " ?p-total "  points in total." crlf)
  (modify ?tpe (value ?res))
)
