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
(defglobal
  ?*SALIENCE-PRODUCTION-STRATEGY* = -1
  ?*RS-WORKLOAD-THRESHOLD* = 6
  ?*C0-PRODUCTION-THRESHOLD* = 1
  ?*C1-PRODUCTION-THRESHOLD* = 2
  ?*C2-PRODUCTION-THRESHOLD* = 2
  ?*C3-PRODUCTION-THRESHOLD* = 2
  ?*C0-CUTOFF* = 17
  ?*C1-CUTOFF* = 16
  ?*C2-CUTOFF* = 16
  ?*C3-CUTOFF* = 15
  ?*TOTAL-PRODUCTION-THRESHOLD* = 3
  ?*TOTAL-PRODUCTION-THRESHOLD-2ROBOTS* = 2
  ?*TOTAL-PRODUCTION-THRESHOLD-1ROBOT* = 1
  ?*SALIENCE-ORDER-SELECTION* = ?*SALIENCE-HIGH*
  ?*UPDATE-WORKLOAD-TIMEOUT* = 2
  ?*ORDER-SELECTION-RESET-TIMEOUT* = 60
)

(deffunction production-strategy-produce-ahead-check (?gt ?start ?end ?complexity)
  "Checks whether the given time is within the bounds of the produce
  ahead time "
  (bind ?ahead-time 0)

  (if (eq ?complexity C3) then
    (bind ?ahead-time ?*PRODUCE-C3-AHEAD-TIME*))
  (if (eq ?complexity C2) then
    (bind ?ahead-time ?*PRODUCE-C2-AHEAD-TIME*))
  (if (eq ?complexity C1) then
    (bind ?ahead-time ?*PRODUCE-C1-AHEAD-TIME*))
  (if (eq ?complexity C0) then
    (bind ?ahead-time ?*PRODUCE-C0-AHEAD-TIME*))

  (return (and (>= ?gt (max 0 (- ?start ?ahead-time)))
         ;(<= ?gt (max 0 (- ?end ?*DELIVER-AHEAD-TIME*)))
	 (<= ?gt (+ (max 0 (- ?end ?*DELIVER-AHEAD-TIME* ?ahead-time)) ?*ORDER-ACCEPT-TIME*))
  ))
)

(deffunction production-strategy-count-active-orders ()
  "Count the number of order production root nodes that are not retracted."
  (bind ?order-roots 0)
  (do-for-all-facts
    ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:id ?goal-meta:goal-id)
      (neq ?goal-meta:root-for-order nil)
      (neq ?goal:mode RETRACTED)
    )
    (bind ?order-roots (+ 1 ?order-roots))
  )

  (return ?order-roots)
)

(deffunction production-strategy-count-active-orders-of-complexity (?complexity)
  "Count the number of order production root nodes that are not retracted."
  (bind ?order-roots 0)
  (do-for-all-facts
    ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:id ?goal-meta:goal-id)
      (neq ?goal-meta:root-for-order nil)
      (neq ?goal:mode RETRACTED)
    )
    (if (any-factp ((?ord-comp wm-fact))
        (and (wm-key-prefix ?ord-comp:key (create$ domain fact order-complexity))
          (eq ?complexity (wm-key-arg ?ord-comp:key com))
          (eq ?goal-meta:root-for-order (wm-key-arg ?ord-comp:key ord))
        )
      ) then
      (bind ?order-roots (+ 1 ?order-roots))
    )
  )

  (return ?order-roots)
)

(deffunction production-strategy-assert-workload-or-payment-for-machine
  "Creating wm-facts for the order based and overall mps workload"
  (?name ?order-id ?mps ?payments)

  (assert
    (wm-fact (key mps ?name order args? m ?mps ord ?order-id) (type INT)
      (is-list FALSE) (value ?payments))
  )
  (if (not (any-factp ((?wm-fact wm-fact)) (and (wm-key-prefix ?wm-fact:key (create$ mps ?name overall) )
                                              (eq ?mps (wm-key-arg ?wm-fact:key m)))))
  then
    (assert
      (wm-fact (key mps ?name overall args? m ?mps) (type INT)
        (is-list FALSE) (value 0))
    )
  )
)

(defrule production-strategy-update-workload-timeout
  "Trigger an update to to the mps-workload after 2s"
  (declare (salience ?*SALIENCE-LOW*))
  (time $?now)
  ?update-fact <- (wm-fact (key mps workload needs-update) (value FALSE))
  ?timer <- (timer (name workload-update-timer) (time $?t&:(timeout ?now ?t ?*UPDATE-WORKLOAD-TIMEOUT*)) (seq ?seq))
  =>
  (modify ?update-fact (value TRUE))
  (modify ?timer (time ?now) (seq (+ ?seq 1)))
)

(defrule production-strategy-sum-workload
  "Summing up the workload of a mps base on all started order productions"
  (declare (salience ?*SALIENCE-LOW*))
  ?update-fact <- (wm-fact (key mps workload needs-update) (value TRUE))
  =>
  (delayed-do-for-all-facts ((?overall-fact wm-fact)) (wm-key-prefix ?overall-fact:key (create$ mps workload overall))
    (bind ?m (wm-key-arg ?overall-fact:key m))
    (bind ?sum 0)

    (delayed-do-for-all-facts ((?workload-fact wm-fact)) (and (wm-key-prefix ?workload-fact:key (create$ mps workload wp))
                                                (eq ?m (wm-key-arg ?workload-fact:key m)))
      (bind ?sum (+ ?sum ?workload-fact:value))
    )
    (modify ?overall-fact (value ?sum))
  )

  (modify ?update-fact (value FALSE))
)

(defrule production-strategy-init-order-meta-facts
" Calculates the points for each production step, max points and initializes
  more order meta facts.
"
  (declare (salience (+ ?*SALIENCE-GOAL-FORMULATE* 1)))
  ; Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
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
  ; Cap Spec
  (wm-fact (key domain fact cs-color args? m ?mps-cap col ?cap-col))
  ; Order Meta CEs
  (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))
  (not (wm-fact (key order meta points-max args? ord ?order)))
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
           (value ?qd-us))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ~?team-color)
           (value ?qd-them))
  (wm-fact (key refbox game-time) (values ?curr-time $?))
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
           (value ?deadline))
=>
  (bind ?wl workload)
  (bind ?pm (create$ state payments))
  (delayed-do-for-all-facts ((?mps-type domain-fact)) (eq (nth$ 2 ?mps-type:param-values) RS)
    (bind ?payment-sum (calculate-order-payments-sum ?order (nth$ 1 ?mps-type:param-values)))
    (production-strategy-assert-workload-or-payment-for-machine ?wl ?order (nth$ 1 ?mps-type:param-values)
                                        (+
                                           ?payment-sum
                                           (calculate-order-interaction-sum ?order
                                                                            (nth$ 1 ?mps-type:param-values))
                                        )
    )
    (production-strategy-assert-workload-or-payment-for-machine ?pm ?order (nth$ 1 ?mps-type:param-values) 0 )
  )
  (production-strategy-assert-workload-or-payment-for-machine ?wl ?order ?mps-cap 1)

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
  (bind ?points-delivery (delivery-points ?qr
                                          ?qd-us
                                          ?qd-them
                                          ?competitive
                                          ?curr-time
                                          ?deadline))
  (bind ?res (+ ?points-ring1 ?points-ring2 ?points-ring3 ?points-cap
               ?points-delivery))
  (printout t "Order " ?order " gives " ?res " points in total." crlf)
  (assert (wm-fact (key order meta points-steps args? ord ?order) (type INT)
                   (is-list TRUE) (values (create$ ?points-ring1 ?points-ring2
                                                   ?points-ring3 ?points-cap
                                                   ?points-delivery)))
          (wm-fact (key order meta points-max args? ord ?order) (type INT)
                   (is-list FALSE) (value ?res))
          (wm-fact (key order meta step-scored args? ord ?order step RING1)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key order meta step-scored args? ord ?order step RING2)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key order meta step-scored args? ord ?order step RING3)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key order meta step-scored args? ord ?order step CAP)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key order meta estimated-points-total args? ord ?order)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key order meta estimated-time-steps args? ord ?order)
                   (type INT) (is-list TRUE)
                   (values (create$ 0 0 0 0 ?*TIME-DELIVER*)))
          (wm-fact (key order meta started args? ord ?order) (type BOOL) (is-list FALSE) (value FALSE)))
)


(defrule production-strategy-estimate-achievable-points-for-posted-order
" Calculates the points one may score when starting the order assuming the
  required production tasks are all performed within the upper bounds
  as specified in the globals.

  Additional points for prepared caps and supplied additional bases are not
  counted.
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  (not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order)))
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
                    ?end
                    RING1)))))
=>
  (bind ?res (estimate-achievable-points
               ?p-steps
               0
               ?t-steps
               ?game-time
               ?end
               RING1))
  (printout t  "Order " ?order " is expected to score " ?res " out of "
               ?p-total " points" crlf)
  (modify ?om (value ?res))
)


;------------------------- Point/Time Step Updates ---------------------------

(defrule production-strategy-update-time-steps-mount-ring
" Tracks how long the mount ring step for a given order might take.
  This is influenced by:
   - estimated time to mount the ring
   - estimated time to provide additional bases, if there are not enough yet
   - time to get a base, if the order is of complexity C1 and the step is for
     the first ring
     (getting a base and mounting the first ring is part of the same production
      step for the agent: MOUNT-FIRST-RING)
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
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
  (wm-fact (key domain fact rs-filled-with args? m ?mps1 n ?cur1))
  (wm-fact (key domain fact rs-filled-with args? m ?mps2 n ?cur2))
  (wm-fact (key domain fact rs-filled-with args? m ?mps3 n ?cur3))
  (not (wm-fact (key domain fact rs-filled-with args? m ?mps1 n ~?cur1)))
  (not (wm-fact (key domain fact rs-filled-with args? m ?mps2 n ~?cur2)))
  (not (wm-fact (key domain fact rs-filled-with args? m ?mps3 n ~?cur3)))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req1 subtrahend ?cur1 difference ?diff1))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req2 subtrahend ?cur2 difference ?diff2))
  (wm-fact (key domain fact rs-sub
            args? minuend ?req3 subtrahend ?cur3 difference ?diff3))
  ; Order Meta CEs
  ?et-steps <- (wm-fact (key order meta estimated-time-steps args? ord ?order)
                        (values $?timelist))
  (test (not (and
         (eq (nth$ (order-steps-index RING1) ?timelist)
                   (* (bool-to-int (not (eq ?col-r1 RING_NONE)))
                      (+ (* (sym-to-int ?diff1) ?*TIME-FILL-RS*)
                         ?*TIME-MOUNT-RING*
                         ?*TIME-GET-BASE*)))
         (eq (nth$ (order-steps-index RING2) ?timelist)
                   (* (bool-to-int (not (eq ?col-r2 RING_NONE)))
                      (+ (* (sym-to-int ?diff2) ?*TIME-FILL-RS*)
                         ?*TIME-MOUNT-RING*)))
         (eq (nth$ (order-steps-index RING3) ?timelist)
                   (* (bool-to-int (not (eq ?col-r3 RING_NONE)))
                      (+ (* (sym-to-int ?diff3) ?*TIME-FILL-RS*)
                         ?*TIME-MOUNT-RING*))))))
=>
  (modify ?et-steps (values (replace$
                              ?timelist
                              (order-steps-index RING1)
                              (order-steps-index RING3)
                              (* (bool-to-int (not (eq ?col-r1 RING_NONE)))
                                 (+ (* (sym-to-int ?diff1) ?*TIME-FILL-RS*)
                                    ?*TIME-MOUNT-RING*
                                    ?*TIME-GET-BASE*))
                              (* (bool-to-int (not (eq ?col-r2 RING_NONE)))
                                 (+ (* (sym-to-int ?diff2) ?*TIME-FILL-RS*)
                                    ?*TIME-MOUNT-RING*))
                              (* (bool-to-int (not (eq ?col-r3 RING_NONE)))
                                 (+ (* (sym-to-int ?diff3) ?*TIME-FILL-RS*)
                                    ?*TIME-MOUNT-RING*)))))
)


(defrule production-strategy-update-time-steps-mount-cap
" Tracks how long the mount cap step for a given order might take.
  This is influenced by:
   - estimated time to mount the cap
   - estimated time to buffer the cap, if it is not buffered already
   - time to get a base, if the order is of complexity 0
     (getting a base and mounting a cap is part of the same production step
      for the agent: PRODUCE-C0)
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  ; Order CEs
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  (not (wm-fact (key domain fact order-cap-color
                 args? ord ?order col ~?cap-col)))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
  ; Order Meta CEs
  ?et-steps <- (wm-fact (key order meta estimated-time-steps args? ord ?order)
                        (values $?timelist))
  (or (and (wm-fact (key domain fact cs-buffered args? m ? col ?cap-col))
           (test (not (eq (nth$ (order-steps-index CAP) ?timelist)
                      (+ ?*TIME-MOUNT-CAP*
             (* (bool-to-int (eq ?com C0))
                             ?*TIME-GET-BASE*))))))
       (and (not (wm-fact (key domain fact cs-buffered
                           args? m ? col ?cap-col)))
            (test (not (eq (nth$ (order-steps-index CAP) ?timelist)
                  (+ ?*TIME-MOUNT-CAP*
                     ?*TIME-RETRIEVE-CAP*
                     (* (bool-to-int (eq ?com C0))
                         ?*TIME-GET-BASE*)))))))
=>
  (bind ?buffer-cap
          (* (bool-to-int
               (not (any-factp ((?wm wm-fact))
                      (and (wm-key-prefix ?wm:key
                             (create$ domain fact cs-buffered))
                           (eq (wm-key-arg ?wm:key col) ?cap-col)))))
             ?*TIME-RETRIEVE-CAP*))
  (modify ?et-steps (values (replace$ ?timelist
                                      (order-steps-index CAP)
                                      (order-steps-index CAP)
                                      (+ ?buffer-cap
                                         ?*TIME-MOUNT-CAP*
                             (* (bool-to-int (eq ?com C0))
                                            ?*TIME-GET-BASE*)))))
)


(defrule production-strategy-update-delivery-points
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
           (value ?qd-us))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ~?team-color)
           (value ?qd-them))
  (wm-fact (key refbox game-time) (values ?curr-time $?))
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
           (value ?deadline))
  ; Order Meta CEs
  (wm-fact (key order meta competitive args? ord ?order) (value ?competitive))
  ?ps <- (wm-fact (key order meta points-steps args? ord ?order)
                  (values $?pointlist&:(neq (nth$ (order-steps-index DELIVER)
                                                  ?pointlist)
                                            (delivery-points ?qr
                                                             ?qd-us
                                                             ?qd-them
                                                             ?competitive
                                                             ?curr-time
                                                             ?deadline))))
=>
  (modify ?ps (values (replace$ ?pointlist
                                (order-steps-index DELIVER)
                                (order-steps-index DELIVER)
                                (delivery-points ?qr
                                                 ?qd-us
                                                 ?qd-them
                                                 ?competitive
                                                 ?curr-time
                                                 ?deadline))))
)


(defrule production-strategy-update-past-deadline-points
" Points for order steps are only awarded if they are performed within the
  deadline (except for delivery)  and the order is not fulfilled yet. Therefore
  reduce the point-step list accordingly.
"
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? ord ?order team ?team-color)
           (value ?qd-us))
  (wm-fact (key refbox game-time) (values ?curr-time $?))
  (wm-fact (key refbox order ?order delivery-end) (type UINT)
           (value ?deadline))
  (test (or (<= ?qr ?qd-us) (< ?deadline ?curr-time)))
  ; Order Meta CEs
  ?ps <- (wm-fact (key order meta points-steps args? ord ?order)
                  (values $?pointlist&:(neq (nth$ (order-steps-index CAP)
                                                  ?pointlist)
                                            0)))
=>
  (modify ?ps (values (replace$ (replace$ ?pointlist
                                          (order-steps-index CAP)
                                          (order-steps-index CAP)
                                          0)
                                (order-steps-index RING1)
                                (order-steps-index RING3)
                                (create$ 0 0 0))))
)


; ========================= WP Meta Facts ====================================
; Provide additional information to each intermediate workpiece, like
; the already scored points and information about future production steps.


(defrule production-strategy-init-wp-meta-facts
" Initializes facts to track for a given workpiece (started order)
   - current points the workpiece already scored
   - estimated total points the workpiece can score
   - type of the next step that has to be performed
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  ; Order CEs
  (wm-fact (key domain fact order-complexity args? ord ?order com ?com))
  ; Order Meta CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (not (wm-fact (key wp meta points-current args? wp ?wp)))
  (wm-fact (key order meta estimated-points-total args? ord ?order)
           (value ?ep-total))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color ))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring1-color $?))
=>
  (bind ?curr-step RING1)
  (bind ?curr-machine ?rs)
  (if (eq ?com C0) then
    (bind ?curr-step CAP)
    (bind ?curr-machine ?cs)
  )

  (assert (wm-fact (key wp meta points-current args? wp ?wp) (type INT)
                   (is-list FALSE) (value 0))
          (wm-fact (key wp meta next-step args? wp ?wp)
                   (type SYMBOL) (is-list FALSE) (value ?curr-step))
          (wm-fact (key wp meta prev-step args? wp ?wp)
                   (type SYMBOL) (is-list FALSE) (value NONE))
          (wm-fact (key wp meta estimated-points-total args? wp ?wp)
                   (type INT) (is-list FALSE) (value ?ep-total))
          (wm-fact (key wp meta next-machine args? wp ?wp)
                   (type SYMBOL) (is-list FALSE) (value ?curr-machine))
  )
  (delayed-do-for-all-facts ((?workload wm-fact))
    (and
      (wm-key-prefix ?workload:key (create$ mps workload order))
      (eq (wm-key-arg ?workload:key ord) ?order)
    )
    (assert (wm-fact (key mps workload wp args? m (wm-key-arg ?workload:key m) wp ?wp) (type INT)
      (is-list FALSE) (value ?workload:value))
    )
  )
)



(defrule production-strategy-update-next-step
" Keeps track of the workpiece progress by checking which step has to be
  performed next.
  This information together with the point-steps and estimated-time-steps
  order meta facts can be used to obtain the point gain and estimated time
  consumption that the next step will cause.
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact wp-at args? wp ?wp m ? side OUTPUT))
  ; WP CEs
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?wp-col-r1))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?wp-col-r2))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?wp-col-r3))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?wp-cap-col))
  (not (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ~?wp-col-r1)))
  (not (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ~?wp-col-r2)))
  (not (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ~?wp-col-r3)))
  (not (wm-fact (key domain fact wp-cap-color args? wp ?wp col ~?wp-cap-col)))
  ; Order CEs
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?col-r1))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?col-r2))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?col-r3))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-col))
  ; MPS CEs
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?col-r1 rn ?rc1))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?col-r2 rn ?rc2))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?col-r3 rn ?rc3))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  ; WP Meta CEs
  ?ns <- (wm-fact (key wp meta points-current args? wp ?wp) (value ?p-curr))
  ?wm <- (wm-fact (key wp meta next-step args? wp ?wp)
                   (value ?curr-step))
  ?ps <- (wm-fact (key wp meta prev-step args? wp ?wp)
                   (value ?last-step))
  ?nm <- (wm-fact (key wp meta next-machine args? wp ?wp)
                   (value ?curr-machine))
  ; Order Meta CE
  (wm-fact (key order meta points-steps args? ord ?order) (values $?p-list))
  ?ss <- (wm-fact (key order meta step-scored args? ord ?order step ?curr-step)
                  (value ?scored))
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
  (bind ?new-machine ?ds)
  (if (not (eq ?wp-col-r1 ?col-r1))
    then
      (bind ?new-step RING1)
      (bind ?new-machine ?rs1)
    else
      (if (not (eq ?wp-col-r2 ?col-r2))
        then
          (bind ?new-step RING2)
          (bind ?new-machine ?rs2)
        else
          (if (not (eq ?wp-col-r3 ?col-r3))
            then
              (bind ?new-step RING3)
              (bind ?new-machine ?rs3)
            else
              (if (not (eq ?wp-cap-col ?cap-col))
                then
                  (bind ?new-step CAP)
                  (bind ?new-machine ?cs)
              )
          )
      )
  )
  (if (member$ ?curr-step (create$ RING1 RING2 RING3)) then
    (bind ?ring-num (string-to-field (sub-string 5 5 ?curr-step)))
    (if (or (eq ?new-step CAP) (eq ?new-step (sym-cat RING (+ 1 ?ring-num)))) then
      (bind ?used-rs ?rs1)
      (bind ?used-pay ?rc1)
      (if (= ?ring-num 2) then
        (bind ?used-rs ?rs2)
        (bind ?used-pay ?rc2)
      )
      (if (= ?ring-num 3) then
        (bind ?used-rs ?rs3)
        (bind ?used-pay ?rc3)
      )
      (do-for-fact ((?workload wm-fact)) (eq ?workload:key (create$ mps workload wp args? m ?used-rs wp ?wp))
        (modify ?workload (value (- (- ?workload:value (sym-to-int ?used-pay)) 1))) ; all payments were done and the operation itself as well
      )
    )
  )
  (if (and (eq ?curr-step CAP) (eq ?new-step DELIVER)) then
    (do-for-fact ((?workload wm-fact)) (eq ?workload:key (create$ mps workload wp args? m ?cs wp ?wp))
      (modify ?workload (value 0))
    )
  )

  (modify ?ss (value (+ ?scored 1)))
  (modify ?ns (value (+ ?p-curr
                        (nth$ (order-steps-index ?curr-step) $?p-list))))
  (modify ?wm (value ?new-step))
  (modify ?nm (value ?new-machine))
  (modify ?ps (value ?curr-step))
)


(defrule production-strategy-estimate-points-for-started-order
" Calculates the points one may score when finishing a started order assuming
  the required production tasks are all performed within the upper bounds
  as specified in the globals.

  Additional points for prepared caps and supplied additional bases are not
  counted.
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  ; Order CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  ; Time CEs
  (wm-fact (key refbox order ?order delivery-end) (value ?end))
  (wm-fact (key refbox game-time) (values ?game-time $?ms))
  ; Order Meta CEs
  (wm-fact (key order meta points-max args? ord ?order) (value ?p-total))
  (wm-fact (key order meta points-steps args? ord ?order) (values $?p-steps))
  (wm-fact (key order meta next-step args? ord ?order) (value ?next-step))
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
                 ?end
                 ?next-step)))))
=>
  (bind ?res (estimate-achievable-points
               ?p-steps
               ?p-curr
               ?t-steps
               ?game-time
               ?end
               ?next-step))
  (printout error "Workpiece " ?wp " for order " ?order " can score us " ?res
                  " out of " ?p-total "  points in total." crlf)
  (modify ?tpe (value ?res))
)


(defrule production-strategy-reduce-points-already-scored
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox order ?order quantity-requested) (value ?qr))
  ; Order Meta CEs
  (wm-fact (key order meta step-scored args? ord ?order step ?step)
           (value ?scored&:(<= ?qr ?scored)))
  ?ps <- (wm-fact (key order meta points-steps args? ord ?order)
                  (values $?p-steps&:(not (eq (nth$ (order-steps-index ?step)
                                                    ?p-steps)
                                          0))))
=>
  (modify ?ps (values (replace$ ?p-steps
                                (order-steps-index ?step)
                                (order-steps-index ?step)
                                0)))
)

; ========================= Order Preference FIlters =============================

(defrule production-strategy-init-order-preference-facts
  "Initialise the possible and preferred order facts to track orders of each
  complexity for production flow control."
  (not (wm-fact (key strategy meta possible-orders $?)))
  =>
  (assert
    (wm-fact (key strategy meta possible-orders) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta active-orders) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-orders args? filter delivery-ahead) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-orders args? filter delivery-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-orders args? filter workload) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-orders args? filter c0-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-orders args? filter c1-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-orders args? filter c2-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-orders args? filter c3-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-orders args? filter total-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta selected-order args? cond filter) (is-list FALSE) (type SYMBOL) (value nil))
    (wm-fact (key strategy meta selected-order args? cond possible) (is-list FALSE) (type SYMBOL) (value nil))
    (wm-fact (key strategy meta selected-order args? cond fallback) (is-list FALSE) (type SYMBOL) (value nil))
    (wm-fact (key strategy meta production-order-time-limit args? com C0) (value ?*C0-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta production-order-time-limit args? com C1) (value ?*C1-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta production-order-time-limit args? com C2) (value ?*C2-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta production-order-time-limit args? com C3) (value ?*C3-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta production-order-limit args? com TOTAL) (value ?*TOTAL-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta robot-active-count args?) (value 0) (type INT))
  )
)

(defrule production-strategy-count-active-robots
  (or
    (wm-fact (key central agent robot args? r ?any-robot1))
    (wm-fact (key central agent robot-lost args? r ?any-robot2))
  )
  ?strategy-fact <- (wm-fact (key strategy meta robot-active-count args?) (value ?value) (type INT))
  =>
  (bind ?count-robots 0)
  (bind ?count-lost-robots 0)
  (do-for-all-facts ((?robot-fact wm-fact))
    (wm-key-prefix ?robot-fact:key (create$ central agent robot))

    (bind ?count-robots (+ 1 ?count-robots))
  )
  (do-for-all-facts ((?robot-lost-fact wm-fact))
    (wm-key-prefix ?robot-lost-fact:key (create$ central agent robot-lost ))
    (bind ?count-lost-robots (+ 1 ?count-lost-robots))
  )
  (if (neq ?value (- ?count-robots ?count-lost-robots))
    then
    (modify ?strategy-fact (value (- ?count-robots ?count-lost-robots)))
  )
)

(deffunction production-strategy-get-cutoff-complexity (?comp)
  (switch ?comp
  (case C0 then (return ?*C0-CUTOFF*))
  (case C1 then (return ?*C1-CUTOFF*))
  (case C2 then (return ?*C2-CUTOFF*))
  (case C3 then (return ?*C3-CUTOFF*))
  )
  (return nil)
)

(defrule production-strategy-adapt-total-order-limit
  ?limit <- (wm-fact (key strategy meta production-order-limit args? com TOTAL) (value ?threshold))
  (wm-fact (key strategy meta robot-active-count args?) (value ?value))
  =>
  (if (and (eq ?value 3) (neq ?threshold  ?*TOTAL-PRODUCTION-THRESHOLD*)) then
    (modify ?limit (value ?*TOTAL-PRODUCTION-THRESHOLD*))
  )
  (if (and (eq ?value 2) (neq ?threshold  ?*TOTAL-PRODUCTION-THRESHOLD-2ROBOTS*)) then
    (modify ?limit (value ?*TOTAL-PRODUCTION-THRESHOLD-2ROBOTS*))
  )
  (if (and (eq ?value 1) (neq ?threshold  ?*TOTAL-PRODUCTION-THRESHOLD-1ROBOT*)) then
    (modify ?limit (value ?*TOTAL-PRODUCTION-THRESHOLD-1ROBOT*))
  )
  (if (and (eq ?value 0) (neq ?threshold 0)) then
    (modify ?limit (value 0))
  )
)

(defrule production-strategy-reduce-order-time-limit
  ?limit <- (wm-fact (key strategy meta production-order-time-limit args? com ?comp) (value ~0))
  (wm-fact (key refbox game-time) (values ?gt&:(eq (* 60 (production-strategy-get-cutoff-complexity ?comp)) ?gt)))
  =>
  (modify ?limit (value 0))
)

(defrule production-strategy-append-active-orders
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ?active <- (wm-fact (key strategy meta active-orders) (values $?values))
  ;there is a root goal for an existing order
  (goal (id ?root) (mode ~RETRACTED))
  (goal-meta (goal-id ?root) (root-for-order ?order-id))
  (wm-fact (key domain fact order-complexity args? ord ?order-id $?))
  ;it is not in the active list yet
  (test (not (member$ ?order-id ?values)))
  =>
  (modify ?active (values $?values ?order-id))
)

(defrule production-strategy-remove-from-active-orders
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ;there is a retracted root goal
  (goal (id ?root) (mode RETRACTED))
  (goal-meta (goal-id ?root) (root-for-order ?order-id))
  ;the order is in the active list
  ?active <- (wm-fact (key strategy meta active-orders) (values $?values&:(member$ ?order-id ?values)))
  =>
  (modify ?active (values (delete-member$ ?values ?order-id)))
)

(defrule production-strategy-append-possible-orders
  "An order is possible if it's not been fulfilled yet and if the machine occupancy
  allows it to be pursued."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ;facts to modify
  ?poss <- (wm-fact (key strategy meta possible-orders) (values $?values))
  ;meta information
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ;neither delivered, nor started
  (wm-fact (key domain fact quantity-delivered args? ord ?order-id team ?team-color) (value ?quant-del))
  (wm-fact (key refbox order ?order-id quantity-requested) (value ?quant-req&:(> ?quant-req ?quant-del)))

  (not (goal-meta (root-for-order ?order-id)))
  ;it is not possible yet
  (test (not (member$ ?order-id ?values)))
  =>
  (modify ?poss (values $?values ?order-id))
)

(defrule production-strategy-remove-from-possible-orders-active
  "An order that has been started, fulfilled is not possible anymore."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?poss <- (wm-fact (key strategy meta possible-orders) (values $?values&:(member$ ?order-id ?values)))
  (or
    (and
      (wm-fact (key domain fact quantity-delivered args? ord ?order-id team ?team-color) (value ?quant-del))
      (wm-fact (key refbox order ?order-id quantity-requested) (value ?quant-req&:(= ?quant-req ?quant-del)))
    )
    (goal-meta (root-for-order ?order-id))
  )
  =>
  (modify ?poss (values (delete-member$ ?values ?order-id)))
)


;filter delivery-ahead
(defrule production-strategy-filter-orders-delivery-ahead-add
  "Add an order to this filter if its production ahead window is open and isn't closed yet."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter delivery-ahead)
                        (values $?values&:(not (member$ ?order-id ?values))))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ;filter condition
  (wm-fact (key refbox order ?order-id delivery-begin) (value ?begin))
  (wm-fact (key refbox order ?order-id delivery-end) (value ?end))
  (wm-fact (key refbox game-time) (values ?gt $?))
  (test (production-strategy-produce-ahead-check ?gt ?begin ?end ?comp))
  =>
  (modify ?filtered (values $?values ?order-id))
)

(defrule production-strategy-filter-orders-delivery-ahead-remove
  "Remove an order from this filter if its production ahead window has finally closed."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter delivery-ahead)
                        (values $?values&:(member$ ?order-id ?values)))
  (or
    (not (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?)))
    (and
      ;reverse filter condition
      (wm-fact (key refbox order ?order-id delivery-begin) (value ?begin))
      (wm-fact (key refbox order ?order-id delivery-end) (value ?end))
      (wm-fact (key refbox game-time) (values ?gt $?))
      (test (not (production-strategy-produce-ahead-check ?gt ?begin ?end ?comp)))
    )
  )
  =>
  (modify ?filtered (values (delete-member$ ?values ?order-id)))
)

;filter delivery-limit
(defrule production-strategy-filter-orders-delivery-limit-add
  "Add an order to this filter its delivery window end is in the future."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter delivery-limit)
                        (values $?values&:(not (member$ ?order-id ?values))))
  ;filter condition
  (wm-fact (key refbox order ?order-id delivery-end) (value ?end))
  (wm-fact (key refbox game-time) (values ?gt $?))
  (test (< ?gt ?end))
  =>
  (modify ?filtered (values $?values ?order-id))
)

(defrule production-strategy-filter-orders-delivery-limit-remove
  "Remove an order from this filter if its delivery window end has arrived."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter delivery-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (or
    (not (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?)))
    (and
      ;reverse filter condition
      (wm-fact (key refbox order ?order-id delivery-end) (value ?end))
      (wm-fact (key refbox game-time) (values ?gt $?))
      (test (> ?gt ?end))
    )
  )
  =>
  (modify ?filtered (values (delete-member$ ?values ?order-id)))
)

;filter machine workload
(defrule production-strategy-filter-orders-workload-add
  "Add an order to this filter its workload doesn't push the summed workload over any machine's limit."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  (wm-fact (key strategy meta active-orders))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter workload)
                        (values $?values&:(not (member$ ?order-id ?values))))
  ;filter condition
  (not
    (and
      (wm-fact (key mps workload overall args? m ?any-rs) (value ?workload))
      (wm-fact (key mps workload order args? m ?any-rs ord ?order-id) (value ?added-workload))
      (test (> (+ ?workload ?added-workload) ?*RS-WORKLOAD-THRESHOLD*))
    )
  )
  =>
  (modify ?filtered (values $?values ?order-id))
)

(defrule production-strategy-filter-orders-workload-remove
  "Remove an order from this filter if its workload would push the summed workload over the limit."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter workload)
                        (values $?values&:(member$ ?order-id ?values)))
  (or
    (not (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?)))
    (and
      (wm-fact (key mps workload overall args? m ?any-rs) (value ?workload))
      (wm-fact (key mps workload order args? m ?any-rs ord ?order-id) (value ?added-workload))
      (test (> (+ ?workload ?added-workload) ?*RS-WORKLOAD-THRESHOLD*))
    )
  )
  =>
  (modify ?filtered (values (delete-member$ ?values ?order-id)))
)

;filter c0 limit
(defrule production-strategy-filter-orders-c0-limit-add
  "Add an order to this filter if there is less than the threshold of active C0 orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  (wm-fact (key strategy meta active-orders))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c0-limit)
                        (values $?values&:(not (member$ ?order-id ?values))))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
 (wm-fact (key strategy meta production-order-time-limit args? com C0) (value ?limit))
 ;filter condition
  (or
    (and
      (test (eq ?comp C0))
      (test (> ?limit (production-strategy-count-active-orders-of-complexity C0)))
    )
    (test (neq ?comp C0))
  )
  =>
  (modify ?filtered (values $?values ?order-id))
)

(defrule production-strategy-filter-orders-c0-limit-remove
  "Remove an order from this filter if there is more than the threshold of active C0 orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c0-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (not (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?order-id)))
)

(defrule production-strategy-filter-order-c0-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com C0))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c0-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (wm-fact (key strategy meta production-order-time-limit args? com C0) (value ?limit))
  (test (<= ?limit (production-strategy-count-active-orders-of-complexity C0)))
  =>
  (modify ?filtered (values ))
)

;filter c1 limit
(defrule production-strategy-filter-orders-c1-limit-add
  "Add an order to this filter if there is less than the threshold of active c1 orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  (wm-fact (key strategy meta active-orders))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c1-limit)
                        (values $?values&:(not (member$ ?order-id ?values))))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  (wm-fact (key strategy meta production-order-time-limit args? com C1) (value ?limit))
  ;filter condition
  (or
    (and
      (test (eq ?comp C1))
      (test (> ?limit (production-strategy-count-active-orders-of-complexity C1)))
    )
    (test (neq ?comp C1))
  )
  =>
  (modify ?filtered (values $?values ?order-id))
)

(defrule production-strategy-filter-orders-c1-limit-remove
  "Remove an order from this filter if there is more than the threshold of active c1 orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c1-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (not (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?order-id)))
)

(defrule production-strategy-filter-order-c1-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com C1))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c1-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (wm-fact (key strategy meta production-order-time-limit args? com C1) (value ?limit))
  (test (<= ?limit (production-strategy-count-active-orders-of-complexity C1)))
  =>
  (modify ?filtered (values ))
)

;filter c2 limit
(defrule production-strategy-filter-orders-c2-limit-add
  "Add an order to this filter if there is less than the threshold of active c2 orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  (wm-fact (key strategy meta active-orders))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c2-limit)
                        (values $?values&:(not (member$ ?order-id ?values))))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  (wm-fact (key strategy meta production-order-time-limit args? com C2) (value ?limit))
  ;filter condition
  (or
    (and
      (test (eq ?comp C2))
      (test (> ?limit (production-strategy-count-active-orders-of-complexity C2)))
    )
    (test (neq ?comp C2))
  )
  =>
  (modify ?filtered (values $?values ?order-id))
)

(defrule production-strategy-filter-orders-c2-limit-remove
  "Remove an order from this filter if there is more than the threshold of active c2 orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c2-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (not (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?order-id)))
)

(defrule production-strategy-filter-order-c2-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com C2))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c2-limit)
                        (values $?values&:(member$ ?order-id ?values)))
   (wm-fact (key strategy meta production-order-time-limit args? com C2) (value ?limit))
  (test (<= ?limit (production-strategy-count-active-orders-of-complexity C2)))
  =>
  (modify ?filtered (values ))
)

;filter c3 limit
(defrule production-strategy-filter-orders-c3-limit-add
  "Add an order to this filter if there is less than the threshold of active c3 orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  (wm-fact (key strategy meta active-orders))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c3-limit)
                        (values $?values&:(not (member$ ?order-id ?values))))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  (wm-fact (key strategy meta production-order-time-limit args? com C3) (value ?limit))
  ;filter condition
  (or
    (and
      (test (eq ?comp C3))
      (test (> ?limit (production-strategy-count-active-orders-of-complexity C3)))
    )
    (test (neq ?comp C3))
  )
  =>
  (modify ?filtered (values $?values ?order-id))
)

(defrule production-strategy-filter-orders-c3-limit-remove
  "Remove an order from this filter if there is more than the threshold of active c3 orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com C3))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c3-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (not (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?order-id)))
)

(defrule production-strategy-filter-order-c3-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com C3))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter c3-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (wm-fact (key strategy meta production-order-time-limit args? com C3) (value ?limit))
  (test (<= ?limit (production-strategy-count-active-orders-of-complexity C3)))
  =>
  (modify ?filtered (values ))
)


;filter total limit
(defrule production-strategy-filter-orders-total-limit-add
  "Add an order to this filter if there is less than the threshold of active total orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  (wm-fact (key strategy meta active-orders))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter total-limit)
                        (values $?values&:(not (member$ ?order-id ?values))))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))

  (wm-fact (key strategy meta production-order-limit args? com TOTAL) (value ?threshold))
  ;filter condition
  (test (> ?threshold (production-strategy-count-active-orders)))
  =>
  (modify ?filtered (values $?values ?order-id))
)

(defrule production-strategy-filter-orders-total-limit-remove-impossible-orders
  "Remove an order from this filter if there is more than the threshold of active total orders"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter total-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (not (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?order-id)))
)

(defrule production-strategy-filter-order-total-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-orders args? filter total-limit)
                        (values $?values&:(member$ ?order-id ?values)))
  (wm-fact (key strategy meta production-order-limit args? com TOTAL) (value ?threshold))
  (test (<= ?threshold (production-strategy-count-active-orders)))
  =>
  (modify ?filtered (values ))
)

(defrule production-strategy-filter-set-selected-order-possible
  "- it is a possible order
   - there is no order of a higher complexity that is also possible"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  (not
    (and
      (wm-fact (key strategy meta possible-orders) (values $? ?o-order-id&:(neq ?o-order-id ?order-id) $?))
      (wm-fact (key domain fact order-complexity args? ord ?o-order-id com ?o-comp))
      (test (< 0 (str-compare ?o-comp ?comp)))
    )
  )

  ?f <- (wm-fact (key strategy meta selected-order args? cond possible) (value ?ex-order-id))
  (or
    (and
      (wm-fact (key domain fact order-complexity args? ord ?ex-order-id com ?ex-comp))
      (test (> 0 (str-compare ?comp ?ex-comp)))
    )
    (test (eq ?ex-order-id nil))
  )
  =>
  (modify ?f (value ?order-id))
)

(defrule production-strategy-filter-unset-selected-order-possible
  " The selected order is not possible anymore (e.g., it was started).
    Clear the selection"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ?f <- (wm-fact (key strategy meta selected-order args? cond possible) (value ?ex-order-id&~nil))
  (wm-fact (key strategy meta possible-orders) (values $?values&:(not (member$ ?ex-order-id ?values))))
  =>
  (modify ?f (value nil))
)

(defrule production-strategy-filter-set-selected-order-filter
  " - it is a possible order
    - it fulfills all the filters
    - there is no order of a higher complexity that fulfills all the filters"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))

  (wm-fact (key strategy meta possible-orders) (values $? ?order-id $?))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?comp))
  (not (wm-fact (key strategy meta filtered-orders $?) (values $?values&:(not (member$ ?order-id ?values)))))
  (not
    (and
      (wm-fact (key strategy meta possible-orders) (values $? ?o-order-id&:(neq ?order-id ?o-order-id) $?))
      (not (wm-fact (key strategy meta filtered-orders $?) (values $?values&:(not (member$ ?o-order-id ?values)))))
      (wm-fact (key domain fact order-complexity args? ord ?o-order-id com ?o-comp))
      (test (< 0 (str-compare ?o-comp ?comp)))
    )
  )

  ?f <- (wm-fact (key strategy meta selected-order args? cond filter) (value ?ex-order-id))
  (or
    (and
      (wm-fact (key domain fact order-complexity args? ord ?ex-order-id com ?ex-comp))
      (test (> 0 (str-compare ?comp ?ex-comp)))
    )
    (test (eq ?ex-order-id nil))
  )
  =>
  (modify ?f (value ?order-id))
)

(defrule production-strategy-filter-set-selected-order-filter-empty
  "There is no order that meets all filters"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ?f <- (wm-fact (key strategy meta selected-order args? cond filter) (value ?order-id&~nil))
  (wm-fact (key strategy meta filtered-orders $?) (values $?values&:(not (member$ ?order-id ?values))))
  =>
  (modify ?f (value nil))
)

(defrule production-strategy-filter-selected-order-filter-empty-fallback
  "None of the orders fulfill all filters"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta selected-order args? cond filter) (value nil))
  (wm-fact (key strategy meta filtered-orders args? filter $?) (values ?any-order $?))
  ?fallback <- (wm-fact (key strategy meta selected-order args? cond fallback) (value ?fallback-order))
  =>
  (bind ?map (create$))
  (bind ?counts (create$))
  (do-for-all-facts ((?filter wm-fact)) (wm-key-prefix ?filter:key (create$ strategy meta filtered-orders))
    (foreach ?order (fact-slot-value ?filter values)
      (if (member$ ?order ?map) then
          (bind ?index (member$ ?order ?map))
          (bind ?count (nth$ ?index ?counts))
          (bind ?counts (replace$ ?counts ?index ?index (+ ?count 1)))
        else
          (bind ?map (create$ ?map ?order))
          (bind ?counts (create$ ?counts 1))
      )
    )
  )
  (bind ?order (nth$ (member$ (nth$ 1 (sort < ?counts)) ?counts) ?map))
  (if (neq ?order ?fallback-order) then
    (modify ?fallback (value ?order))
  )
)

(defrule production-strategy-filter-selected-order-filter-empty-fallback-remove
  "There is at least order that meets all filters"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta selected-order args? cond filter) (value ~nil))
  ?fallback <- (wm-fact (key strategy meta selected-order args? cond fallback) (value ~nil))
  =>
  (modify ?fallback (value nil))
)

(defrule production-strategy-nothing-executable-timer-create
  "All robots have no goals assigned, create a timer to measure the duration of this condition"
  (time $?now)
  (not (timer (name production-strategy-nothing-executable-timer)))

  (not
    (and
      (wm-fact (key central agent robot args? r ?robot))
      (goal (mode DISPATCHED) (id ?goal-id) (class ~MOVE-OUT-OF-WAY&~WAIT-NOTHING-EXECUTABLE) (sub-type SIMPLE))
      (goal-meta (goal-id ?goal-id) (assigned-to ?robot))
    )
  )
  =>
  (assert (timer (name production-strategy-nothing-executable-timer) (time ?now)))
)

(defrule production-strategy-nothing-executable-timer-remove
  "At leats one robots has a goal assigned, remove the timer"
  (wm-fact (key central agent robot args? r ?robot))
  (goal (mode DISPATCHED) (id ?goal-id) (class ~MOVE-OUT-OF-WAY&~WAIT-NOTHING-EXECUTABLE) (sub-type SIMPLE))
  (goal-meta (goal-id ?goal-id) (assigned-to ?robot))

  ?timer <- (timer (name production-strategy-nothing-executable-timer))
  =>
  (retract ?timer)
)

; re-evaluate criteria for orders that did not progress yet
(defrule production-strategy-clear-started-orders
  "Trigger a re-evaluation of orders to pursue every minute"
  (declare (salience ?*SALIENCE-LOW*))
  (time $?now)
  ?reset-fact <- (wm-fact (key order selection reset) (value FALSE))
  ?timer <- (timer (name order-selection-reset-timer) (time $?t&:(timeout ?now ?t ?*ORDER-SELECTION-RESET-TIMEOUT*)) (seq ?seq))
  =>
  (modify ?timer (time ?now) (seq (+ ?seq 1)))
  (modify ?reset-fact (value TRUE))
)

(defrule production-strategy-purge-non-started-order
  (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
  (wm-fact (key order selection reset) (value TRUE))
  ?os <- (wm-fact (key order meta started args? ord ?order) (value TRUE))
  ?parent-fact <- (goal (id ?parent))
  ?parent-meta-fact <- (goal-meta (goal-id ?parent) (root-for-order ?order))
  ; goal was not started yet
  (not (goal (parent ?parent) (mode ~FORMULATED)))
  ?do <- (domain-object (name ?wp-name&:(eq ?wp-name (sym-cat wp- ?order))))
  (not (domain-fact (name wp-usable) (param-values ?wp-name)))
  =>
  (delayed-do-for-all-facts ((?g goal)) (eq ?g:parent ?parent)
    (retract ?g)
  )
  (delayed-do-for-all-facts ((?g goal)) (member$ ?wp-name ?g:params)
    (retract ?g)
  )
  (delayed-do-for-all-facts ((?df domain-fact)) (member$ ?wp-name ?df:param-values)
    (retract ?df)
  )
  (delayed-do-for-all-facts ((?wm wm-fact)) (member$ ?wp-name ?wm:key)
    (retract ?wm)
  )
  (retract ?do ?parent-fact ?parent-meta-fact)
  (modify ?os (value FALSE))
)

(defrule production-strategy-finish-purging-non-started-orders
  (declare (salience ?*SALIENCE-GOAL-EXECUTABLE-CHECK*))
  ?osr <- (wm-fact (key order selection reset) (value TRUE))
  ?update-fact <- (wm-fact (key mps workload needs-update))
  =>
  (modify ?osr (value FALSE))
  (modify ?update-fact (value TRUE))
)


; ========================= Dynamic Priorities =============================

; -- increase priority to clear a CS output

(defrule production-strategy-increase-priority-to-free-cs
  "If there is a WP at the output of a CS and there is a WP for an order
   approaching the CS, increase the priority of a discard goal to free the CS."
  (wm-fact (key domain fact wp-at args? wp ?cc m ?cs side OUTPUT))
  (domain-object (name ?cc) (type cap-carrier))

  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?ord))
  (wm-fact (key domain fact order-complexity args? ord ?ord com ?com))
  (wm-fact (key wp meta next-step args? wp ?wp) (value ?step))

  (or
    (and
      (wm-fact (key wp meta next-step args? wp ?wp) (value CAP))
      (wm-fact (key wp meta next-machine args? wp ?wp) (value ?cs))
    )
    (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side INPUT))
  )
  (not (wm-fact (key strategy meta priority increase free-cs args? wp ?cc mps ?cs) (value ?higher-prio&:(> ?higher-prio (dynamic-prio-from-complexity ?com ?step)))))

  =>
  (bind ?priority (dynamic-prio-from-complexity ?com ?step))
  ; adjust existing goals that already received a priority update
  (bind ?old-prio-val 0)
  (do-for-fact ((?old-prio wm-fact)) (eq ?old-prio:key (create$ strategy meta priority increase free-cs args? wp ?cc mps ?cs))
    (bind ?old-prio-val ?old-prio:value)
    (retract ?old-prio)
  )
  (assert (wm-fact (key strategy meta priority increase free-cs args? wp ?cc mps ?cs) (value ?priority)))
  (bind ?priority (- ?priority ?old-prio-val))
  (delayed-do-for-all-facts ((?goal goal))
    (and
      (eq ?goal:mode FORMULATED)
      (> ?goal:priority ?old-prio-val)
      (or
        (and (eq ?goal:class DISCARD)
             (member$ ?cc ?goal:params)
        )
        (and (eq ?goal:class PAY-FOR-RINGS-WITH-BASE)
             (member$ ?cc ?goal:params)
        )
      )
    )
    (modify ?goal (priority (+ ?goal:priority ?priority)))
  )
)

(defrule production-strategy-revert-priority-to-free-cs
  ?wf <- (wm-fact (key strategy meta priority increase free-cs args? wp ?cc mps ?cs) (value ?priority))
  (not (wm-fact (key domain fact wp-at args? wp ?cc m ?cs side OUTPUT)))
  =>
  (delayed-do-for-all-facts ((?goal goal))
    (and
      (eq ?goal:mode FORMULATED)
      (> ?goal:priority ?priority)
      (or
        (and (eq ?goal:class DISCARD)
             (member$ ?cc ?goal:params)
        )
        (and (eq ?goal:class PAY-FOR-RINGS-WITH-CAP-CARRIER)
             (member$ ?cc ?goal:params)
        )
      )
    )
    (modify ?goal (priority (- ?goal:priority ?priority)))
  )
  (retract ?wf)
)

(defrule production-strategy-apply-priority-to-free-cs
  (wm-fact (key strategy meta priority increase free-cs args? wp ?wp ?mps $?) (value ?prio-increase))
  ?g <- (goal (class DISCARD|PAY-FOR-RINGS-WITH-BASE)
    (mode FORMULATED) (priority ?prio) (params $? wp ?wp $?))
  (test (< ?prio ?prio-increase))
  =>
  (modify ?g (priority (+ ?prio ?prio-increase)))
)

; -- increase priority to buffer a cap

(defrule production-strategy-increase-priority-to-buffer-cs
  "If there is a WP that needs a cap next, make it a priority to buffer the station"
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?ord))
  (domain-fact (name order-complexity) (param-values ?ord ?com))
  (domain-fact (name order-cap-color) (param-values ?ord ?cap-color))

  (domain-fact (name cs-color) (param-values ?cs ?cap-color))
  (domain-fact (name cs-can-perform) (param-values ?cs RETRIEVE_CAP))

  (domain-fact (name order-cap-color) (param-values ?ord ?cap-color))
  ; The order is actually started or a C0
  (or
    (domain-fact (name wp-usable) (param-values ?wp))
    (test (eq ?com C0))
  )
  (wm-fact (key wp meta next-step args? wp ?wp) (value ?step))

  (not (wm-fact (key strategy meta priority increase buffer-cs args? mps ?cs) (value ?higher-prio&:(> ?higher-prio (dynamic-prio-from-complexity ?com ?step)))))
  =>
  (bind ?priority (dynamic-prio-from-complexity ?com ?step))
  ; adjust existing goals that already received a priority update
  (bind ?old-prio-val 0)
  (do-for-fact ((?old-prio wm-fact)) (eq ?old-prio:key (create$ strategy meta priority increase buffer-cs args? mps ?cs))
    (bind ?old-prio-val ?old-prio:value)
    (retract ?old-prio)
  )
  (assert (wm-fact (key strategy meta priority increase buffer-cs args? mps ?cs) (value ?priority)))
  (bind ?priority (- ?priority ?old-prio-val))
  (delayed-do-for-all-facts ((?goal goal))
    (and
      (eq ?goal:mode FORMULATED)
      (eq ?goal:class BUFFER-CAP)
      (> ?goal:priority ?old-prio-val)
      (member$ ?cs ?goal:params)
      (member$ ?cap-color ?goal:params)
    )
    (modify ?goal (priority (+ ?goal:priority ?priority)))
  )
)

(defrule production-strategy-revert-priority-to-buffer-cs
  ?wf <- (wm-fact (key strategy meta priority increase buffer-cs args? mps ?cs) (value ?priority))
  (not (wm-fact (key wp meta next-machine args? wp ?wp) (value ?cs)))
  =>
  (delayed-do-for-all-facts ((?goal goal))
    (and
      (eq ?goal:mode FORMULATED)
      (eq ?goal:class BUFFER-CAP)
      (> ?goal:priority ?priority)
      (member$ ?cs ?goal:params)
    )
    (modify ?goal (priority (- ?goal:priority ?priority)))
  )
  (retract ?wf)
)

(defrule production-strategy-apply-priority-to-buffer-cs
  (wm-fact (key strategy meta priority increase buffer-cs args? mps ?mps) (value ?prio-increase))
  ?g <- (goal (class BUFFER-CS)
    (mode FORMULATED) (priority ?prio) (params $? target-mps ?mps $?))
  (test (< ?prio ?prio-increase))
  =>
  (modify ?g (priority (+ ?prio ?prio-increase)))
)

; -- increase priority to pay for a workpiece that waits

(defrule production-strategy-increase-priority-to-pay-for-wp
  "If a WP needs payment for it's next step, bump all respective payment goals"
  (domain-fact (name mps-type) (param-values ?mps RS))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (domain-fact (name order-complexity) (param-values ?ord ?com))
  (wm-fact (key wp meta next-step args? wp ?wp)
    (value ?ring&:(str-index RING ?ring)))
  (domain-fact (name ?order-ring-color) (param-values ?order ?ring-color))
  (test
    (eq ?order-ring-color (sym-cat order-ring (sub-string 5 5 ?ring) -color))
  )
  (domain-fact (name rs-ring-spec) (param-values ?mps ?ring-color ?ring-spec))
  (domain-fact (name rs-filled-with) (param-values ?mps ?filled-with))
  (test (< (sym-to-int ?filled-with) (sym-to-int ?ring-spec)))

  (not (wm-fact (key strategy meta priority increase pay-for-wp args? wp ? mps ?mps $) (value ?higher-prio&:(> ?higher-prio (dynamic-prio-from-complexity ?com ?ring)))))
  =>
  (bind ?priority (dynamic-prio-from-complexity ?com ?ring))
  ; adjust existing goals that already received a priority update
  (bind ?old-prio-val 0)
  (do-for-fact ((?old-prio wm-fact)) (and (wm-key-prefix ?old-prio:key (create$ strategy meta priority increase pay-for-wp))
    (eq (wm-key-arg ?old-prio:key mps) ?mps))
    (bind ?old-prio-val ?old-prio:value)
    (retract ?old-prio)
  )
  (assert (wm-fact (key strategy meta priority increase pay-for-wp args? wp ?wp mps ?mps ring-col ?ring-color) (value ?priority)))
  (bind ?priority (- ?priority ?old-prio-val))
  (delayed-do-for-all-facts ((?goal goal))
    (and
      (eq ?goal:mode FORMULATED)
      (eq ?goal:class PAY-FOR-RINGS-WITH-BASE)
      (> ?goal:priority ?old-prio-val)
      (member$ ?mps ?goal:params)
    )
    (modify ?goal (priority (+ ?goal:priority ?priority)))
  )
)

(defrule production-strategy-revert-priority-to-pay-for-wp
  ?wf <- (wm-fact (key strategy meta priority increase pay-for-wp args? wp ?wp mps ?mps ring-col ?ring-color) (value ?priority))
  (or
    (not (wm-fact (key wp meta next-machine args? wp ?wp) (value ?mps)))
    (domain-fact (name rs-input-ready-to-mount-ring) (param-values ?mps ?ring-color))
    (and
      (domain-fact (name rs-ring-spec) (param-values ?mps ?ring-color ?ring-spec))
      (domain-fact (name rs-filled-width) (param-values ?mps ?filled-with&:(> (sym-to-int ?filled-with) (sym-to-int ?ring-spec))))
    )
  )
  =>
  (delayed-do-for-all-facts ((?goal goal))
    (and
      (eq ?goal:mode FORMULATED)
      (eq ?goal:class PAY-FOR-RINGS-WITH-BASE)
      (member$ ?mps ?goal:params)
    )
    (modify ?goal (priority (- ?goal:priority ?priority)))
  )
  (retract ?wf)
)

(defrule production-strategy-apply-priority-to-pay-for-wp
  ?g <- (goal (class PAY-FOR-RINGS-WITH-BASE)
    (mode FORMULATED) (priority ?prio) (params $? target-mps ?mps))
  (wm-fact (key strategy meta priority increase pay-for-wp args? wp ? ?mps $?) (value ?prio-increase))
  (test (< ?prio ?prio-increase))
  =>
  (modify ?g (priority (+ ?prio ?prio-increase)))
)

; -- decrease priority for standing orders until game-time 10

(defrule production-strategy-decrease-priority-for-standing-orders
  "If there is an active standing order, decrease the goal priority until half time"
  (wm-fact (key refbox game-time) (values ?curr-time $?))
  (wm-fact (key refbox order ?order-id delivery-end) (type UINT) (value ?deadline))
  (wm-fact (key refbox order ?order-id delivery-begin) (value ?begin))
  (test (and (eq ?deadline ?*FULL-GAME-TIME*) (eq ?begin 0) (< ?curr-time ?*HALF-GAME-TIME*)))
  (wm-fact (key strategy meta active-orders) (values $?values&:(member$ ?order-id ?values)))
  (not (wm-fact (key strategy meta priority decrease standing-order args? order-id ?order-id) (value ?val&:(eq ?val ?*PRODUCTION-STANDING-ORDER-PRIORITY*))))
  =>
  (bind ?priority ?*PRODUCTION-STANDING-ORDER-PRIORITY*)
  (assert (wm-fact (key strategy meta priority decrease standing-order args? order-id ?order-id) (value ?priority)))
  (delayed-do-for-all-facts ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:mode FORMULATED)
      (eq ?goal-meta:goal-id ?goal:id)
      (eq ?goal-meta:order-id ?order-id)
      (>= ?goal:priority ?*PRODUCTION-C0-PRIORITY*)
    )
    (modify ?goal (priority ?priority))
  )
)

(defrule production-strategy-revert-priority-for-standing-orders
  ?wf <- (wm-fact (key strategy meta priority decrease standing-order args? order-id ?order-id))
  (wm-fact (key strategy meta active-orders) (values $? ?order-id $?))
  (wm-fact (key refbox game-time) (values ?curr-time $?))
  (wm-fact (key refbox order ?order-id delivery-begin) (value ?begin))
  (wm-fact (key refbox order ?order delivery-end) (type UINT) (value ?deadline))
  (test (and (eq ?deadline ?*FULL-GAME-TIME*) (eq ?begin 0) (>= ?curr-time ?*HALF-GAME-TIME*)))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order-id))
  (wm-fact (key domain fact order-complexity args? ord ?order-id com ?com))
  =>
  (delayed-do-for-all-facts ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:mode FORMULATED)
      (eq ?goal-meta:goal-id ?goal:id)
      (eq ?goal-meta:order-id ?order-id)
      (eq ?goal:priority ?*PRODUCTION-STANDING-ORDER-PRIORITY*)
    )
    ;figure out the correct "next-step" from the goal and bind it to ?step 
    (if (str-index CAP ?goal:class)
        then (bind ?step CAP)
	else (if (str-index DELIVER ?goal:class)
                 then (bind ?step DELIVER)
                 else (if (str-index RING ?goal:class)
                          then (bind ?step (switch ?goal-meta:ring-nr 
                                           (case ONE then RING1)
                                           (case TWO then RING2)
                                           (case THREE then RING3)
                          ))
                 )
        )
    )
    (if (str-index INSTRUCT ?goal:class) 
        then (bind ?priority (prio-from-complexity ?com))
        else (bind ?priority (dynamic-prio-from-complexity-for-production-orders ?com ?step))
    )
    (modify ?goal (priority ?priority))
  )
  (retract ?wf)
)

(defrule production-strategy-apply-priority-for-standing-orders
  ?g <- (goal (id ?goal-id) (mode FORMULATED) (priority ?prio))
  (goal-meta (goal-id ?goal-id) (order-id ?order-id))
  (wm-fact (key strategy meta priority decrease standing-order args? order-id ?order-id) (value ?lower-priority))
  (test (>= ?prio ?*PRODUCTION-C0-PRIORITY*))
  =>
  (modify ?g (priority ?lower-priority))
)
