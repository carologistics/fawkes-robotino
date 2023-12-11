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
; Provide additional information to each product like for example total and
; expected point gains, time estimates and info about the required production
; steps.
(defglobal
  ?*SALIENCE-PRODUCTION-STRATEGY* = -1
  ?*RS-WORKLOAD-THRESHOLD* = 6
  ?*C0-PRODUCTION-THRESHOLD* = 1
  ?*C1-PRODUCTION-THRESHOLD* = 2
  ?*C2-PRODUCTION-THRESHOLD* = 2
  ?*C3-PRODUCTION-THRESHOLD* = 2
  ?*C0-CUTOFF* = 1700
  ?*C1-CUTOFF* = 1600
  ?*C2-CUTOFF* = 1600
  ?*C3-CUTOFF* = 1500
  ?*TOTAL-PRODUCTION-THRESHOLD* = 3
  ?*TOTAL-PRODUCTION-THRESHOLD-2ROBOTS* = 2
  ?*TOTAL-PRODUCTION-THRESHOLD-1ROBOT* = 1
  ?*SALIENCE-ORDER-SELECTION* = ?*SALIENCE-HIGH*
  ?*UPDATE-WORKLOAD-TIMEOUT* = 2
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
         (<= ?gt (max 0 (- ?end ?*DELIVER-AHEAD-TIME*)))))
)

(deffunction production-strategy-count-active-products ()
  "Count the number of product production root nodes that are not retracted."
  (bind ?product-roots 0)
  (do-for-all-facts
    ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:id ?goal-meta:goal-id)
      (neq ?goal-meta:root-for-product nil)
      (neq ?goal:mode RETRACTED)
    )
    (bind ?product-roots (+ 1 ?product-roots))
  )

  (return ?product-roots)
)

(deffunction production-strategy-count-active-products-of-complexity (?complexity)
  "Count the number of product production root nodes that are not retracted."
  (bind ?product-roots 0)
  (do-for-all-facts
    ((?goal goal) (?goal-meta goal-meta))
    (and
      (eq ?goal:id ?goal-meta:goal-id)
      (neq ?goal-meta:root-for-product nil)
      (neq ?goal:mode RETRACTED)
    )
    (if (any-factp ((?ord-comp wm-fact))
        (and (wm-key-prefix ?ord-comp:key (create$ domain fact product-complexity))
          (eq ?complexity (wm-key-arg ?ord-comp:key com))
          (eq ?goal-meta:root-for-product (wm-key-arg ?ord-comp:key prod))
        )
      ) then
      (bind ?product-roots (+ 1 ?product-roots))
    )
  )

  (return ?product-roots)
)

(deffunction production-strategy-assert-workload-or-payment-for-machine
  "Creating wm-facts for the product based and overall mps workload"
  (?name ?product-id ?mps ?payments)

  (assert
    (wm-fact (key mps ?name product args? m ?mps prod ?product-id) (type INT)
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
  "Summing up the workload of a mps base on all started product productions"
  (declare (salience ?*SALIENCE-LOW*))
  ?update-fact <- (wm-fact (key mps workload needs-update) (value TRUE))
  =>
  (delayed-do-for-all-facts ((?overall-fact wm-fact)) (wm-key-prefix ?overall-fact:key (create$ mps workload overall))
    (bind ?m (wm-key-arg ?overall-fact:key m))
    (bind ?sum 0)

    (delayed-do-for-all-facts ((?workload-fact wm-fact)) (and (wm-key-prefix ?workload-fact:key (create$ mps workload product))
                                                (eq ?m (wm-key-arg ?workload-fact:key m)))
      ;the product has been started and not fulfilled yet
      (if (any-factp ((?product-started wm-fact)) (and (wm-key-prefix ?product-started:key (create$ product meta started))
                                                    (eq (wm-key-arg ?workload-fact:key prod)
                                                        (wm-key-arg ?product-started:key prod))
                                                    (eq ?product-started:value TRUE)))
        then
        (if (not (any-factp ((?product-fulfilled wm-fact)) (and (wm-key-prefix ?product-fulfilled:key (create$ domain fact product-fulfilled))
                                                      (eq (wm-key-arg ?workload-fact:key prod)
                                                          (wm-key-arg ?product-fulfilled:key prod)))))
        then
            (bind ?sum (+ ?sum ?workload-fact:value))
        )
      )
    )
    (modify ?overall-fact (value ?sum))
  )

  (modify ?update-fact (value FALSE))
)

(defrule production-strategy-init-product-meta-facts
" Calculates the points for each production step, max points and initializes
  more product meta facts.
"
  (declare (salience (+ ?*SALIENCE-GOAL-FORMULATE* 1)))
  ; Order CEs
  (wm-fact (key domain fact product-complexity args? prod ?product com ?com))
  (wm-fact (key domain fact product-ring1-color args? prod ?product col ?col-r1))
  (wm-fact (key domain fact product-ring2-color args? prod ?product col ?col-r2))
  (wm-fact (key domain fact product-ring3-color args? prod ?product col ?col-r3))
  (wm-fact (key domain fact product-cap-color args? prod ?product col ?col-cap))
  ; Ring Specs CEs
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps1 r ?col-r1 rn ?req1&:(neq ?req1 NA)))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps2 r ?col-r2 rn ?req2&:(neq ?req2 NA)))
  (wm-fact (key domain fact rs-ring-spec
            args? m ?mps3 r ?col-r3 rn ?req3&:(neq ?req3 NA)))
  ; Cap Spec
  (wm-fact (key domain fact cs-color args? m ?mps-cap col ?col-cap))
  ; Order Meta CEs
  (wm-fact (key product meta competitive args? prod ?product) (value ?competitive))
  (not (wm-fact (key product meta points-max args? prod ?product)))
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox product ?product quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? prod ?product team ?team-color)
           (value ?qd-us))
  (wm-fact (key domain fact quantity-delivered args? prod ?product team ~?team-color)
           (value ?qd-them))
  (wm-fact (key refbox game-time) (values ?curr-time $?))
  (wm-fact (key refbox product ?product delivery-end) (type UINT)
           (value ?deadline))
=>
  (bind ?wl workload)
  (bind ?pm (create$ state payments))
  (delayed-do-for-all-facts ((?mps-type domain-fact)) (eq (nth$ 2 ?mps-type:param-values) RS)
    (bind ?payment-sum (calculate-product-payments-sum ?product (nth$ 1 ?mps-type:param-values)))
    (production-strategy-assert-workload-or-payment-for-machine ?wl ?product (nth$ 1 ?mps-type:param-values)
                                        (+
                                           ?payment-sum
                                           (calculate-product-interaction-sum ?product
                                                                            (nth$ 1 ?mps-type:param-values))
                                        )
    )
    (production-strategy-assert-workload-or-payment-for-machine ?pm ?product (nth$ 1 ?mps-type:param-values) 0 )
  )
  (production-strategy-assert-workload-or-payment-for-machine ?wl ?product ?mps-cap 1)

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
  (printout t "Order " ?product " gives " ?res " points in total." crlf)
  (assert (wm-fact (key product meta points-steps args? prod ?product) (type INT)
                   (is-list TRUE) (values (create$ ?points-ring1 ?points-ring2
                                                   ?points-ring3 ?points-cap
                                                   ?points-delivery)))
          (wm-fact (key product meta points-max args? prod ?product) (type INT)
                   (is-list FALSE) (value ?res))
          (wm-fact (key product meta step-scored args? prod ?product step RING1)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key product meta step-scored args? prod ?product step RING2)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key product meta step-scored args? prod ?product step RING3)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key product meta step-scored args? prod ?product step CAP)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key product meta estimated-points-total args? prod ?product)
                   (type INT) (is-list FALSE) (value 0))
          (wm-fact (key product meta estimated-time-steps args? prod ?product)
                   (type INT) (is-list TRUE)
                   (values (create$ 0 0 0 0 ?*TIME-DELIVER*)))
          (wm-fact (key product meta started args? prod ?product) (type BOOL) (is-list FALSE) (value FALSE)))
)


(defrule production-strategy-estimate-achievable-points-for-posted-product
" Calculates the points one may score when starting the product assuming the
  required production tasks are all performed within the upper bounds
  as specified in the globals.

  Additional points for prepared caps and supplied additional bases are not
  counted.
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  (not (wm-fact (key product meta wp-for-product args? wp ?wp prod ?product)))
  ; Time CEs
  (wm-fact (key refbox product ?product delivery-end) (value ?end&:(> ?end 0)))
  (wm-fact (key refbox game-time) (values ?game-time $?ms))
  ; Order Meta CEs
  (wm-fact (key product meta points-steps args? prod ?product) (values $?p-steps))
  (wm-fact (key product meta estimated-time-steps args? prod ?product)
           (values $?t-steps))
  (wm-fact (key product meta points-max args? prod ?product) (value ?p-total))
  ?om <- (wm-fact (key product meta estimated-points-total args? prod ?product)
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
  (printout t  "Order " ?product " is expected to score " ?res " out of "
               ?p-total " points" crlf)
  (modify ?om (value ?res))
)


;------------------------- Point/Time Step Updates ---------------------------

(defrule production-strategy-update-time-steps-mount-ring
" Tracks how long the mount ring step for a given product might take.
  This is influenced by:
   - estimated time to mount the ring
   - estimated time to provide additional bases, if there are not enough yet
   - time to get a base, if the product is of complexity C1 and the step is for
     the first ring
     (getting a base and mounting the first ring is part of the same production
      step for the agent: MOUNT-FIRST-RING)
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  ; Order CEs
  (wm-fact (key domain fact product-complexity args? prod ?product com ?com))
  (wm-fact (key domain fact product-ring1-color args? prod ?product col ?col-r1))
  (wm-fact (key domain fact product-ring2-color args? prod ?product col ?col-r2))
  (wm-fact (key domain fact product-ring3-color args? prod ?product col ?col-r3))
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
  ?et-steps <- (wm-fact (key product meta estimated-time-steps args? prod ?product)
                        (values $?timelist))
  (test (not (and
         (eq (nth$ (product-steps-index RING1) ?timelist)
                   (* (bool-to-int (not (eq ?col-r1 RING_NONE)))
                      (+ (* (sym-to-int ?diff1) ?*TIME-FILL-RS*)
                         ?*TIME-MOUNT-RING*
                         ?*TIME-GET-BASE*)))
         (eq (nth$ (product-steps-index RING2) ?timelist)
                   (* (bool-to-int (not (eq ?col-r2 RING_NONE)))
                      (+ (* (sym-to-int ?diff2) ?*TIME-FILL-RS*)
                         ?*TIME-MOUNT-RING*)))
         (eq (nth$ (product-steps-index RING3) ?timelist)
                   (* (bool-to-int (not (eq ?col-r3 RING_NONE)))
                      (+ (* (sym-to-int ?diff3) ?*TIME-FILL-RS*)
                         ?*TIME-MOUNT-RING*))))))
=>
  (modify ?et-steps (values (replace$
                              ?timelist
                              (product-steps-index RING1)
                              (product-steps-index RING3)
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
" Tracks how long the mount cap step for a given product might take.
  This is influenced by:
   - estimated time to mount the cap
   - estimated time to buffer the cap, if it is not buffered already
   - time to get a base, if the product is of complexity 0
     (getting a base and mounting a cap is part of the same production step
      for the agent: PRODUCE-C0)
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  ; Order CEs
  (wm-fact (key domain fact product-cap-color args? prod ?product col ?cap-col))
  (not (wm-fact (key domain fact product-cap-color
                 args? prod ?product col ~?cap-col)))
  (wm-fact (key domain fact product-complexity args? prod ?product com ?com))
  ; Order Meta CEs
  ?et-steps <- (wm-fact (key product meta estimated-time-steps args? prod ?product)
                        (values $?timelist))
  (or (and (wm-fact (key domain fact cs-buffered args? m ? col ?cap-col))
           (test (not (eq (nth$ (product-steps-index CAP) ?timelist)
                      (+ ?*TIME-MOUNT-CAP*
             (* (bool-to-int (eq ?com C0))
                             ?*TIME-GET-BASE*))))))
       (and (not (wm-fact (key domain fact cs-buffered
                           args? m ? col ?cap-col)))
            (test (not (eq (nth$ (product-steps-index CAP) ?timelist)
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
                                      (product-steps-index CAP)
                                      (product-steps-index CAP)
                                      (+ ?buffer-cap
                                         ?*TIME-MOUNT-CAP*
                             (* (bool-to-int (eq ?com C0))
                                            ?*TIME-GET-BASE*)))))
)


(defrule production-strategy-update-delivery-points
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox product ?product quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? prod ?product team ?team-color)
           (value ?qd-us))
  (wm-fact (key domain fact quantity-delivered args? prod ?product team ~?team-color)
           (value ?qd-them))
  (wm-fact (key refbox game-time) (values ?curr-time $?))
  (wm-fact (key refbox product ?product delivery-end) (type UINT)
           (value ?deadline))
  ; Order Meta CEs
  (wm-fact (key product meta competitive args? prod ?product) (value ?competitive))
  ?ps <- (wm-fact (key product meta points-steps args? prod ?product)
                  (values $?pointlist&:(neq (nth$ (product-steps-index DELIVER)
                                                  ?pointlist)
                                            (delivery-points ?qr
                                                             ?qd-us
                                                             ?qd-them
                                                             ?competitive
                                                             ?curr-time
                                                             ?deadline))))
=>
  (modify ?ps (values (replace$ ?pointlist
                                (product-steps-index DELIVER)
                                (product-steps-index DELIVER)
                                (delivery-points ?qr
                                                 ?qd-us
                                                 ?qd-them
                                                 ?competitive
                                                 ?curr-time
                                                 ?deadline))))
)


(defrule production-strategy-update-past-deadline-points
" Points for product steps are only awarded if they are performed within the
  deadline (except for delivery)  and the product is not fulfilled yet. Therefore
  reduce the point-step list accordingly.
"
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox product ?product quantity-requested) (value ?qr))
  (wm-fact (key domain fact quantity-delivered args? prod ?product team ?team-color)
           (value ?qd-us))
  (wm-fact (key refbox game-time) (values ?curr-time $?))
  (wm-fact (key refbox product ?product delivery-end) (type UINT)
           (value ?deadline))
  (test (or (<= ?qr ?qd-us) (< ?deadline ?curr-time)))
  ; Order Meta CEs
  ?ps <- (wm-fact (key product meta points-steps args? prod ?product)
                  (values $?pointlist&:(neq (nth$ (product-steps-index CAP)
                                                  ?pointlist)
                                            0)))
=>
  (modify ?ps (values (replace$ (replace$ ?pointlist
                                          (product-steps-index CAP)
                                          (product-steps-index CAP)
                                          0)
                                (product-steps-index RING1)
                                (product-steps-index RING3)
                                (create$ 0 0 0))))
)


; ========================= WP Meta Facts ====================================
; Provide additional information to each intermediate workpiece, like
; the already scored points and information about future production steps.


(defrule production-strategy-init-wp-meta-facts
" Initializes facts to track for a given workpiece (started product)
   - current points the workpiece already scored
   - estimated total points the workpiece can score
   - type of the next step that has to be performed
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  ; Order CEs
  (wm-fact (key domain fact product-complexity args? prod ?product com ?com))
  ; Order Meta CEs
  (wm-fact (key product meta wp-for-product args? wp ?wp prod ?product))
  (not (wm-fact (key wp meta points-current args? wp ?wp)))
  (wm-fact (key product meta estimated-points-total args? prod ?product)
           (value ?ep-total))
  (wm-fact (key domain fact product-cap-color args? prod ?product col ?cap-color))
  (wm-fact (key domain fact product-ring1-color args? prod ?product col ?ring1-color ))
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
)



(defrule production-strategy-update-next-step
" Keeps track of the workpiece progress by checking which step has to be
  performed next.
  This information together with the point-steps and estimated-time-steps
  product meta facts can be used to obtain the point gain and estimated time
  consumption that the next step will cause.
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  (wm-fact (key product meta wp-for-product args? wp ?wp prod ?product))
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
  (wm-fact (key domain fact product-ring1-color args? prod ?product col ?col-r1))
  (wm-fact (key domain fact product-ring2-color args? prod ?product col ?col-r2))
  (wm-fact (key domain fact product-ring3-color args? prod ?product col ?col-r3))
  (wm-fact (key domain fact product-cap-color args? prod ?product col ?cap-col))
  ; MPS CEs
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-col))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs1 r ?col-r1 $?))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs2 r ?col-r2 $?))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs3 r ?col-r3 $?))
  (wm-fact (key refbox team-color) (value ?team-color))
  ; WP Meta CEs
  ?ns <- (wm-fact (key wp meta points-current args? wp ?wp) (value ?p-curr))
  ?wm <- (wm-fact (key wp meta next-step args? wp ?wp)
                   (value ?curr-step))
  ?ps <- (wm-fact (key wp meta prev-step args? wp ?wp)
                   (value ?last-step))
  ?nm <- (wm-fact (key wp meta next-machine args? wp ?wp)
                   (value ?curr-machine))
  ; Order Meta CE
  (wm-fact (key product meta points-steps args? prod ?product) (values $?p-list))
  ?ss <- (wm-fact (key product meta step-scored args? prod ?product step ?curr-step)
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
  (bind ?ds MOCKUP-DS)
  (do-for-fact ((?wf wm-fact)) (wm-key-prefix ?wf:key (create$ domain fact mps-type))
    (eq DS (wm-key-arg ?wf:key t))
    (bind ?ds (wm-key-arg ?wf:key m))
  )
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
  (modify ?ss (value (+ ?scored 1)))
  (modify ?ns (value (+ ?p-curr
                        (nth$ (product-steps-index ?curr-step) $?p-list))))
  (modify ?wm (value ?new-step))
  (modify ?nm (value ?new-machine))
  (modify ?ps (value ?curr-step))
)


(defrule production-strategy-estimate-points-for-started-product
" Calculates the points one may score when finishing a started product assuming
  the required production tasks are all performed within the upper bounds
  as specified in the globals.

  Additional points for prepared caps and supplied additional bases are not
  counted.
"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  ; Order CEs
  (wm-fact (key product meta wp-for-product args? wp ?wp prod ?product))
  ; Time CEs
  (wm-fact (key refbox product ?product delivery-end) (value ?end))
  (wm-fact (key refbox game-time) (values ?game-time $?ms))
  ; Order Meta CEs
  (wm-fact (key product meta points-max args? prod ?product) (value ?p-total))
  (wm-fact (key product meta points-steps args? prod ?product) (values $?p-steps))
  (wm-fact (key product meta next-step args? prod ?product) (value ?next-step))
  (wm-fact (key product meta estimated-time-steps args? prod ?product)
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
  (printout error "Workpiece " ?wp " for product " ?product " can score us " ?res
                  " out of " ?p-total "  points in total." crlf)
  (modify ?tpe (value ?res))
)


(defrule production-strategy-reduce-points-already-scored
  ; Refbox CEs
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key refbox product ?product quantity-requested) (value ?qr))
  ; Order Meta CEs
  (wm-fact (key product meta step-scored args? prod ?product step ?step)
           (value ?scored&:(<= ?qr ?scored)))
  ?ps <- (wm-fact (key product meta points-steps args? prod ?product)
                  (values $?p-steps&:(not (eq (nth$ (product-steps-index ?step)
                                                    ?p-steps)
                                          0))))
=>
  (modify ?ps (values (replace$ ?p-steps
                                (product-steps-index ?step)
                                (product-steps-index ?step)
                                0)))
)

; ========================= Order Preference FIlters =============================

(defrule production-strategy-init-product-preference-facts
  "Initialise the possible and preferred product facts to track products of each
  complexity for production flow control."
  (not (wm-fact (key strategy meta possible-products $?)))
  =>
  (assert
    (wm-fact (key strategy meta possible-products) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta active-products) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-products args? filter delivery-ahead) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-products args? filter delivery-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-products args? filter workload) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-products args? filter c0-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-products args? filter c1-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-products args? filter c2-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-products args? filter c3-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta filtered-products args? filter total-limit) (is-list TRUE) (type SYMBOL))
    (wm-fact (key strategy meta selected-product args? cond filter) (is-list FALSE) (type SYMBOL) (value nil))
    (wm-fact (key strategy meta selected-product args? cond possible) (is-list FALSE) (type SYMBOL) (value nil))
    (wm-fact (key strategy meta selected-product args? cond fallback) (is-list FALSE) (type SYMBOL) (value nil))
    (wm-fact (key strategy meta production-product-time-limit args? com C0) (value ?*C0-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta production-product-time-limit args? com C1) (value ?*C1-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta production-product-time-limit args? com C2) (value ?*C2-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta production-product-time-limit args? com C3) (value ?*C3-PRODUCTION-THRESHOLD*) (type INT))
    (wm-fact (key strategy meta production-product-limit args? com TOTAL) (value ?*TOTAL-PRODUCTION-THRESHOLD*) (type INT))
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

(defrule production-strategy-adapt-total-product-limit
  ?limit <- (wm-fact (key strategy meta production-product-limit args? com TOTAL) (value ?threshold))
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

(defrule production-strategy-reduce-product-time-limit
  ?limit <- (wm-fact (key strategy meta production-product-time-limit args? com ?comp) (value ~0))
  (wm-fact (key refbox game-time) (values ?gt&:(eq (* 60 (production-strategy-get-cutoff-complexity ?comp)) ?gt)))
  => 
  (modify ?limit (value 0))
)

(defrule production-strategy-append-active-products
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ?active <- (wm-fact (key strategy meta active-products) (values $?values))
  ;there is a root goal for an existing product
  (goal (id ?root) (mode ~RETRACTED))
  (goal-meta (goal-id ?root) (root-for-product ?product-id))
  (wm-fact (key domain fact product-complexity args? prod ?product-id $?))
  ;it is not in the active list yet
  (test (not (member$ ?product-id ?values)))
  =>
  (modify ?active (values $?values ?product-id))
)

(defrule production-strategy-remove-from-active-products
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ;there is a retracted root goal
  (goal (id ?root) (mode RETRACTED))
  (goal-meta (goal-id ?root) (root-for-product ?product-id))
  ;the product is in the active list
  ?active <- (wm-fact (key strategy meta active-products) (values $?values&:(member$ ?product-id ?values)))
  =>
  (modify ?active (values (delete-member$ ?values ?product-id)))
)

(defrule production-strategy-append-possible-products
  "An product is possible if it's not been fulfilled yet and if the machine occupancy
  allows it to be pursued."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ;facts to modify
  ?poss <- (wm-fact (key strategy meta possible-products) (values $?values))
  ;meta information
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ;neither delivered, nor started
  (wm-fact (key domain fact quantity-delivered args? prod ?product-id team ?team-color) (value ?quant-del))
  (wm-fact (key refbox product ?product-id quantity-requested) (value ?quant-req&:(> ?quant-req ?quant-del)))

  (not (goal-meta (root-for-product ?product-id)))
  ;it is not possible yet
  (test (not (member$ ?product-id ?values)))
  =>
  (modify ?poss (values $?values ?product-id))
)

(defrule production-strategy-remove-from-possible-products-active
  "An product that has been started, fulfilled is not possible anymore."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?poss <- (wm-fact (key strategy meta possible-products) (values $?values&:(member$ ?product-id ?values)))
  (or
    (and
      (wm-fact (key domain fact quantity-delivered args? prod ?product-id team ?team-color) (value ?quant-del))
      (wm-fact (key refbox product ?product-id quantity-requested) (value ?quant-req&:(= ?quant-req ?quant-del)))
    )
    (goal-meta (root-for-product ?product-id))
  )
  =>
  (modify ?poss (values (delete-member$ ?values ?product-id)))
)


;filter delivery-ahead
(defrule production-strategy-filter-products-delivery-ahead-add
  "Add an product to this filter if its production ahead window is open and isn't closed yet."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter delivery-ahead)
                        (values $?values&:(not (member$ ?product-id ?values))))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ;filter condition
  (wm-fact (key refbox product ?product-id delivery-begin) (value ?begin))
  (wm-fact (key refbox product ?product-id delivery-end) (value ?end))
  (wm-fact (key refbox game-time) (values ?gt $?))
  (test (production-strategy-produce-ahead-check ?gt ?begin ?end ?comp))
  =>
  (modify ?filtered (values $?values ?product-id))
)

(defrule production-strategy-filter-products-delivery-ahead-remove
  "Remove an product from this filter if its production ahead window has finally closed."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter delivery-ahead)
                        (values $?values&:(member$ ?product-id ?values)))
  (or
    (not (wm-fact (key strategy meta possible-products) (values $? ?product-id $?)))
    (and
      ;reverse filter condition
      (wm-fact (key refbox product ?product-id delivery-begin) (value ?begin))
      (wm-fact (key refbox product ?product-id delivery-end) (value ?end))
      (wm-fact (key refbox game-time) (values ?gt $?))
      (test (not (production-strategy-produce-ahead-check ?gt ?begin ?end ?comp)))
    )
  )
  =>
  (modify ?filtered (values (delete-member$ ?values ?product-id)))
)

;filter delivery-limit
(defrule production-strategy-filter-products-delivery-limit-add
  "Add an product to this filter its delivery window end is in the future."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter delivery-limit)
                        (values $?values&:(not (member$ ?product-id ?values))))
  ;filter condition
  (wm-fact (key refbox product ?product-id delivery-end) (value ?end))
  (wm-fact (key refbox game-time) (values ?gt $?))
  (test (< ?gt ?end))
  =>
  (modify ?filtered (values $?values ?product-id))
)

(defrule production-strategy-filter-products-delivery-limit-remove
  "Remove an product from this filter if its delivery window end has arrived."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter delivery-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (or
    (not (wm-fact (key strategy meta possible-products) (values $? ?product-id $?)))
    (and
      ;reverse filter condition
      (wm-fact (key refbox product ?product-id delivery-end) (value ?end))
      (wm-fact (key refbox game-time) (values ?gt $?))
      (test (> ?gt ?end))
    )
  )
  =>
  (modify ?filtered (values (delete-member$ ?values ?product-id)))
)

;filter machine workload
(defrule production-strategy-filter-products-workload-add
  "Add an product to this filter its workload doesn't push the summed workload over any machine's limit."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  (wm-fact (key strategy meta active-products))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter workload)
                        (values $?values&:(not (member$ ?product-id ?values))))
  ;filter condition
  (not
    (and
      (wm-fact (key mps workload overall args? m ?any-rs) (value ?workload))
      (wm-fact (key mps workload product args? m ?any-rs prod ?product-id) (value ?added-workload))
      (test (> (+ ?workload ?added-workload) ?*RS-WORKLOAD-THRESHOLD*))
    )
  )
  =>
  (modify ?filtered (values $?values ?product-id))
)

(defrule production-strategy-filter-products-workload-remove
  "Remove an product from this filter if its workload would push the summed workload over the limit."
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter workload)
                        (values $?values&:(member$ ?product-id ?values)))
  (or
    (not (wm-fact (key strategy meta possible-products) (values $? ?product-id $?)))
    (and
      (wm-fact (key mps workload overall args? m ?any-rs) (value ?workload))
      (wm-fact (key mps workload product args? m ?any-rs prod ?product-id) (value ?added-workload))
      (test (> (+ ?workload ?added-workload) ?*RS-WORKLOAD-THRESHOLD*))
    )
  )
  =>
  (modify ?filtered (values (delete-member$ ?values ?product-id)))
)

;filter c0 limit
(defrule production-strategy-filter-products-c0-limit-add
  "Add an product to this filter if there is less than the threshold of active C0 products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  (wm-fact (key strategy meta active-products))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c0-limit)
                        (values $?values&:(not (member$ ?product-id ?values))))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
 (wm-fact (key strategy meta production-product-time-limit args? com C0) (value ?limit))
 ;filter condition
  (or
    (and
      (test (eq ?comp C0))
      (test (> ?limit (production-strategy-count-active-products-of-complexity C0)))
    )
    (test (neq ?comp C0))
  )
  =>
  (modify ?filtered (values $?values ?product-id))
)

(defrule production-strategy-filter-products-c0-limit-remove
  "Remove an product from this filter if there is more than the threshold of active C0 products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c0-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (not (wm-fact (key strategy meta possible-products) (values $? ?product-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?product-id)))
)

(defrule production-strategy-filter-product-c0-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com C0))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c0-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (wm-fact (key strategy meta production-product-time-limit args? com C0) (value ?limit))
  (test (<= ?limit (production-strategy-count-active-products-of-complexity C0)))
  =>
  (modify ?filtered (values ))
)

;filter c1 limit
(defrule production-strategy-filter-products-c1-limit-add
  "Add an product to this filter if there is less than the threshold of active c1 products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  (wm-fact (key strategy meta active-products))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c1-limit)
                        (values $?values&:(not (member$ ?product-id ?values))))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  (wm-fact (key strategy meta production-product-time-limit args? com C1) (value ?limit))
  ;filter condition
  (or
    (and
      (test (eq ?comp C1))
      (test (> ?limit (production-strategy-count-active-products-of-complexity C1)))
    )
    (test (neq ?comp C1))
  )
  =>
  (modify ?filtered (values $?values ?product-id))
)

(defrule production-strategy-filter-products-c1-limit-remove
  "Remove an product from this filter if there is more than the threshold of active c1 products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c1-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (not (wm-fact (key strategy meta possible-products) (values $? ?product-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?product-id)))
)

(defrule production-strategy-filter-product-c1-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com C1))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c1-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (wm-fact (key strategy meta production-product-time-limit args? com C1) (value ?limit))
  (test (<= ?limit (production-strategy-count-active-products-of-complexity C1)))
  =>
  (modify ?filtered (values ))
)

;filter c2 limit
(defrule production-strategy-filter-products-c2-limit-add
  "Add an product to this filter if there is less than the threshold of active c2 products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  (wm-fact (key strategy meta active-products))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c2-limit)
                        (values $?values&:(not (member$ ?product-id ?values))))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  (wm-fact (key strategy meta production-product-time-limit args? com C2) (value ?limit))
  ;filter condition
  (or
    (and
      (test (eq ?comp C2))
      (test (> ?limit (production-strategy-count-active-products-of-complexity C2)))
    )
    (test (neq ?comp C2))
  )
  =>
  (modify ?filtered (values $?values ?product-id))
)

(defrule production-strategy-filter-products-c2-limit-remove
  "Remove an product from this filter if there is more than the threshold of active c2 products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c2-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (not (wm-fact (key strategy meta possible-products) (values $? ?product-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?product-id)))
)

(defrule production-strategy-filter-product-c2-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com C2))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c2-limit)
                        (values $?values&:(member$ ?product-id ?values)))
   (wm-fact (key strategy meta production-product-time-limit args? com C2) (value ?limit))
  (test (<= ?limit (production-strategy-count-active-products-of-complexity C2)))
  =>
  (modify ?filtered (values ))
)

;filter c3 limit
(defrule production-strategy-filter-products-c3-limit-add
  "Add an product to this filter if there is less than the threshold of active c3 products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  (wm-fact (key strategy meta active-products))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c3-limit)
                        (values $?values&:(not (member$ ?product-id ?values))))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  (wm-fact (key strategy meta production-product-time-limit args? com C3) (value ?limit))
  ;filter condition
  (or
    (and
      (test (eq ?comp C3))
      (test (> ?limit (production-strategy-count-active-products-of-complexity C3)))
    )
    (test (neq ?comp C3))
  )
  =>
  (modify ?filtered (values $?values ?product-id))
)

(defrule production-strategy-filter-products-c3-limit-remove
  "Remove an product from this filter if there is more than the threshold of active c3 products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com C3))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c3-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (not (wm-fact (key strategy meta possible-products) (values $? ?product-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?product-id)))
)

(defrule production-strategy-filter-product-c3-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com C3))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter c3-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (wm-fact (key strategy meta production-product-time-limit args? com C3) (value ?limit))
  (test (<= ?limit (production-strategy-count-active-products-of-complexity C3)))
  =>
  (modify ?filtered (values ))
)


;filter total limit
(defrule production-strategy-filter-products-total-limit-add
  "Add an product to this filter if there is less than the threshold of active total products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  (wm-fact (key strategy meta active-products))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter total-limit)
                        (values $?values&:(not (member$ ?product-id ?values))))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))

  (wm-fact (key strategy meta production-product-limit args? com TOTAL) (value ?threshold))
  ;filter condition
  (test (> ?threshold (production-strategy-count-active-products)))
  =>
  (modify ?filtered (values $?values ?product-id))
)

(defrule production-strategy-filter-products-total-limit-remove-impossible-products
  "Remove an product from this filter if there is more than the threshold of active total products"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter total-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (not (wm-fact (key strategy meta possible-products) (values $? ?product-id $?)))
  =>
  (modify ?filtered (values (delete-member$ ?values ?product-id)))
)

(defrule production-strategy-filter-product-total-limit-remove-limit-reached
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  ?filtered <- (wm-fact (key strategy meta filtered-products args? filter total-limit)
                        (values $?values&:(member$ ?product-id ?values)))
  (wm-fact (key strategy meta production-product-limit args? com TOTAL) (value ?threshold))
  (test (<= ?threshold (production-strategy-count-active-products)))
  =>
  (modify ?filtered (values ))
)

(defrule production-strategy-filter-set-selected-product-possible
  "- it is a possible product
   - there is no product of a higher complexity that is also possible"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  (not
    (and
      (wm-fact (key strategy meta possible-products) (values $? ?o-product-id&:(neq ?o-product-id ?product-id) $?))
      (wm-fact (key domain fact product-complexity args? prod ?o-product-id com ?o-comp))
      (test (< 0 (str-compare ?o-comp ?comp)))
    )
  )

  ?f <- (wm-fact (key strategy meta selected-product args? cond possible) (value ?ex-product-id))
  (or
    (and
      (wm-fact (key domain fact product-complexity args? prod ?ex-product-id com ?ex-comp))
      (test (> 0 (str-compare ?comp ?ex-comp)))
    )
    (test (eq ?ex-product-id nil))
  )
  =>
  (modify ?f (value ?product-id))
)

(defrule production-strategy-filter-unset-selected-product-possible
  " The selected product is not possible anymore (e.g., it was started).
    Clear the selection"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ?f <- (wm-fact (key strategy meta selected-product args? cond possible) (value ?ex-product-id&~nil))
  (wm-fact (key strategy meta possible-products) (values $?values&:(not (member$ ?ex-product-id ?values))))
  =>
  (modify ?f (value nil))
)

(defrule production-strategy-filter-set-selected-product-filter
  " - it is a possible product
    - it fulfills all the filters
    - there is no product of a higher complexity that fulfills all the filters"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))

  (wm-fact (key strategy meta possible-products) (values $? ?product-id $?))
  (wm-fact (key domain fact product-complexity args? prod ?product-id com ?comp))
  (not (wm-fact (key strategy meta filtered-products $?) (values $?values&:(not (member$ ?product-id ?values)))))
  (not
    (and
      (wm-fact (key strategy meta possible-products) (values $? ?o-product-id&:(neq ?product-id ?o-product-id) $?))
      (not (wm-fact (key strategy meta filtered-products $?) (values $?values&:(not (member$ ?o-product-id ?values)))))
      (wm-fact (key domain fact product-complexity args? prod ?o-product-id com ?o-comp))
      (test (< 0 (str-compare ?o-comp ?comp)))
    )
  )

  ?f <- (wm-fact (key strategy meta selected-product args? cond filter) (value ?ex-product-id))
  (or
    (and
      (wm-fact (key domain fact product-complexity args? prod ?ex-product-id com ?ex-comp))
      (test (> 0 (str-compare ?comp ?ex-comp)))
    )
    (test (eq ?ex-product-id nil))
  )
  =>
  (modify ?f (value ?product-id))
)

(defrule production-strategy-filter-set-selected-product-filter-empty
  "There is no product that meets all filters"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  ?f <- (wm-fact (key strategy meta selected-product args? cond filter) (value ?product-id&~nil))
  (wm-fact (key strategy meta filtered-products $?) (values $?values&:(not (member$ ?product-id ?values))))
  =>
  (modify ?f (value nil))
)

(defrule production-strategy-filter-selected-product-filter-empty-fallback
  "None of the products fulfill all filters"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta selected-product args? cond filter) (value nil))
  (wm-fact (key strategy meta filtered-products args? filter $?) (values ?any-product $?))
  ?fallback <- (wm-fact (key strategy meta selected-product args? cond fallback) (value ?fallback-product))
  =>
  (bind ?map (create$))
  (bind ?counts (create$))
  (do-for-all-facts ((?filter wm-fact)) (wm-key-prefix ?filter:key (create$ strategy meta filtered-products))
    (foreach ?product (fact-slot-value ?filter values)
      (if (member$ ?product ?map) then
          (bind ?index (member$ ?product ?map))
          (bind ?count (nth$ ?index ?counts))
          (bind ?counts (replace$ ?counts ?index ?index (+ ?count 1)))
        else
          (bind ?map (create$ ?map ?product))
          (bind ?counts (create$ ?counts 1))
      )
    )
  )
  (bind ?product (nth$ (member$ (nth$ 1 (sort < ?counts)) ?counts) ?map))
  (if (neq ?product ?fallback-product) then
    (modify ?fallback (value ?product))
  )
)

(defrule production-strategy-filter-selected-product-filter-empty-fallback-remove
  "There is at least product that meets all filters"
  (declare (salience ?*SALIENCE-ORDER-SELECTION*))
  (wm-fact (key strategy meta selected-product args? cond filter) (value ~nil))
  ?fallback <- (wm-fact (key strategy meta selected-product args? cond fallback) (value ~nil))
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
