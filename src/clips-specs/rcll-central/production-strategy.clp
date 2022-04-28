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
)


(defrule order-workload-addition
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  (domain-fact (name order-ring1-color|order-ring2-color|order-ring3-color )(param-values ?order-id ?ring-color&:(neq RING_NONE ?ring-color)))
  (domain-fact (name rs-ring-spec)(param-values ?mn ?ring-color ?rn))
  =>
  (bind ?payments 0)
  (if (eq ?rn ONE) then (bind ?payments 1))
  (if (eq ?rn TWO) then (bind ?payments 2))
  (if (eq ?rn THREE) then (bind ?payments 3))
  (if (not (any-factp ((?wm-fact wm-fact)) (and (wm-key-prefix ?wm-fact:key (create$ mps workload) )
                                                   (eq ?mn (wm-key-arg ?wm-fact:key m))
                                                   (eq ?order-id (wm-key-arg ?wm-fact:key ord))
                                            )))
    then
      (assert
        (wm-fact (key mps workload args? m ?mn ord ?order-id) (type INT)
          (is-list FALSE) (value (+ ?payments 1)))
      )
    else
      (bind ?fact (nth$ 1 (find-fact ((?wm-fact wm-fact)) (and (wm-key-prefix ?wm-fact:key (create$ mps workload) )
                                                            (eq ?mn (wm-key-arg ?wm-fact:key m))
                                                            (eq ?order-id (wm-key-arg ?wm-fact:key ord))))))
      (modify ?fact (value (+ (fact-slot-value ?fact value) 1)))
  )
)

(defrule order-workload-subtraction
  "TODO"
  (declare (salience ?*SALIENCE-PRODUCTION-STRATEGY*))
  ?g <- (goal (class MOUNT-RING)(mode RETRACTED)(outcome COMPLETED))
  (wm-fact (key mps workload args? m ?mn ord ?order-id))
  =>
  (bind ?params (fact-slot-value ?g params))
  (bind ?order-id (sym-cat (sub-string 4 6 (nth$ 2 ?params))))
  (bind ?color (nth$ (length$ ?params) (fact-slot-value ?g params)))
  (assert (oi ?order-id ?color))
  ;(bind ?fact (nth$ 1 (find-fact ((?wm-fact wm-fact)) (and (wm-key-prefix ?wm-fact:key (create$ mps workload) )
  ;                                                          (eq ?mn (wm-key-arg ?wm-fact:key m))
  ;                                                          (eq ?order-id (wm-key-arg ?wm-fact:key ord))))))
  ;(modify ?fact (value (- (fact-slot-value ?fact value) 1)))

)

(defrule production-strategy-init-order-meta-facts
" Calculates the points for each production step, max points and initializes
  more order meta facts.
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
                   (values (create$ 0 0 0 0 ?*TIME-DELIVER*))))
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
=>
  (bind ?curr-step RING1)
  (if (eq ?com C0) then (bind ?curr-step CAP))
  (assert (wm-fact (key wp meta points-current args? wp ?wp) (type INT)
                   (is-list FALSE) (value 0))
          (wm-fact (key wp meta next-step args? wp ?wp)
                   (type SYMBOL) (is-list FALSE) (value ?curr-step))
          (wm-fact (key wp meta estimated-points-total args? wp ?wp)
                   (type INT) (is-list FALSE) (value ?ep-total)))
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
  ; WP Meta CEs
  ?ns <- (wm-fact (key wp meta points-current args? wp ?wp) (value ?p-curr))
  ?wm <- (wm-fact (key wp meta next-step args? wp ?wp)
                   (value ?curr-step))
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
  (modify ?ss (value (+ ?scored 1)))
  (modify ?ns (value (+ ?p-curr
                        (nth$ (order-steps-index ?curr-step) $?p-list))))
  (modify ?wm (value ?new-step))
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
