;---------------------------------------------------------------------------
;  fixed-sequence.clp - Goal expander for RCLL goals
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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

(defglobal
  ?*SALIENCE-IDLE-CHECK* = -1
  ?*SALIENCE-EXPANDER-GENERIC* = 600
  ?*SALIENCE-GET-BASE-DIFF* = -10
  ?*SALIENCE-DELIVER-DIFF* = 100
)

(deftemplate wp-lock
  (slot wp(type SYMBOL))
  (slot order(type SYMBOL))
)

(defrule goal-expander-send-beacon-signal
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class SEND-BEACON) (mode SELECTED)
              (parent ?parent-id))
=>
  (assert
    (plan (id BEACONPLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id BEACONPLAN) (goal-id ?goal-id)
      (action-name send-beacon)))
  (modify ?g (mode EXPANDED))
)

; Set Robot to idle if there is no task left
; Will trigger the selection of new tasks
(defrule idle-check
(declare (salience ?*SALIENCE-IDLE-CHECK*))
(wm-fact (key domain fact entered-field args? r ?robot))
(not(goal (params robot ?robot $?rest-params)))
(not (idle-robot (robot ?robot)))
(not (wm-fact (key domain fact holding args? r ?robot wp ?any-wp)))
=>
(printout t "Robot " ?robot " set to idle" crlf)
(assert (idle-robot (robot ?robot)))
)


(defrule goal-expander-refill-shelf
  (declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
  ?p <- (goal (mode DISPATCHED) (id ?parent) (class PRODUCE-CPARENT))
  ?g <- (goal (id ?goal-id) (class REFILL-SHELF) (mode SELECTED)
              (params mps ?mps color ?col) (parent ?parent-id))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (assert
    (plan (id REFILL-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name lock) (param-values ?mps))
    (plan-action (id 2) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf) (skiller (remote-skiller ?robot))
                 (param-values ?mps LEFT (sym-cat CC- (random-id)) ?col))
    (plan-action (id 3) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf) (skiller (remote-skiller ?robot))
                 (param-values ?mps MIDDLE (sym-cat CC- (random-id)) ?col))
    (plan-action (id 4) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf) (skiller (remote-skiller ?robot))
                 (param-values ?mps RIGHT (sym-cat CC- (random-id)) ?col))
    (plan-action (id 5) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                  (action-name unlock) (param-values ?mps)))
  
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-get-base-to-fill-rs
(declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
 ?p <- (goal (mode DISPATCHED) (id ?parent) (class PRODUCE-CPARENT))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id)
             (class GET-BASE-TO-FILL-RS)
             (params robot ?robot
                      bs ?bs
                      bs-side ?bs-side
                      base-color ?base-color
                      wp ?spawned-wp
       ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (assert
  (plan (id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to)
        (param-values ?robot ?curr-location ?curr-side (wait-pos ?bs ?bs-side)))
  (plan-action (id 2) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?bs ?bs-side))
  (plan-action (id 3) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name move)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to to-side )
        (param-values ?robot (wait-pos ?bs ?bs-side) WAIT ?bs ?bs-side))
  (plan-action (id 4) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name lock)
        (param-values ?bs))
  (plan-action (id 5) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name prepare-bs)
        (skiller (remote-skiller ?robot))
        (param-values ?bs ?bs-side ?base-color))
  (plan-action (id 6) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name bs-dispense)
        (skiller (remote-skiller ?robot))
        (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
  (plan-action (id 7) (plan-id  GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
        (skiller (remote-skiller ?robot))
        (param-names r wp m side)
        (param-values ?robot ?spawned-wp ?bs ?bs-side))
  (plan-action (id 8) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name unlock)
        (param-values ?bs))
  (plan-action (id 9) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?bs ?bs-side))
  (plan-action (id 10) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to)
        (param-values ?robot ?bs ?bs-side (wait-pos ?bs ?bs-side)))
 )
 (modify ?g (mode EXPANDED))
)


(defrule goal-expander-discard-wp
(declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
 ?p <- (goal (mode DISPATCHED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class DISCARD-WP) (mode SELECTED)
             (parent ?parent)
             (params robot ?robot
                    wp ?wp
             ))
  =>
  (assert
    (plan (id DISCARD-WP-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id DISCARD-WP-PLAN) (goal-id ?goal-id)
          (action-name wp-discard)
          (skiller (remote-skiller ?robot))
          (param-names r cc )
          (param-values ?robot ?wp))
  )
  (modify ?g (mode EXPANDED))
)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule goal-expander-enter-field
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
              (params r ?robot team-color ?team-color))
 (wm-fact (key domain fact mps-type args? m ?mps t ?t))
 (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
=>
(bind ?planid (sym-cat ENTER-FIELD-PLAN- (gensym*)))
  (assert
    (plan (id ?planid) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
                                 (action-name enter-field)
                                 (skiller (remote-skiller ?robot))
                                 (param-names r team-color)
                                 (param-values ?robot ?team-color)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-deliver
(declare (salience (+ ?*SALIENCE-EXPANDER-GENERIC* ?*SALIENCE-DELIVER-DIFF*)))
 ?g <- (goal (id ?goal-id) (parent ?parent) (class DELIVER) (mode SELECTED) (params order ?order))

 (wm-fact (key refbox team-color) (value ?team-color))

 (wm-fact (key domain fact mps-type args? m ?ds t DS))
 (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
 (not (wm-fact (key domain fact mps-state args? m ?ds s BROKEN)))

 (wm-fact (key domain fact holding args? r ?robot wp ?wp))

 (wm-fact (key domain fact order-complexity args? ord ?order com ?comp))

 (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
 (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))

 (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
 (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))

 (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
 (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))

 (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
 (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))

 (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
 (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
 
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key domain fact entered-field args? r ?robot))

 (wm-fact (key refbox game-time) (values $?game-time))
 (wm-fact (key refbox order ?order delivery-begin) (value ?delivery-begin&:(< ?delivery-begin (nth$ 1 ?game-time))))

(not(goal (params robot ?robot $?rest-params)))
 =>
      (bind ?planid (sym-cat DELIVER-PLAN- (gensym*)))
      (if (eq ?comp C0)
            then
      (assert
      (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name fulfill-order-c0)
              (skiller (remote-skiller ?robot))
              (param-values ?order ?wp ?ds INPUT ?base-color ?cap-color))))
      (if (eq ?comp C1)
            then
      (assert
      (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name fulfill-order-c1)
              (skiller (remote-skiller ?robot))
              (param-values ?order ?wp ?ds INPUT ?base-color ?cap-color ?ring1-color))))
      (if (eq ?comp C2)
            then
      (assert
      (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name fulfill-order-c2)
              (skiller (remote-skiller ?robot))
              (param-values ?order ?wp ?ds INPUT ?base-color ?cap-color ?ring1-color ?ring2-color))))
      (if (eq ?comp C3)
            then
      (assert
      (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name fulfill-order-c3)
              (skiller (remote-skiller ?robot))
              (param-values ?order ?wp ?ds INPUT ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color))))
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?ds INPUT)))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?ds INPUT))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?ds INPUT) WAIT ?ds INPUT))
        (plan-action (id 4) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?wp ?ds))
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
              (action-name prepare-ds)
              (skiller (remote-skiller ?robot))
              (param-values ?ds ?order))
        ;empty slot 6 for the deliver action
        (plan-action (id 7) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock) (param-values ?ds INPUT))
        (plan-action (id 8) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?ds INPUT (wait-pos ?ds INPUT)))
	  )
        (modify ?g (mode EXPANDED)(params robot ?robot order ?order))
)

(defrule goal-expander-mount-cap
(declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
 ?g <- (goal (id ?goal-id) (parent ?parent) (class MOUNT-CAP) (mode SELECTED) (params order ?order cs ?any-cs))

 (wm-fact (key refbox team-color) (value ?team-color))

 (wm-fact (key domain fact mps-type args? m ?cs t CS))
 (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
 (not (wm-fact (key domain fact mps-state args? m ?cs s BROKEN)))

 (wm-fact (key domain fact cs-buffered args? m ?cs col ?cap-color))
 (wm-fact (key domain fact mps-side-free args? m ?cs side OUTPUT))

 (wm-fact (key domain fact holding args? r ?robot wp ?wp))
 (wp-lock (wp ?wp) (order ?order))

 (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
 (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))

 (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
 (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))

 (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
 (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))

 (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
 (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))

 (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))

 
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key domain fact entered-field args? r ?robot))

 (not (goal (class BUFFER-CS)(mode EXPANDED|DISPATCHED)(params robot ?some-robot1 cs-color ?some-cap-color cs ?cs)))
 (not (goal (class MOUNT-CAP)(mode EXPANDED|DISPATCHED)(params robot ?some-robot2 order ?some-order cs ?cs)))
 (not (goal (class MOUNT-CAP)(mode EXPANDED|DISPATCHED)(params robot ?some-robot3 order ?order cs ?cs)))
 (not(goal (params robot ?robot $?rest-params)))
 =>
      (bind ?planid (sym-cat MOUNT-CAP-PLAN- (gensym*)))
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?cs INPUT)))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?cs INPUT))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?cs INPUT) WAIT ?cs INPUT))
        (plan-action (id 4) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?wp ?cs))
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
              (action-name prepare-cs)
              (skiller (remote-skiller ?robot))
              (param-values ?cs MOUNT_CAP))
        (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name cs-mount-cap)
              (skiller (remote-skiller ?robot))
              (param-values ?cs ?wp ?cap-color))
        (plan-action (id 7) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?cs INPUT (wait-pos ?cs INPUT)))
        (plan-action (id 8) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock) (param-values ?cs INPUT))
        (plan-action (id 9) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot (wait-pos ?cs INPUT) WAIT (wait-pos ?cs OUTPUT)))
        (plan-action (id 10) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?cs OUTPUT))
        (plan-action (id 11) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?cs OUTPUT) WAIT ?cs OUTPUT))
        (plan-action (id 12) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?cs OUTPUT))
        (plan-action (id 13) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?cs OUTPUT))
        (plan-action (id 14) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?cs OUTPUT (wait-pos ?cs OUTPUT)))
	  )
        (modify ?g (mode EXPANDED)(params robot ?robot order ?order cs ?cs))
)

(defrule goal-expander-get-base-complete
  (declare (salience (+ ?*SALIENCE-EXPANDER-GENERIC* ?*SALIENCE-GET-BASE-DIFF*)))
 ?g <- (goal (id ?goal-id) (parent ?parent) (class GET-BASE) (mode SELECTED) (params order ?order bs ?any-bs))
 (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
 (wm-fact (key domain fact entered-field args? r ?robot))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key domain fact can-hold args? r ?robot))

 (wm-fact (key refbox team-color) (value ?team-color))
 (wm-fact (key domain fact mps-type args? m ?bs t BS))
 (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
 (not (wm-fact (key domain fact mps-state args? m ?bs s BROKEN)))

 (wm-fact (key domain fact mps-side-free args? m ?bs side INPUT))

 ;(not (goal (class GET-BASE)(mode EXPANDED|DISPATCHED)(params robot ?some-robot order ?some-order bs ?bs)))
 (not(goal (params robot ?robot $?rest-params)))
 =>
      (bind ?wp (sym-cat WP- (random-id)))
      (bind ?planid (sym-cat GET-BASE-PLAN- (gensym*)))
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name spawn-wp)
              (param-values ?wp ?robot))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?bs INPUT)))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?bs INPUT))
        (plan-action (id 4) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side )
              (param-values ?robot (wait-pos ?bs INPUT) WAIT ?bs INPUT))
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
              (action-name prepare-bs)
              (skiller (remote-skiller ?robot))
              (param-names m side bc)
              (param-values ?bs INPUT ?base-color))
        (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name bs-dispense)
              (skiller (remote-skiller ?robot))
              (param-names r m side wp basecol)
              (param-values ?robot ?bs INPUT ?wp ?base-color))
        (plan-action (id 7) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?bs INPUT))
        (plan-action (id 8) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?bs INPUT (wait-pos ?bs INPUT)))
        (plan-action (id 9) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?bs INPUT))
	  )
        (modify ?g (mode EXPANDED)(params robot ?robot order ?order bs ?bs))
      (assert (wp-lock (wp ?wp) (order ?order)))
      (printout t "lock wp " ?wp " for order " ?order crlf)
)

(defrule goal-expander-get-base-recover-wp
  (declare (salience (+ ?*SALIENCE-EXPANDER-GENERIC* ?*SALIENCE-GET-BASE-DIFF*)))
 ?g <- (goal (id ?goal-id) (parent ?parent) (class GET-BASE) (mode SELECTED) (params order ?order bs ?any-bs))
 (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
 (wm-fact (key domain fact entered-field args? r ?robot))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key domain fact can-hold args? r ?robot))

 (wm-fact (key refbox team-color) (value ?team-color))
 (wm-fact (key domain fact mps-type args? m ?bs t BS))
 (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))

 (wm-fact (key domain fact wp-at args? wp ?wp m ?bs side INPUT))
 (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))

 (not (goal (class GET-BASE)(mode EXPANDED|DISPATCHED)(params robot ?some-robot order ?some-order bs ?bs)))
 (not(goal (params robot ?robot $?rest-params)))
 =>
      (bind ?wp (sym-cat WP- (random-id)))
      (bind ?planid (sym-cat GET-BASE-PLAN- (gensym*)))
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?bs INPUT)))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?bs INPUT))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side )
              (param-values ?robot (wait-pos ?bs INPUT) WAIT ?bs INPUT))
        (plan-action (id 4) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?bs INPUT))
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?bs INPUT (wait-pos ?bs INPUT)))
        (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?bs INPUT))
	  )
        (modify ?g (mode EXPANDED)(params robot ?robot order ?order bs ?bs))
      (assert (wp-lock (wp ?wp) (order ?order)))
      (printout t "lock wp " ?wp " for order " ?order crlf)
)

(defrule goal-expander-get-cap-carrier
  (declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
 ?g <- (goal (id ?goal-id) (parent ?parent) (class GET-CC) (mode SELECTED) (params cs-color ?cap-color cs ?any-cs))

 (wm-fact (key refbox team-color) (value ?team-color))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key domain fact entered-field args? r ?robot))
 (wm-fact (key domain fact can-hold args? r ?robot))
 (not(goal (params robot ?robot $?rest-params)))

 (wm-fact (key domain fact mps-type args? m ?cs t CS))
 (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))

 (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
 (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))

 (wm-fact (key domain fact mps-type args? m ?some-cs t CS))
 (wm-fact (key domain fact mps-team args? m ?some-cs col ?team-color))

 (or (wm-fact (key domain fact cs-can-perform args? m ?cs op RETRIEVE_CAP))
     (wm-fact (key domain fact cs-can-perform args? m ?some-cs op RETRIEVE_CAP)))

 (not (goal (class GET-CC)(mode EXPANDED|DISPATCHED)(params robot ?some-robot cs-color ?some-cap-color cs ?cs)))

 =>
      (bind ?planid (sym-cat GET-CC-PLAN- (gensym*)))
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?cs INPUT)))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?cs INPUT))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?cs INPUT) WAIT ?cs INPUT))
        (plan-action (id 4) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get-shelf)
              (skiller (remote-skiller ?robot))
              (param-names r cc m spot)
              (param-values ?robot ?cc ?cs ?shelf-spot))
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?cs INPUT))
        (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?cs INPUT (wait-pos ?cs INPUT)))
	  )
        (modify ?g (mode EXPANDED)(params robot ?robot cs-color ?cap-color cs ?cs))
)



(defrule goal-expander-buffer-cs
  (declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
 ?g <- (goal (id ?goal-id) (parent ?parent) (class BUFFER-CS) (mode SELECTED) (params cs-color ?cap-color cs ?any-cs))

 (wm-fact (key refbox team-color) (value ?team-color))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key domain fact entered-field args? r ?robot))

 ; match cc, avoid proper wp
 (wm-fact (key domain fact holding args? r ?robot wp ?cc))
 (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
 (not (wp-lock (wp ?cc) (order ?some-order)))

 (not(goal (params robot ?robot $?rest-params)))

 (wm-fact (key domain fact mps-type args? m ?cs t CS))
 (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
 (not (wm-fact (key domain fact mps-state args? m ?cs s BROKEN)))

 (wm-fact (key domain fact cs-can-perform args? m ?cs op RETRIEVE_CAP))

 (not (goal (class BUFFER-CS)(mode EXPANDED|DISPATCHED)(params robot ?some-robot cs-color ?some-cap-color cs ?cs)))
 =>
      (bind ?planid (sym-cat BUFFER-CS-PLAN- (gensym*)))
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?cs INPUT)))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?cs INPUT))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?cs INPUT) WAIT ?cs INPUT))
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?cc ?cs))
        (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name prepare-cs)
              (skiller (remote-skiller ?robot))
              (param-values ?cs RETRIEVE_CAP))
        (plan-action (id 7) (plan-id ?planid) (goal-id ?goal-id)
              (action-name cs-retrieve-cap)
              (skiller (remote-skiller ?robot))
              (param-values ?cs ?cc ?cap-color))
        (plan-action (id 8) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?cs INPUT (wait-pos ?cs INPUT)))
        (plan-action (id 9) (plan-id ?planid) (goal-id ?goal-id)
             (action-name location-unlock) (param-values ?cs INPUT))
        (plan-action (id 10) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot (wait-pos ?cs INPUT) WAIT (wait-pos ?cs OUTPUT)))
        (plan-action (id 11) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?cs OUTPUT))
        (plan-action (id 12) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?cs OUTPUT) WAIT ?cs OUTPUT))
        (plan-action (id 13) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?cc ?cs OUTPUT))
        (plan-action (id 14) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-discard)
              (skiller (remote-skiller ?robot))
              (param-names r cc )
              (param-values ?robot ?cc))
        (plan-action (id 15) (plan-id ?planid)(goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?cs OUTPUT (wait-pos ?cs OUTPUT)))
        (plan-action (id 16) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock) (param-values ?cs OUTPUT))
	  )
        (modify ?g (mode EXPANDED)(params robot ?robot cs-color ?cap-color cs ?cs))
)



(defrule goal-expander-buffer-rs
  (declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
 ?g <- (goal (id ?goal-id) (parent ?parent) (class BUFFER-RS) (mode SELECTED) (params order ?order rs ?rs))
 (wm-fact (key domain fact entered-field args? r ?robot))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))

 ;; check wp conditions
 (wm-fact (key domain fact holding args? r ?robot wp ?wp))
 (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
 (wp-lock (wp ?wp) (order ?order))

 (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-before&~THREE))
 (wm-fact (key domain fact rs-inc args? summand ?bases-before sum ?bases-after))
 (not (wm-fact (key domain fact mps-state args? m ?rs s BROKEN)))

 (not(goal (class MOUNT-RING) (mode EXPANDED|DISPATCHED) (params robot ?some-robot1 order ?some-order1 rs ?rs wp ?some-wp ring-num ?some-ring-num)))
 (not(goal (class BUFFER-RS) (mode EXPANDED|DISPATCHED) (params robot ?some-robot2 order ?some-order2 rs ?rs)))

 (not(goal (params robot ?robot $?rest-params)))
 =>
      (printout t "buffer rs "  ?rs " bases before: " ?bases-before " bases after: " ?bases-after crlf)
      (bind ?planid (sym-cat BUFFER-RS-PLAN- (gensym*)))
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?rs INPUT)))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?rs INPUT))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?rs INPUT) WAIT ?rs INPUT))
        (plan-action (id 4) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-put-slide-cc)
              (skiller (remote-skiller ?robot))
              (param-values ?robot ?wp ?rs ?bases-before ?bases-after))
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?rs INPUT))
        (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?rs INPUT (wait-pos ?rs INPUT)))
	  )
        (modify ?g (mode EXPANDED)(params robot ?robot order ?order rs ?rs))
)



(defrule goal-expander-mount-ring
  (declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
 ?g <- (goal (id ?goal-id) (parent ?parent) (class MOUNT-RING) (mode SELECTED) (params order ?order rs ?rs ring-num ?ring-num))

 (wm-fact (key refbox team-color) (value ?team-color))

;; get information from order
 (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
 (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
 (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
 (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))

;; check wp conditions
 (wm-fact (key domain fact holding args? r ?robot wp ?wp))
 (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
 (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
 (wp-lock (wp ?wp) (order ?order))

 (or (and (test (eq ?ring-num ONE))
         (wm-fact (key domain fact wp-ring1-color args? wp ?wp col RING_NONE))
         (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE))
         (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE))
         (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring1-color rn ?bases-needed)))
    (and (test (eq ?ring-num TWO))
         (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
         (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE))
         (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE))
         (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring2-color rn ?bases-needed)))
    (and (test (eq ?ring-num THREE)) 
         (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
         (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
         (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE))
         (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring3-color rn ?bases-needed)))
         )

;ring management taken from rcll code
 (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
 (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))

 (not(goal (class MOUNT-RING) (mode EXPANDED|DISPATCHED) (params robot ?some-robot1 order ?some-order rs ?rs ring-num ?some-ring-num)))
 (not(goal (class BUFFER-RS) (mode EXPANDED|DISPATCHED) (params robot ?some-robot2 rs ?rs)))

 (wm-fact (key domain fact mps-side-free args? m ?rs side INPUT))
 (wm-fact (key domain fact mps-side-free args? m ?rs side OUTPUT))
 (not (wm-fact (key domain fact mps-state args? m ?rs s BROKEN)))
 
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))

 (not(goal (params robot ?robot $?rest-params)))
 =>
      (printout t "mount ring "  ?rs " bases needed: " ?bases-needed " bases before: " ?bases-filled " bases after: " ?bases-remain crlf)
      (bind ?planid (sym-cat MOUNT-RING-PLAN- (gensym*)))
      (if (eq ?ring-num ONE)
            then
      (assert
      (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
            (action-name prepare-rs)
            (skiller (remote-skiller ?robot))
            (param-values ?rs ?ring1-color ?bases-filled ?bases-remain ?bases-needed))
      (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
            (action-name rs-mount-ring1)
            (skiller (remote-skiller ?robot))
            (param-values ?rs ?wp ?ring1-color ?bases-filled ?bases-remain ?bases-needed))))
      (if (eq ?ring-num TWO)
            then
      (assert
      (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
            (action-name prepare-rs)
            (skiller (remote-skiller ?robot))
            (param-values ?rs ?ring2-color ?bases-filled ?bases-remain ?bases-needed))
      (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
            (action-name rs-mount-ring2)
            (skiller (remote-skiller ?robot))
            (param-values ?rs ?wp ?ring2-color ?ring1-color ?bases-filled ?bases-remain ?bases-needed))))
      (if (eq ?ring-num THREE)
            then
      (assert
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
            (action-name prepare-rs)
            (skiller (remote-skiller ?robot))
            (param-values ?rs ?ring3-color ?bases-filled ?bases-remain ?bases-needed))
        (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
            (action-name rs-mount-ring3)
            (skiller (remote-skiller ?robot))
            (param-values ?rs ?wp ?ring3-color ?ring1-color ?ring2-color ?bases-filled ?bases-remain ?bases-needed))))
      
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?rs INPUT)))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
            (action-name location-lock)
            (param-values ?rs INPUT))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
            (action-name move)
            (skiller (remote-skiller ?robot))
            (param-names r from from-side to to-side )
            (param-values ?robot (wait-pos ?rs INPUT) WAIT ?rs INPUT))
        (plan-action (id 4) (plan-id ?planid) (goal-id ?goal-id)
            (action-name wp-put)
            (skiller (remote-skiller ?robot))
            (param-names r wp m)
            (param-values ?robot ?wp ?rs))
        ;; Mount Ring Actions are performed here
        (plan-action (id 7) (plan-id ?planid) (goal-id ?goal-id)
            (action-name go-wait)
            (skiller (remote-skiller ?robot))
            (param-names r from from-side to)
            (param-values ?robot ?rs INPUT (wait-pos ?rs INPUT)))
        (plan-action (id 8) (plan-id ?planid) (goal-id ?goal-id)
            (action-name location-unlock)
            (param-values ?rs INPUT))
        (plan-action (id 9) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot (wait-pos ?rs INPUT) WAIT (wait-pos ?rs OUTPUT)))
        (plan-action (id 10) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?rs OUTPUT))
        (plan-action (id 11) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?rs OUTPUT) WAIT ?rs OUTPUT))
        (plan-action (id 12) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?rs OUTPUT))
        (plan-action (id 13) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?rs OUTPUT (wait-pos ?rs OUTPUT)))
        (plan-action (id 14) (plan-id ?planid) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?rs OUTPUT))
	  )
        (modify ?g (mode EXPANDED)(params robot ?robot order ?order rs ?rs wp ?wp ring-num ?ring-num))
)



(defrule goal-expander-go-wait
  "Move to a waiting position."
  (declare (salience ?*SALIENCE-EXPANDER-GENERIC*))
   ?p <- (goal (mode DISPATCHED) (id ?parent))
   ?g <- (goal (id ?goal-id) (class GO-WAIT) (mode SELECTED) (parent ?parent))
   (wm-fact (key domain fact self args? r ?robot))
   (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
   =>
   (assert
        (plan (id GO-WAIT-PLAN) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id GO-WAIT-PLAN) (goal-id ?goal-id)
                     (action-name go-wait)
                     (skiller(remote-skiller ?robot))
                     (param-names r from from-side to)
                     (param-values ?robot ?curr-location ?curr-side START INPUT))
   )
   (modify ?g (mode EXPANDED))
)


