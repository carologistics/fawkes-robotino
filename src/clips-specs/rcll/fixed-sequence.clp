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

(defrule goal-expander-send-beacon-signal
  ?p <- (goal (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class SEND-BEACON) (mode SELECTED)
              (parent ?parent-id))
=>
  (assert
    (plan (id BEACONPLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id BEACONPLAN) (goal-id ?goal-id)
      (action-name send-beacon)))
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-spawn-wp
  ?p <- (goal (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class SPAWN-WP) (mode SELECTED)
              (parent ?parent-id) (params used-wp ?used-wp count ?count))
=>
  (assert
    (plan (id SPAWNPLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id SPAWNPLAN) (goal-id ?goal-id)
                 (action-name spawn-wp)
                 (param-values (sym-cat WP- (random-id)) ?used-wp ?count))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-spawn-ss-c0
  ?p <- (goal (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class SPAWN-SS-C0) (mode SELECTED)
              (parent ?parent-id) (params robot ?robot ss ?ss base ?base cap ?cap))
=>
  (bind ?wp (sym-cat WP- (random-id)))
  (assert
    (plan (id SPAWNPLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id SPAWNPLAN) (goal-id ?goal-id)
                 (action-name spawn-wp)
                 (param-values ?wp ?robot))
    (plan-action (id 2) (plan-id SPAWNPLAN) (goal-id ?goal-id)
                 (action-name ss-store-wp)
                 (param-values ?robot ?ss ?wp ?base ?cap))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-refill-shelf
  ?p <- (goal (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class REFILL-SHELF) (mode SELECTED)
              (params mps ?mps) (parent ?parent-id))
  (wm-fact (key domain fact cs-color args? m ?mps col ?col))
  =>
  (assert
    (plan (id REFILL-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name lock) (param-values ?mps))
    (plan-action (id 2) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf)
                 (param-values ?mps LEFT (sym-cat CC- (random-id)) ?col))
    (plan-action (id 3) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf)
                 (param-values ?mps MIDDLE (sym-cat CC- (random-id)) ?col))
    (plan-action (id 4) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf)
                 (param-values ?mps RIGHT (sym-cat CC- (random-id)) ?col))
    (plan-action (id 5) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name unlock) (param-values ?mps))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-exploration
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class EXPLORATION))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact self args? r ?r))
  (wm-fact (id ?id&: (eq ?id (str-cat "/config/rcll/route/" ?team-color "/" ?r))) (values $?route))
  =>
  (assert (plan (goal-id ?goal-id) (id EXPLORATION-PLAN)))
  (bind ?action-id 1)
  (foreach ?node ?route
	(assert (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id EXPLORATION-PLAN) (action-name move-node) (param-values ?r ?node)))
	(bind ?action-id (+ ?action-id 1))
  )
  (modify ?g (mode EXPANDED))
)



(defrule goal-expander-enter-field
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
              (params r ?robot team-color ?team-color))
=>
  (assert
    (plan (id ENTER-FIELD-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ENTER-FIELD-PLAN) (goal-id ?goal-id)
                                 (action-name enter-field)
                                 (param-names r team-color)
                                 (param-values ?robot ?team-color))
    )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-wait
  ?p <- (goal (id ?parent))
  ?g <- (goal (id ?goal-id) (class WAIT) (parent ?parent) (mode SELECTED)
              (params r ?robot
                      point ?waitpoint))
  =>
  (assert
    (plan (id WAIT-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id WAIT-PLAN) (goal-id ?goal-id)
                 (action-name wait)
                 (param-names r point)
                 (param-values ?robot ?waitpoint)
    )
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-go-wait
  "Move to a waiting position."
   ?p <- (goal (id ?parent))
   ?g <- (goal (id ?goal-id) (class GO-WAIT) (mode SELECTED) (parent ?parent)
               (params r ?robot
                 point ?waitpoint
         ))
   (wm-fact (key domain fact self args? r ?robot))
   (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
   =>
   (assert
        (plan (id GO-WAIT-PLAN) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id GO-WAIT-PLAN) (goal-id ?goal-id)
                     (action-name go-wait)
                     (param-names r from from-side to)
                     (param-values ?robot ?curr-location ?curr-side ?waitpoint))
   )
   (modify ?g (mode EXPANDED))
)


(defrule goal-expander-prefill-cap-station
   "Feed a CS with a cap from its shelf so that afterwards
   it can directly put the cap on a product."
    ?p <- (goal (id ?parent))
    ?g <- (goal (id ?goal-id) (class FILL-CAP) (mode SELECTED) (parent ?parent)
                (params robot ?robot
                        mps ?mps
                        cc ?cc
                ))
    (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
    (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
    (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?shelf-spot))
    =>
   (assert
        (plan (id FILL-CAP-PLAN) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (param-names r from from-side to)
                                    (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps INPUT)))
        (plan-action (id 2) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?mps INPUT))
        (plan-action (id 3) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?mps INPUT) WAIT ?mps INPUT))
        (plan-action (id 4) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name wp-get-shelf)
                                    (param-names r cc m spot)
                                    (param-values ?robot ?cc ?mps ?shelf-spot))
        (plan-action (id 5) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name wp-put)
                                    (param-names r wp m)
                                    (param-values ?robot ?cc ?mps))
        (plan-action (id 6) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name request-cs-retrieve-cap)
                                    (param-values ?robot ?mps ?cc ?cap-color))
        (plan-action (id 7) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?mps INPUT))
        (plan-action (id 8) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (param-names r from from-side to)
                                    (param-values ?robot ?mps INPUT (wait-pos ?mps INPUT)))
    )
    (modify ?g (mode EXPANDED))
)

(defrule goal-remove-workpiece-from-mps
 ?p <- (goal (id ?parent))
 ?g <- (goal (id ?goal-id) (class CLEAR-MPS) (mode SELECTED) (parent ?parent)
             (params robot ?robot
                      mps ?mps
                      wp ?wp
                      side ?side
                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (assert
  (plan (id CLEAR-MPS-PLAN) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps ?side)))
  (plan-action (id 2) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps ?side))
  (plan-action (id 3) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot (wait-pos ?mps ?side) WAIT ?mps ?side))
  (plan-action (id 4) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?wp ?mps ?side))
  (plan-action (id 5) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
                              (action-name location-unlock)
                              (param-values ?mps ?side))
  (plan-action (id 6) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?mps ?side (wait-pos ?mps ?side)))
 )
 (modify ?g (mode EXPANDED))
)

;Alternative pesudo implemantation for the RESET-MPS but as a plan
; The goal commitment should decide which one to pick
; (defrule goal-reset-mps
;  ?p <- (goal (mode EXPANDED) (id ?parent))
;  ?g <- (goal (mode SELECTED) (parent ?parent) (id CLEAR-CS)
;                                              (params robot ?robot
;                                                       mps ?mps
;                                                       wp ?wp
;                                                       ))
;   =>
;   (assert
;     (plan (id RESET-MPS-PLAN) (goal-id CLEAR-CS))
;     (plan-action (id 1) (plan-id RESET-MPS-PLAN) (goal-id CLEAR-CS)
;           (action-name reset-mps)
;           (param-names r m)
;           (param-values ?robot ?mps))
;   )
;   (modify ?g (mode EXPANDED))
; )

(defrule goal-expander-discard-unneeded-base
 ?p <- (goal (id ?parent))
 ?g <- (goal (id ?goal-id) (class DISCARD-UNKNOWN) (mode SELECTED)
             (parent ?parent)
             (params robot ?robot
                    wp ?wp
             ))
  =>
  (assert
    (plan (id DISCARD-UNKNOWN-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id DISCARD-UNKNOWN-PLAN) (goal-id ?goal-id)
          (action-name wp-discard)
          (param-names r cc )
          (param-values ?robot ?wp))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-fill-rs
 ?p <- (goal (id ?parent))
 ?g <- (goal (id ?goal-id) (class FILL-RS) (mode SELECTED) (parent ?parent)
             (params robot ?robot
                      mps ?mps
                      wp ?wp
                      rs-before ?rs-before
                      rs-after ?rs-after
             ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (assert
    (plan (id FILL-RS-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps INPUT)))
    (plan-action (id 2) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?mps INPUT))
    (plan-action (id 3) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?mps INPUT) WAIT ?mps INPUT))
    (plan-action (id 4) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name wp-put-slide-cc)
          (param-names r wp m rs-before rs-after)
          (param-values ?robot ?wp ?mps ?rs-before ?rs-after))
    (plan-action (id 5) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?mps INPUT))
    (plan-action (id 6) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?mps INPUT (wait-pos ?mps INPUT)))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-get-base-to-fill-rs
 ?p <- (goal (id ?parent))
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
        (param-names r from from-side to)
        (param-values ?robot ?curr-location ?curr-side (wait-pos ?bs ?bs-side)))
  (plan-action (id 2) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?bs ?bs-side))
  (plan-action (id 3) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot (wait-pos ?bs ?bs-side) WAIT ?bs ?bs-side))
  (plan-action (id 4) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name lock)
        (param-values ?bs))
  (plan-action (id 5) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name prepare-bs)
        (param-values ?bs ?bs-side ?base-color))
  (plan-action (id 6) (plan-id GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name bs-dispense)
        (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
  (plan-action (id 7) (plan-id  GET-BASE-TO-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
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
        (param-names r from from-side to)
        (param-values ?robot ?bs ?bs-side (wait-pos ?bs ?bs-side)))
 )
 (modify ?g (mode EXPANDED))
)


(defrule goal-expander-get-shelf-to-fill-rs
 ?p <- (goal (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id)
             (class GET-SHELF-TO-FILL-RS)
             (params robot ?robot
                      cs ?cs
                      wp ?wp
                      spot ?spot
       ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
 (assert
    (plan (id GET-SHELF-TO-FILL-RS-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id GET-SHELF-TO-FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name go-wait)
          (param-names r from from-side to)
          (param-values ?robot ?curr-location ?curr-side (wait-pos ?cs INPUT)))
    (plan-action (id 2) (plan-id GET-SHELF-TO-FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name location-lock)
          (param-values ?cs INPUT))
    (plan-action (id 3) (plan-id GET-SHELF-TO-FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name move)
          (param-names r from from-side to to-side )
          (param-values ?robot (wait-pos ?cs INPUT) WAIT ?cs INPUT))
    (plan-action (id 4) (plan-id  GET-SHELF-TO-FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name wp-get-shelf)
          (param-names r cc m spot)
          (param-values ?robot ?wp ?cs ?spot))
    (plan-action (id 5) (plan-id GET-SHELF-TO-FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name location-unlock)
          (param-values ?cs INPUT))
    (plan-action (id 6) (plan-id GET-SHELF-TO-FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name go-wait)
          (param-names r from from-side to)
          (param-values ?robot ?cs INPUT (wait-pos ?cs INPUT)))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-produce-c0
 ?p <- (goal (id ?parent))
 ?g <- (goal (id ?goal-id) (class PRODUCE-C0) (mode SELECTED) (parent ?parent)
                                             (params robot ?robot
                                                      bs ?bs
                                                      bs-side ?bs-side
                                                      bs-color ?base-color
                                                      mps ?mps
                                                      cs-color ?cap-color
                                                      order ?order
                                                      wp ?spawned-wp
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?spawned-wp))))
    then
      (bind ?offset 10)
      (assert
        (plan-action (id 1) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?bs ?bs-side)))
        (plan-action (id 2) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?bs ?bs-side))
        (plan-action (id 3) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name move)
              (param-names r from from-side to to-side )
              (param-values ?robot (wait-pos ?bs ?bs-side) WAIT ?bs ?bs-side))
        (plan-action (id 4) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name lock) (param-values ?bs))
        (plan-action (id 5) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name prepare-bs)
              (param-names m side bc)
              (param-values ?bs ?bs-side ?base-color))
        (plan-action (id 6) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name bs-dispense)
              (param-names r m side wp basecol)
              (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
        (plan-action (id 7) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name wp-get)
              (param-names r wp m side)
              (param-values ?robot ?spawned-wp ?bs ?bs-side))
        (plan-action (id 8) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name unlock) (param-values ?bs))
        (plan-action (id 9) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?bs ?bs-side))
        (plan-action (id 10) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?bs ?bs-side (wait-pos ?mps INPUT))))
    else
      (bind ?offset 1)
      (assert
        (plan-action (id 1) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps INPUT))))
  )
  (assert
  (plan (id PRODUCE-C0-PLAN) (goal-id ?goal-id))
  (plan-action (id (+ ?offset 1)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps INPUT))
  (plan-action (id (+ ?offset 2)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot (wait-pos ?mps INPUT) WAIT ?mps INPUT))
  (plan-action (id (+ ?offset 3)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?spawned-wp ?mps))
  (plan-action (id (+ ?offset 4)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name request-cs-mount-cap)
        (param-values ?robot ?mps ?spawned-wp ?cap-color))
  (plan-action (id (+ ?offset 5)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps INPUT))
  (plan-action (id (+ ?offset 6)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?mps INPUT (wait-pos ?mps INPUT)))
 )
 (modify ?g (mode EXPANDED))
)

(defrule goal-mount-first-ring
 ?p <- (goal (id ?parent))
 ?g <- (goal (id ?goal-id) (class MOUNT-FIRST-RING) (mode SELECTED)
             (parent ?parent)
             (params robot ?robot
                      bs ?bs
                      bs-side ?bs-side
                      bs-color ?base-color
                      mps ?mps
                      ring-color ?ring-color
                      rs-before ?rs-before
                      rs-after ?rs-after
                      rs-req ?rs-req
                      order ?order
                      wp ?spawned-wp
             ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?spawned-wp))))
    then
      (bind ?offset 10)
      (assert
        (plan (id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?bs ?bs-side)))
        (plan-action (id 2) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?bs ?bs-side))
        (plan-action (id 3) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name move)
              (param-names r from from-side to to-side )
              (param-values ?robot (wait-pos ?bs ?bs-side) WAIT ?bs ?bs-side))
        (plan-action (id 4) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name lock) (param-values ?bs))
        (plan-action (id 5) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name prepare-bs)
              (param-names m side bc)
              (param-values ?bs ?bs-side ?base-color))
        (plan-action (id 6) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name bs-dispense)
              (param-names r m side wp basecol)
              (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
        (plan-action (id 7) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name wp-get)
              (param-names r wp m side)
              (param-values ?robot ?spawned-wp ?bs ?bs-side))
        (plan-action (id 8) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name unlock) (param-values ?bs))
        (plan-action (id 9) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?bs ?bs-side))
        (plan-action (id 10) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?bs ?bs-side (wait-pos ?mps INPUT))))
    else
      (bind ?offset 1)
      (assert
        (plan-action (id 1) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps INPUT))))
  )
  (assert
  (plan (id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id))
  (plan-action (id (+ ?offset 1)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps INPUT))
  (plan-action (id (+ ?offset 2)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot (wait-pos ?mps INPUT) WAIT ?mps INPUT))
  (plan-action (id (+ ?offset 3)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?spawned-wp ?mps))
  (plan-action (id (+ ?offset 4)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name request-rs-mount-ring)
        (param-values ?robot ?mps ?spawned-wp ONE ?ring-color
                       RING_NONE RING_NONE RING_NONE
                       ?rs-req))
  (plan-action (id (+ ?offset 5)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps INPUT))
  (plan-action (id (+ ?offset 6)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?mps INPUT (wait-pos ?mps INPUT)))
 )
 (modify ?g (mode EXPANDED))
)


(defrule goal-mount-next-ring
 ?p <- (goal (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id)
             (class MOUNT-NEXT-RING)
                                             (params robot ?robot
                                                      prev-rs ?prev-rs
                                                      prev-rs-side ?prev-rs-side
                                                      wp ?wp
                                                      rs ?rs
                                                      ring1-color ?ring1-color
                                                      ring2-color ?ring2-color
                                                      ring3-color ?ring3-color
                                                      curr-ring-color ?curr-ring-color
                                                      ring-pos ?ring-pos
                                                      rs-before ?rs-before
                                                      rs-after ?rs-after
                                                      rs-req ?rs-req
                                                      order ?order
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?wp))))
    then
      (bind ?offset 8)
      (assert
      (plan-action (id 1) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name go-wait)
            (param-names r from from-side to)
            (param-values ?robot ?curr-location ?curr-side (wait-pos ?prev-rs ?prev-rs-side)))
      (plan-action (id 2) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name location-lock)
            (param-values ?prev-rs ?prev-rs-side))
      (plan-action (id 3) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name move)
            (param-names r from from-side to to-side)
            (param-values ?robot (wait-pos ?prev-rs ?prev-rs-side) WAIT ?prev-rs ?prev-rs-side))
      (plan-action (id 4) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name wp-get)
            (param-names r wp m side)
            (param-values ?robot ?wp ?prev-rs ?prev-rs-side))
      (plan-action (id 5) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name location-unlock)
            (param-values ?prev-rs ?prev-rs-side))
      (plan-action (id 6) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name go-wait)
            (param-names r from from-side to)
            (param-values ?robot ?prev-rs ?prev-rs-side (wait-pos ?rs INPUT))))
    else
      (bind ?offset 1)
      (assert
        (plan-action (id 1) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?rs INPUT))))
  )
    (assert
      (plan (id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id))
      (plan-action (id (+ ?offset 1)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name location-lock)
            (param-values ?rs INPUT))
      (plan-action (id (+ ?offset 2)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name move)
            (param-names r from from-side to to-side )
            (param-values ?robot (wait-pos ?rs INPUT) WAIT ?rs INPUT))
      (plan-action (id (+ ?offset 3)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name wp-put)
            (param-names r wp m)
            (param-values ?robot ?wp ?rs))
      (plan-action (id (+ ?offset 4)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name request-rs-mount-ring)
            (param-values ?robot ?rs ?wp ?ring-pos ?curr-ring-color
                              ?ring1-color ?ring2-color ?ring3-color ?rs-req))
      (plan-action (id (+ ?offset 5)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name location-unlock)
            (param-values ?rs INPUT))
      (plan-action (id (+ ?offset 6)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name go-wait)
            (param-names r from from-side to)
            (param-values ?robot ?rs INPUT (wait-pos ?rs INPUT)))
     )
    (modify ?g (mode EXPANDED))
)


(defrule goal-produce-cx
 ?p <- (goal (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id) (class PRODUCE-CX)
                                              (params robot ?robot
                                                        wp ?wp
                                                        rs ?rs
                                                        mps ?mps
                                                        cs-color ?cap-color
                                                        order ?order
                                                        ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?wp))))
    then
      (bind ?offset 8)
      (assert
        (plan-action (id 1) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?rs OUTPUT)))
        (plan-action (id 2) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?rs OUTPUT))
        (plan-action (id 3) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name move)
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?rs OUTPUT) WAIT ?rs OUTPUT))
        (plan-action (id 4) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name lock)
              (param-values ?rs))
        (plan-action (id 5) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name wp-get)
              (param-names r wp m side)
              (param-values ?robot ?wp ?rs OUTPUT))
        (plan-action (id 6) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name unlock)
              (param-values ?rs))
        (plan-action (id 7) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?rs OUTPUT))
        (plan-action (id 8) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?rs OUTPUT (wait-pos ?mps INPUT))))
    else
      (bind ?offset 1)
      (assert
        (plan-action (id 1) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps INPUT))))
  )
  (assert
    (plan (id PRODUCE-CX-PLAN) (goal-id ?goal-id))
    (plan-action (id (+ ?offset 1)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name location-lock)
          (param-values ?mps INPUT))
    (plan-action (id (+ ?offset 2)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name move)
          (param-names r from from-side to to-side)
          (param-values ?robot (wait-pos ?mps INPUT) WAIT ?mps INPUT))
    (plan-action (id (+ ?offset 3)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name wp-put)
          (param-names r wp m)
          (param-values ?robot ?wp ?mps))
    (plan-action (id (+ ?offset 4)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name request-cs-mount-cap)
          (param-values ?robot ?mps ?wp ?cap-color))
    (plan-action (id (+ ?offset 5)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name location-unlock)
          (param-values ?mps INPUT))
    (plan-action (id (+ ?offset 6)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name go-wait)
          (param-names r from from-side to)
          (param-values ?robot ?mps INPUT (wait-pos ?mps INPUT)))
   )
  (modify ?g (mode EXPANDED))
)



(defrule goal-reset-mps
  ?p <- (goal (id ?parent))
  ?g <- (goal (id ?goal-id) (class RESET-MPS) (mode SELECTED) (parent ?parent)
              (params r ?robot
                      m ?mps))
  =>
  (assert
    (plan (id RESET-MPS-PLAN) (goal-id ?goal-id))
      (plan-action (id 1) (plan-id RESET-MPS-PLAN) (goal-id ?goal-id)
        (action-name reset-mps)
        (param-names m)
        (param-values ?mps))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-deliver
 ?p <- (goal (id ?parent))
 ?g <- (goal (id ?goal-id) (class DELIVER) (mode SELECTED) (parent ?parent)
             (params robot ?robot
                          mps ?mps
                          order ?order
                          wp ?wp
                          ds ?ds
                          ds-gate ?gate
                          base-color ?base-color
                          ring1-color ?ring1-color
                          ring2-color ?ring2-color
                          ring3-color ?ring3-color
                          cap-color ?cap-color
       ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key order meta wp-for-order args? wp ?wp ord ?ord))
 (wm-fact (key domain fact order-complexity args? ord ?ord com ?complexity))
 =>
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?wp))))
    then
      (bind ?offset 8)
      (assert
        (plan-action (id 1) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps OUTPUT)))
        (plan-action (id 2) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?mps OUTPUT))
        (plan-action (id 3) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name move)
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?mps OUTPUT) WAIT ?mps OUTPUT))
        (plan-action (id 4) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name lock) (param-values ?mps))
        (plan-action (id 5) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name wp-get)
              (param-names r wp m side)
              (param-values ?robot ?wp ?mps OUTPUT))
        (plan-action (id 6) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name unlock) (param-values ?mps))
        (plan-action (id 7) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?mps OUTPUT))
        (plan-action (id 8) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?mps OUTPUT (wait-pos ?ds INPUT))))
    else
      (bind ?offset 1)
      (assert
        (plan-action (id 1) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?ds INPUT))))
  )
 (assert
  (plan (id DELIVER-PLAN) (goal-id ?goal-id))
  (plan-action (id (+ ?offset 1)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?ds INPUT))
  (plan-action (id (+ ?offset 2)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot (wait-pos ?ds INPUT) WAIT ?ds INPUT))
  (plan-action (id (+ ?offset 3)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?ds))
  (plan-action (id (+ ?offset 4)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?wp ?ds))
  (plan-action (id (+ ?offset 5)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name prepare-ds)
        (param-names m ord)
        (param-values ?ds ?order)))

 (bind ?param-names (create$ ord wp m g basecol capcol))
 (bind ?param-values (create$ ?order ?wp ?ds ?gate ?base-color ?cap-color))
 (switch ?complexity
  (case C1 then
      (bind ?param-names (create$ ord wp m g basecol capcol ring1col))
      (bind ?param-values (create$ ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color)))
  (case C2 then
      (bind ?param-names (create$ ord wp m g basecol capcol ring1col ring2col))
      (bind ?param-values (create$ ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color ?ring2-color)))
  (case C3 then
      (bind ?param-names (create$ ord wp m g basecol capcol ring1col ring2col ring3col))
      (bind ?param-values (create$ ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)))
 )
 (assert
   (plan-action (id (+ ?offset 6)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name (sym-cat fulfill-order- (lowcase ?complexity)))
        (param-names ?param-names)
        (param-values ?param-values))
   (plan-action (id (+ ?offset 7)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?ds))
   (plan-action (id (+ ?offset 8)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?ds INPUT))
   (plan-action (id (+ ?offset 9)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?ds INPUT (wait-pos ?ds INPUT)))
 )
 (modify ?g (mode EXPANDED))
)

(defrule goal-expander-wait-mps-process
  ?p <- (goal (id ?parent))
  ?g <- (goal (id ?goal-id) (class WAIT-FOR-MPS-PROCESS) (mode SELECTED) (parent ?parent)
            (params robot ?robot
                    pos ?pos)
  )
  (wm-fact (key domain fact at args? r ?robot m ?mps-other side ?side-other))
  =>
  (assert
    (plan (id WAIT-FOR-MPS-PROCESS-PLAN) (goal-id ?goal-id))
  )
  (if (eq ?pos ?mps-other) then 
    (assert
      (plan-action (id 1) (plan-id WAIT-FOR-MPS-PROCESS-PLAN) (goal-id ?goal-id)
                        (action-name wait)
                        (param-values ?robot ?pos))
    )
  else
    (assert 
      (plan-action (id 1) (plan-id WAIT-FOR-MPS-PROCESS-PLAN) (goal-id ?goal-id)
                        (action-name go-wait)
                        (param-values ?robot ?mps-other ?side-other ?pos))
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-process-mps
 ?p <- (goal (id ?parent))
 ?g <- (goal (id ?goal-id) (class PROCESS-MPS) (mode SELECTED) (parent ?parent)
             (params m ?mps))
  ?pre <- (wm-fact (key mps-handling prepare ?prepare-action ?mps args? $?prepare-params))
  ?pro <- (wm-fact (key mps-handling process ?process-action ?mps args? $?process-params))
  =>
  (bind ?prepare-param-values (values-from-name-value-list ?prepare-params))
  (bind ?process-param-values (values-from-name-value-list ?process-params))

  (bind ?success TRUE)
  ;Special case: if process is a ring mount, we have to get the current number of rings stored
  ; and adapt the param-values
  (if (eq ?prepare-action prepare-rs) then
    (bind ?rs-req (nth$ (+ 1 (member$ r-req ?prepare-params)) ?prepare-params))
    (if (not (do-for-fact ((?wm wm-fact)) (and (wm-key-prefix ?wm:key (create$ domain fact rs-filled-with))
                                       (eq (wm-key-arg ?wm:key m) ?mps))
      (bind ?rs-before (wm-key-arg ?wm:key n))
    )) then
      (printout error "Cant find rs-filled-with for " ?mps crlf)
      (bind ?success FALSE)
    )
    (if (not (do-for-fact ((?wm wm-fact)) (and (wm-key-prefix ?wm:key (create$ domain fact rs-sub))
                                       (eq (wm-key-arg ?wm:key minuend) ?rs-before)
                                       (eq (wm-key-arg ?wm:key subtrahend) ?rs-req))
      (bind ?rs-after (wm-key-arg ?wm:key difference))
    )) then
      (printout error "Cant find rs-sub fact with " ?rs-before "-" ?rs-req crlf)
      (bind ?success FALSE)
    )
    (if ?success then
      (bind ?prepare-param-values (insert$ ?prepare-param-values (length$ ?prepare-param-values) ?rs-before))
      (bind ?process-param-values (insert$ ?process-param-values (length$ ?process-param-values) ?rs-before))

      (bind ?prepare-param-values (insert$ ?prepare-param-values (length$ ?prepare-param-values) ?rs-after))
      (bind ?process-param-values (insert$ ?process-param-values (length$ ?process-param-values) ?rs-after))
    )
  )

  (if ?success then
    (assert
      (plan (id (sym-cat PROCESS-MPS- ?mps)) (goal-id ?goal-id))
      (plan-action (id 1) (plan-id (sym-cat PROCESS-MPS- ?mps)) (goal-id ?goal-id)
            (action-name lock) (param-values ?mps))
      (plan-action (id 2) (plan-id (sym-cat PROCESS-MPS- ?mps)) (goal-id ?goal-id)
        (action-name ?prepare-action)
        (param-values ?prepare-param-values))
      (plan-action (id 3) (plan-id (sym-cat PROCESS-MPS- ?mps)) (goal-id ?goal-id)
                                    (action-name ?process-action)
                                    (param-values ?process-param-values))
      (plan-action (id 4) (plan-id (sym-cat PROCESS-MPS- ?mps)) (goal-id ?goal-id)
            (action-name unlock)
            (param-values ?mps))
    )
    (modify ?g (mode EXPANDED))
  else
    (retract ?pre ?pro)
    (printout error "Tried to expand Process RS goal on changed fact" crlf)
    (modify ?g (mode RETRACTED) (outcome REJECTED))
  )
)

; Centralized Goal Reasoning
(defrule goal-expander-calc-action-duration
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 ?pf <- (plan-action (state FORMULATED)
                   (plan-id ?plan-id)
                   (id ?action-id)
                   (action-name ?action-name)
                   (param-names $?param-names)
                   (param-values $?param-values)
                   (duration ?d1&:(<= ?d1 0)))
 (not (plan-action (action-name ?action-name)
                   (param-names $?param-names )
                   (param-values $?param-values)
                   (duration ?d2&:(> ?d2 0))))
 (skill-action-mapping (name ?action-name))
=>


 ;(bind ?duration (map-action-skill (str-cat ?action-name) ?param-names ?param-values))
 ;(printout error "mapped to " ?duration crlf))
 (bind ?duration (integer (round (estimate-action-duration (str-cat ?action-name) ?param-names ?param-values))))
 (if (> ?duration 0)
     then
       (modify ?pf (duration ?duration))
     else
      (printout error "Cant find duration for action id " ?action-id "in plan " ?plan-id crlf))
)

(defrule goal-expander-copy-action-duration
 (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
 ?pf <- (plan-action (state FORMULATED)
                   (action-name ?action-name)
                   (param-names $?param-names)
                   (param-values $?param-values)
                   (duration ?d1&:(<= ?d1 0)))
 (plan-action (action-name ?action-name)
              (param-names $?param-names )
              (param-values $?param-values)
              (duration ?d2&:(> ?d2 0)))
=>
 (modify ?pf  (duration ?d2))
)

(defrule goal-expander-setup-robot
   "Move a robot from one location to another to satisfy setup requirements "
   (declare (salience ?*SALIENCE-GOAL-EXPAND*))
    ?g <- (goal (class SETUP) (id ?goal-id) (mode SELECTED)
                (params r ?r-id
                        setup1 $?setup1
                        setup2 $?setup2))
    (resource (id ?r-id) (entity ?robot) (type ROBOT))
    (resource-setup (resource-id ?r-id)
                    (from-state $?setup1)
                    (to-state $?setup2)
                    (duration ?setup-duration))
   (not (plan (goal-id ?goal-id)))
   =>
   (bind ?plan-id  (sym-cat ?goal-id _P ))

   (assert
      (wm-fact (key meta plan required-resource args? id ?plan-id r ?robot
                                setup [ ?setup1 ] ))
      (wm-fact (key meta plan released-resource args? id ?plan-id r ?robot
                                setup [ ?setup2 ] ))

        (plan (id ?plan-id) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names (create$ r from from-side to to-side))
                                    (param-values (create$ ?robot (nth$ 1 ?setup1) (nth$ 2 ?setup1)
                                                                  (nth$ 1 ?setup2) (nth$ 2 ?setup2)))
                                    (duration  ?setup-duration))
    )

)


(defrule goal-expander-fill-cap-station-central
   "Feed a CS with a cap from its shelf so that afterwards
   it can directly put the cap on a product."
   (declare (salience ?*SALIENCE-GOAL-EXPAND*))
   ?g <- (goal (id ?goal-id) (class FILL-CAP) (params $?params) (mode SELECTED))
   (test (subsetp (create$ cap-station cc shelf-spot cap-color) ?params))

   ;Resources groundable during scheduling (Schedulable resources)
   (wm-fact (key domain fact at args? r ?robot m ? side ?))
   (wm-fact (key domain fact can-hold args? r ?robot))
   =>
   (bind ?cs (nth$ (+ 1 (member$ cap-station ?params)) ?params))
   (bind ?cc (nth$ (+ 1 (member$ cc ?params)) ?params))
   (bind ?shelf-spot (nth$ (+ 1 (member$ shelf-spot ?params)) ?params))
   (bind ?cap-color (nth$ (+ 1 (member$ cap-color ?params)) ?params))


   (bind ?plan-id (formate-event-name (sym-cat ?goal-id _ ?robot _ ?cs )))

   (assert
      (wm-fact (key meta plan required-resource args? id ?plan-id r ?cc setup [ ] ))
      (wm-fact (key meta plan released-resource args? id ?plan-id r ?cc setup [ ] ))

      (wm-fact (key meta plan required-resource args? id ?plan-id r ?cs setup [ ] ))
      (wm-fact (key meta plan released-resource args? id ?plan-id r ?cs setup [ ] ))

      (wm-fact (key meta plan required-resource args? id ?plan-id r ?robot
                                setup [ (wait-pos ?cs INPUT) WAIT ] ))
      (wm-fact (key meta plan released-resource args? id ?plan-id r ?robot
                                setup [ (wait-pos ?cs INPUT) WAIT ] ))

        (plan (id ?plan-id) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?cs INPUT))
        (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?cs INPUT) WAIT ?cs INPUT))
        (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name wp-get-shelf)
                                    (param-names r cc m spot)
                                    (param-values ?robot ?cc ?cs ?shelf-spot))
        (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name wp-put)
                                    (param-names r wp m)
                                    (param-values ?robot ?cc ?cs))
        (plan-action (id 13) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name lock)
                                    (param-values ?cs))
        (plan-action (id 14) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name prepare-cs)
                                    (param-names m op)
                                    (param-values ?cs RETRIEVE_CAP))
        (plan-action (id 15) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name cs-retrieve-cap)
                                    (param-names m cc capcol)
                                    (param-values ?cs ?cc ?cap-color))
        (plan-action (id 16) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name unlock)
                                    (param-values ?cs))
        (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?cs INPUT))
        (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot ?cs INPUT (wait-pos ?cs INPUT) WAIT))
    )
)

(defrule goal-remove-workpiece-from-mps-central
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 ?g <- (goal (id ?goal-id) (class PREPARE-CS) (params $?params) (mode SELECTED))
 (test (subsetp (create$ cc cap-station) ?params))

 ;Resources groundable during scheduling (Schedulable resources)
 (wm-fact (key domain fact at args? r ?robot m ? side ?))
 (wm-fact (key domain fact can-hold args? r ?robot))
 =>
 (bind ?mps (nth$ (+ 1 (member$ cap-station ?params)) ?params))
 (bind ?cc (nth$ (+ 1 (member$ cc ?params)) ?params))
 (bind ?side OUTPUT)


 (bind ?plan-id  (formate-event-name (sym-cat ?goal-id _ ?robot _ ?mps)))

 (assert
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?cc setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?cc setup [ ] ))

  (wm-fact (key meta plan required-resource args? id ?plan-id r ?mps setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?mps setup [ ] ))

  (wm-fact (key meta plan required-resource args? id ?plan-id r ?robot
                     setup [ (wait-pos ?mps OUTPUT) WAIT ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?robot
                     setup [ (wait-pos ?mps OUTPUT) WAIT ] ))


  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps ?side))
  (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot (wait-pos ?mps ?side) WAIT ?mps ?side))
  (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?cc ?mps ?side))
  (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps ?side))
  (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?mps ?side (wait-pos ?mps ?side) WAIT))
     (plan-action (id 6) (plan-id ?plan-id ) (goal-id ?goal-id)
          (action-name wp-discard)
          (param-names r cc )
          (param-values ?robot ?cc))

 )
)


(defrule goal-mount-cap-central
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 ?g <- (goal (id ?goal-id) (class MOUNT-CAP) (params $?params) (mode SELECTED))
 (test (subsetp (create$ wp cap-station cap-color) ?params))

 (wm-fact (key domain fact mps-type args? m ?prior-mps t ?prior-mps-type))
 (or (and (test (member$ (create$ com C0) ?params))
          (test (member$ (create$ base-station ?prior-mps) ?params))
          (domain-object (name ?prior-side&~WAIT) (type mps-side)))
     (and (test (member$ (create$ com C1) ?params))
          (test (member$ (create$ ring1-station ?prior-mps) ?params))
          (domain-object (name ?prior-side&OUTPUT) (type mps-side)))
     (and (test (member$ (create$ com C2) ?params))
          (test (member$ (create$ ring2-station ?prior-mps) ?params))
          (domain-object (name ?prior-side&OUTPUT) (type mps-side)))
     (and (test (member$ (create$ com C3) ?params))
          (test (member$ (create$ ring3-station ?prior-mps) ?params))
          (domain-object (name ?prior-side&OUTPUT) (type mps-side))))


 (wm-fact (key domain fact at args? r ?robot m ? side ?))
 (wm-fact (key domain fact can-hold args? r ?robot))
 =>
 (bind ?wp (nth$ (+ 1 (member$ wp ?params)) ?params))
 (bind ?cs (nth$ (+ 1 (member$ cap-station ?params)) ?params))
 (bind ?cap-color (nth$ (+ 1 (member$ cap-color ?params)) ?params))

 (bind ?plan-id  (formate-event-name (sym-cat ?goal-id _ ?robot  ?prior-mps  (lowcase ?prior-side) _ ?cs )))

 (assert
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?wp setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?wp setup [ ] ))
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?cs setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?cs setup [ ] ))
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?prior-mps setup [ ]  ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?prior-mps setup [ ]  ))

  (wm-fact (key meta plan required-resource args? id ?plan-id r ?robot
                setup [ (wait-pos ?prior-mps ?prior-side) WAIT ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?robot
                setup [ (wait-pos ?cs INPUT) WAIT ] )))


 (assert
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
         (action-name location-lock)
         (param-values ?prior-mps ?prior-side))
  (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
         (action-name move)
         (param-names r from from-side to to-side )
         (param-values ?robot (wait-pos ?prior-mps ?prior-side) WAIT ?prior-mps ?prior-side))
  (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
         (action-name lock) (param-values ?prior-mps)))

 (if (eq ?prior-mps-type BS) then
     (bind ?base-color (nth$ (+ 1 (member$ base-color ?params)) ?params))
     (assert (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name prepare-bs) (param-names m side bc)
                          (param-values ?prior-mps ?prior-side ?base-color))
             (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name bs-dispense) (param-names r m side wp basecol)
                          (param-values ?robot ?prior-mps ?prior-side ?wp ?base-color))))

 (assert
  (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
         (action-name wp-get)
         (param-names r wp m side)
         (param-values ?robot ?wp ?prior-mps ?prior-side))
  (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
         (action-name unlock) (param-values ?prior-mps))
  (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?prior-mps ?prior-side))
  (plan-action (id 9) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?prior-mps ?prior-side (wait-pos ?cs INPUT) WAIT))
  (plan-action (id 10) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?cs INPUT))
  (plan-action (id 11) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot (wait-pos ?cs INPUT) WAIT ?cs INPUT))
  (plan-action (id 12) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?wp ?cs))
  (plan-action (id 13) (plan-id ?plan-id ) (goal-id ?goal-id)
        (action-name lock)
        (param-values ?cs))
  (plan-action (id 14) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name prepare-cs)
        (param-names m op)
        (param-values ?cs MOUNT_CAP))
  (plan-action (id 15) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name cs-mount-cap)
        (param-names m wp capcol)
        (param-values ?cs ?wp ?cap-color))
  (plan-action (id 16) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name unlock)
        (param-values ?cs))
  (plan-action (id 17) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?cs INPUT))
  (plan-action (id 19) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?cs INPUT (wait-pos ?cs INPUT) WAIT))
 )
)

(defrule goal-deliver-central
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 ?g <- (goal (id ?goal-id) (class DELIVER) (params $?params) (mode SELECTED))
 (test (subsetp (create$ ord com base-color ring1-color ring2-color ring3-color
                         cap-color gate wp cap-station delivery-station ) ?params))

 ; Resources groundable during scheduling (Schedulable resources)
 ;ROBOT CEs
 (wm-fact (key domain fact at args? r ?robot m ? side ?))
 (wm-fact (key domain fact can-hold args? r ?robot))
 =>
 (bind ?wp (nth$ (+ 1 (member$ wp ?params)) ?params))
 (bind ?order (nth$ (+ 1 (member$ ord ?params)) ?params))
 (bind ?gate (nth$ (+ 1 (member$ gate ?params)) ?params))
 (bind ?complexity (nth$ (+ 1 (member$ com ?params)) ?params))
 (bind ?cap-color (nth$ (+ 1 (member$ cap-color ?params)) ?params))
 (bind ?base-color (nth$ (+ 1 (member$ base-color ?params)) ?params))
 (bind ?ring1-color (nth$ (+ 1 (member$ ring1-color ?params)) ?params))
 (bind ?ring2-color (nth$ (+ 1 (member$ ring2-color ?params)) ?params))
 (bind ?ring3-color (nth$ (+ 1 (member$ ring3-color ?params)) ?params))
 (bind ?cs (nth$ (+ 1 (member$ cap-station ?params)) ?params))
 (bind ?ds (nth$ (+ 1 (member$ delivery-station ?params)) ?params))

 (bind ?plan-id (formate-event-name (sym-cat ?goal-id _ ?robot _ ?cs _ ?ds )))
 (assert
     (wm-fact (key meta plan required-resource args? id ?plan-id r ?wp setup [ ] ))
     (wm-fact (key meta plan released-resource args? id ?plan-id r ?wp setup [ ] ))

     (wm-fact (key meta plan required-resource args? id ?plan-id r ?ds setup [ ] ))
     (wm-fact (key meta plan released-resource args? id ?plan-id r ?ds setup [ ] ))

     (wm-fact (key meta plan required-resource args? id ?plan-id r ?cs setup [ ]  ))
     (wm-fact (key meta plan released-resource args? id ?plan-id r ?cs setup [ ] ))

     (wm-fact (key meta plan required-resource args? id ?plan-id r ?robot
                        setup [ (wait-pos ?cs OUTPUT) WAIT ] ))
     (wm-fact (key meta plan released-resource args? id ?plan-id r ?robot
                        setup [ (wait-pos ?ds INPUT) WAIT ] ))

     (plan (id ?plan-id) (goal-id ?goal-id))
     (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name location-lock)
                  (param-values ?cs OUTPUT))
     (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move)
                  (param-names r from from-side to to-side)
                  (param-values ?robot (wait-pos ?cs OUTPUT) WAIT ?cs OUTPUT))
     (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name lock)
			   (param-values ?cs))
     (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name wp-get)
                  (param-names r wp m side)
                  (param-values ?robot ?wp ?cs OUTPUT))
     (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name unlock)
			   (param-values ?cs))
     (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name location-unlock)
                  (param-values ?cs OUTPUT))
     (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move)
                  (param-names r from from-side to to-side)
                  (param-values ?robot ?cs OUTPUT (wait-pos ?ds INPUT) WAIT))
     (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name location-lock)
                  (param-values ?ds INPUT))
     (plan-action (id 9) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move)
                  (param-names r from from-side to to-side)
                  (param-values ?robot (wait-pos ?ds INPUT) WAIT ?ds INPUT))
     (plan-action (id 10) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name lock)
			   (param-values ?ds))
     (plan-action (id 11) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name wp-put)
                  (param-names r wp m)
                  (param-values ?robot ?wp ?ds))
     (plan-action (id 12) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name prepare-ds)
                  (param-names m ord)
                  (param-values ?ds ?order))
     )
 (bind ?param-names (create$ ord wp m g basecol capcol))
 (bind ?param-values (create$ ?order ?wp ?ds ?gate ?base-color ?cap-color))
 (switch ?complexity
  (case C1 then
      (bind ?param-names (create$ ord wp m g basecol capcol ring1col))
      (bind ?param-values (create$ ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color)))
  (case C2 then
      (bind ?param-names (create$ ord wp m g basecol capcol ring1col ring2col))
      (bind ?param-values (create$ ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color ?ring2-color)))
  (case C3 then
      (bind ?param-names (create$ ord wp m g basecol capcol ring1col ring2col ring3col))
      (bind ?param-values (create$ ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)))
 )
 (assert
   (plan-action (id 13) (plan-id ?plan-id) (goal-id ?goal-id)
                (action-name (sym-cat fulfill-order- (lowcase ?complexity)))
                (param-names ?param-names)
                (param-values ?param-values))
   (plan-action (id 14) (plan-id ?plan-id) (goal-id ?goal-id)
                (action-name unlock)
	           (param-values ?ds))
   (plan-action (id 15) (plan-id ?plan-id) (goal-id ?goal-id)
                (action-name location-unlock)
                (param-values ?ds INPUT))
   (plan-action (id 16) (plan-id ?plan-id) (goal-id ?goal-id)
                (action-name move)
                (param-names r from from-side to to-side)
                (param-values ?robot ?ds INPUT (wait-pos ?ds INPUT) WAIT))
 )
)

(defrule goal-mount-ring-central
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?goal-id) (class ?class) (params $?params) (mode SELECTED))
 (test (subsetp (create$ wp) ?params))

 (wm-fact (key domain fact mps-type args? m ?prior-mps t ?prior-mps-type))
 (wm-fact (key domain fact mps-type args? m ?curr-mps t RS))
 (wm-fact (key domain fact rs-ring-spec args? m ?curr-mps r ?ring-color rn ?rs-req))
 (or (and (test (eq ?class MOUNT-RING1))
          (test (member$ (create$ base-station ?prior-mps) ?params))
          (test (member$ (create$ ring1-station ?curr-mps) ?params))
          (test (member$ (create$ ring1-color ?ring-color) ?params))
          (domain-object (name ?prior-side&INPUT|OUTPUT) (type mps-side))
          )
     (and (test (eq ?class MOUNT-RING2))
          (test (member$ (create$ ring1-station ?prior-mps) ?params))
          (test (member$ (create$ ring2-station ?curr-mps) ?params))
          (test (member$ (create$ ring2-color ?ring-color) ?params))
          (domain-object (name ?prior-side&OUTPUT) (type mps-side))
          )
     (and (test (eq ?class MOUNT-RING3))
          (test (member$ (create$ ring2-station ?prior-mps) ?params))
          (test (member$ (create$ ring3-station ?curr-mps) ?params))
          (test (member$ (create$ ring3-color ?ring-color) ?params))
          (domain-object (name ?prior-side&OUTPUT) (type mps-side))
          ))

 ;Resources groundable during scheduling (Schedulable resources)
 (wm-fact (key domain fact at args? r ?robot m ? side ?))
 (wm-fact (key domain fact can-hold args? r ?robot))
 =>
 (bind ?wp (nth$ (+ 1 (member$ wp ?params)) ?params))
 (bind ?ring1-color (nth$ (+ 1 (member$ ring1-color ?params)) ?params))
 (bind ?ring2-color (nth$ (+ 1 (member$ ring2-color ?params)) ?params))
 (bind ?ring3-color (nth$ (+ 1 (member$ ring3-color ?params)) ?params))

 (bind ?ring-pos ONE)
 (if (eq ?class MOUNT-RING2) then (bind ?ring-pos TWO))
 (if (eq ?class MOUNT-RING3) then (bind ?ring-pos THREE))

 (bind ?plan-id  (formate-event-name (sym-cat ?goal-id _ ?robot _ ?prior-mps (lowcase ?prior-side) _ ?curr-mps )))

 ;late binding of rs-before and rs-after
 (bind ?binding-id (sym-cat X (gensym*)))
 (bind ?rs-before  (sym-cat ?binding-id #n))
 (bind ?fact-key (create$ domain fact rs-filled-with args? m ?curr-mps n ?rs-before))
 (assert (wm-fact (key meta binding args? id ?binding-id policy BIND-ANY)
                  (is-list TRUE)
                  (values $?fact-key)))

 (bind ?binding-id (sym-cat X (gensym*)))
 (bind ?rs-after  (sym-cat ?binding-id #difference))
 (bind ?fact-key (create$ domain fact rs-sub args? minuend ?rs-before subtrahend ?rs-req difference ?rs-after))
 (assert (wm-fact (key meta binding args? id ?binding-id policy BIND-ANY)
                  (is-list TRUE)
                  (values $?fact-key)))

 ;Mount ring action params
 (bind ?ring-colors (create$))
 (switch (sym-to-int ?ring-pos)
      (case 1 then TRUE)
      (case 2 then
        (bind ?ring-colors ?ring1-color))
      (case 3 then
        (bind ?ring-colors (create$ ?ring1-color ?ring2-color)))
     (default
        (printout t "ERROR, plan-action params of request-rs-mount-ring are wrong" crlf)))

 (assert
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?wp setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?wp setup [ ] ))
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?prior-mps setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?prior-mps setup [ ] ))
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?curr-mps setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?curr-mps setup [ ] ))
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?robot
                setup [ (wait-pos ?prior-mps ?prior-side) WAIT ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?robot
                setup [ (wait-pos ?curr-mps INPUT) WAIT ] )))

 (assert
        (plan (id ?plan-id) (goal-id ?goal-id))

        (plan-action (id 1) (action-name location-lock)
                     (plan-id ?plan-id) (goal-id ?goal-id)
                     (param-values ?prior-mps ?prior-side))
        (plan-action (id 2) (action-name move)
                     (plan-id ?plan-id) (goal-id ?goal-id)
                     (param-names r from from-side to to-side )
                     (param-values ?robot
                                   (wait-pos ?prior-mps ?prior-side)
                                   WAIT
                                   ?prior-mps
                                   ?prior-side))
        (plan-action (id 3) (action-name lock)
                     (plan-id ?plan-id) (goal-id ?goal-id)
                     (param-values ?prior-mps)))

 (if (eq ?prior-mps-type BS) then
     (bind ?base-color (nth$ (+ 1 (member$ base-color ?params)) ?params))
     (assert (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name prepare-bs) (param-names m side bc)
                          (param-values ?prior-mps ?prior-side ?base-color))
             (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name bs-dispense) (param-names r m side wp basecol)
                          (param-values ?robot ?prior-mps ?prior-side ?wp ?base-color))))

 (assert
        (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name wp-get) (param-names r wp m side)
                     (param-values ?robot ?wp ?prior-mps ?prior-side))
        (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
                      (action-name unlock) (param-values ?prior-mps))
        (plan-action (id 9) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name location-unlock)
                     (param-values ?prior-mps ?prior-side))
        (plan-action (id 10) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name move) (param-names r from from-side to to-side)
                     (param-values ?robot ?prior-mps ?prior-side (wait-pos ?curr-mps INPUT) WAIT))
        (plan-action (id 11) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name location-lock)
                     (param-values ?curr-mps INPUT))
        (plan-action (id 12) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name move) (param-names r from from-side to to-side)
                     (param-values ?robot (wait-pos ?curr-mps INPUT) WAIT ?curr-mps INPUT))
        (plan-action (id 13) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name wp-put) (param-names r wp m)
                     (param-values ?robot ?wp ?curr-mps))
        (plan-action (id 14) (plan-id ?plan-id ) (goal-id ?goal-id)
                     (action-name lock)
                     (param-values ?curr-mps))
        (plan-action (id 15) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name prepare-rs)
                     (param-values ?curr-mps ?ring-color
                                   ?rs-before ?rs-after ?rs-req))
        (plan-action (id 16) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name (sym-cat rs-mount-ring (sym-to-int ?ring-pos)))
                     (param-values ?curr-mps ?wp ?ring-color ?ring-colors
                                   ?rs-before ?rs-after ?rs-req))
        (plan-action (id 17) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name unlock)
                      (param-values ?curr-mps))
        (plan-action (id 18) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name location-unlock)
                     (param-values ?curr-mps INPUT))
        (plan-action (id 19) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name move)
                     (param-names r from from-side to to-side)
                     (param-values ?robot ?curr-mps INPUT (wait-pos ?curr-mps INPUT) WAIT))
 )
)

(defrule goal-expander-fill-rs-central
 (declare (salience ?*SALIENCE-GOAL-EXPAND*))
 (goal (id ?goal-id) (class FILL-RS) (params $?params) (mode SELECTED))
 (test (subsetp (create$ fill-rs) ?params))

 ;Resources groundable during scheduling (Schedulable resources)
 (wm-fact (key domain fact at args? r ?robot m ? side ?))
 (wm-fact (key domain fact can-hold args? r ?robot))

 (wm-fact (key refbox team-color) (value ?team-color))
 (wm-fact (key domain fact mps-team args? m ?from-mps col ?team-color))
 (wm-fact (key domain fact mps-type args? m ?from-mps t ?from-type))
 (or (and (test (eq ?from-type BS))
          (domain-object (name ?from-side&INPUT|OUTPUT) (type mps-side)))
     (and (test (eq ?from-type CS))
          (domain-object (name ?from-side&INPUT) (type mps-side))))

 ;(wm-fact (key domain fact rs-inc args? summand ?last-filled sum ?fill-base#))
 ;(test (member$ (create$ fill-base# ?fill-base#) ?params))
 =>
 (bind ?rs (nth$ (+ 1 (member$ fill-rs ?params)) ?params))
 (bind ?plan-id (formate-event-name (sym-cat ?goal-id _ ?robot _ ?from-mps (lowcase ?from-side) _  ?rs )))

 (bind ?binding-id (sym-cat X (gensym*)))
 (bind ?spot (sym-cat ?binding-id #spot))
 (bind ?wp  (sym-cat ?binding-id #wp))
 (if (eq ?from-type BS) then
     (bind ?fact-key (create$ domain fact wp-unused args? wp ?wp)))
 (if (eq ?from-type CS) then
     (bind ?fact-key (create$ domain fact wp-on-shelf args? wp ?wp m ?from-mps spot ?spot)))
 (assert (wm-fact (key meta binding args? id ?binding-id policy BIND-UNIQUE)
                  (is-list TRUE)
                  (values $?fact-key)))

 (bind ?binding-id (sym-cat X (gensym*)))
 (bind ?rs-before  (sym-cat ?binding-id #n))
 (bind ?fact-key (create$ domain fact rs-filled-with args? m ?rs n ?rs-before))
 (assert (wm-fact (key meta binding args? id ?binding-id policy BIND-ANY)
                  (is-list TRUE)
                  (values $?fact-key)))

 (bind ?binding-id (sym-cat X (gensym*)))
 (bind ?rs-after  (sym-cat ?binding-id #sum))
 (bind ?fact-key (create$ domain fact rs-inc args? summand ?rs-before sum ?rs-after))
 (assert (wm-fact (key meta binding args? id ?binding-id policy BIND-ANY)
                  (is-list TRUE)
                  (values $?fact-key)))



 (assert
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?wp setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?wp setup [ ] ))
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?from-mps setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?from-mps setup [ ] ))
  (wm-fact (key meta plan required-resource args? id ?plan-id r (sym-cat ?rs SLIDE) setup [ ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r (sym-cat ?rs SLIDE) setup [ ] ))
  (wm-fact (key meta plan required-resource args? id ?plan-id r ?robot
                setup [ (wait-pos ?from-mps ?from-side) WAIT ] ))
  (wm-fact (key meta plan released-resource args? id ?plan-id r ?robot
                setup [ (wait-pos ?rs INPUT) WAIT ] )))

 ;(wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before&ZERO|ONE|TWO))

 (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name location-lock)
                 (param-values ?from-mps ?from-side))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name move)
                 (param-names r from from-side to to-side)
                 (param-values ?robot
                               (wait-pos ?from-mps ?from-side) WAIT
                               ?from-mps ?from-side)))

 (if (eq ?from-type BS) then
     (bind ?base-color (nth$ (+ 1 (member$ base-color ?params)) ?params))
     (assert (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name lock)
                          (param-values ?from-mps))
             (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name prepare-bs) (param-names m side bc)
                          (param-values ?from-mps ?from-side ?base-color))
             (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name bs-dispense) (param-names r m side wp basecol)
                          (param-values ?robot ?from-mps ?from-side ?wp ?base-color))
             (plan-action (id 6) (plan-id  ?plan-id) (goal-id ?goal-id)
                          (action-name wp-get)
                          (param-names r wp m side)
                          (param-values ?robot ?wp ?from-mps ?from-side))
             (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name unlock)
                          (param-values ?from-mps))))

  (if (and (eq ?from-type CS) (eq ?from-side INPUT)) then
     (assert (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                          (action-name wp-get-shelf)
                          (param-names r cc m spot)
                          (param-values ?robot ?wp ?from-mps ?spot))))

 (assert
    (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name location-unlock)
                  (param-values ?from-mps ?from-side))
     (plan-action (id 9) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move)
                  (param-names r from from-side to to-side)
                  (param-values ?robot
                                ?from-mps ?from-side
                                (wait-pos ?rs INPUT)
                                WAIT))
     (plan-action (id 10) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name location-lock)
                 (param-values ?rs INPUT))
     (plan-action (id 11) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move)
                  (param-names r from from-side to to-side)
                  (param-values ?robot
                               (wait-pos ?rs INPUT) WAIT
                               ?rs INPUT))
     (plan-action (id 12) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name lock)
                  (param-values ?rs))
     (plan-action (id 13) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name wp-put-slide-cc)
                  (param-names r wp m rs-before rs-after)
                  (param-values ?robot ?wp ?rs ?rs-before ?rs-after))
     (plan-action (id 14) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name unlock)
                  (param-values ?rs))
     (plan-action (id 15) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name location-unlock)
                  (param-values ?rs INPUT))
     (plan-action (id 16) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name move)
                  (param-names r from from-side to to-side)
                  (param-values ?robot ?rs INPUT (wait-pos ?rs INPUT) WAIT))
  )
)
