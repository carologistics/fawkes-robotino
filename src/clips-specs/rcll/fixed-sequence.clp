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


(defrule goal-expander-spawn-wp
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class SPAWN-WP) (mode SELECTED)
              (parent ?parent-id) (params robot ?robot))
=>
  (assert
    (plan (id SPAWNPLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id SPAWNPLAN) (goal-id ?goal-id)
                 (action-name spawn-wp)
                 (param-values (sym-cat WP- (random-id)) ?robot))
  )
  (modify ?g (mode EXPANDED))
)
(defrule goal-expander-ss-assign-wp-to-order
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class SS-ASSIGN-WP) (mode SELECTED)
              (parent ?parent-id)
              (params robot ?robot mps ?ss old-wp ?old-wp wp ?wp order ?order
                      shelf ?shelf slot ?slot
                      base-col ?base-col ring1-col ?ring1-col
                      ring2-col ?ring2-col ring3-col ?ring3-col
                      cap-col ?cap-col))
=>
  (assert
    (plan (id SS-ASSIGN-WP-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id SS-ASSIGN-WP-PLAN) (goal-id ?goal-id)
                                (action-name lock)
                                (param-values ?ss))
    (plan-action (id 2) (plan-id SS-ASSIGN-WP-PLAN) (goal-id ?goal-id)
                 (action-name prepare-ss-to-assign-wp)
                 (param-names m r old-wp wp shelf slot base-col ring1-col
                                ring2-col ring3-col cap-col)
                 (param-values ?ss ?robot ?old-wp ?wp ?shelf ?slot ?base-col
                               ?ring1-col ?ring2-col ?ring3-col ?cap-col))
    (plan-action (id 3) (plan-id SS-ASSIGN-WP-PLAN) (goal-id ?goal-id)
                                (action-name unlock)
                                (param-values ?ss))
    (plan-action (id 4) (plan-id SS-ASSIGN-WP-PLAN) (goal-id ?goal-id)
                                (action-name assign-wp-to-order)
                                (param-values ?order ?wp ?base-col ?ring1-col
                                              ?ring2-col ?ring3-col ?cap-col))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-store-wp
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class STORE-WP) (mode SELECTED)
              (parent ?parent-id) (params robot ?robot mps ?mps side ?side ss ?ss wp ?wp shelf ?shelf slot ?slot))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
=>
   (assert
        (plan (id STORE-WP-PLAN) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (param-names r from from-side to)
                                    (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps ?side)))
        (plan-action (id 2) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?mps ?side))
        (plan-action (id 3) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?mps ?side) WAIT ?mps ?side))
        (plan-action (id 4) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name wp-get)
                                    (param-names r wp m side)
                                    (param-values ?robot ?wp ?mps ?side))
        (plan-action (id 5) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?mps ?side))
        (plan-action (id 6) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (param-names r from from-side to)
                                    (param-values ?robot ?mps ?side (wait-pos ?ss INPUT)))
        (plan-action (id 7) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?ss INPUT))
        (plan-action (id 8) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?ss INPUT) WAIT ?ss INPUT))
        (plan-action (id 9) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name lock)
                                    (param-values ?ss))
        (plan-action (id 10) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name wp-put)
                                    (param-names r wp m)
                                    (param-values ?robot ?wp ?ss))
        (plan-action (id 11) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name prepare-ss-to-store)
                                    (param-names m wp op shelf slot)
                                    (param-values ?ss ?wp STORE ?shelf ?slot))
        (plan-action (id 12) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name ss-store-wp)
                                    (param-names m wp shelf slot)
                                    (param-values ?ss ?wp ?shelf ?slot))
        (plan-action (id 13) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name unlock)
                                    (param-values ?ss))
        (plan-action (id 14) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?ss INPUT))
        (plan-action (id 15) (plan-id STORE-WP-PLAN) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (param-names r from from-side to)
                                    (param-values ?robot ?ss INPUT (wait-pos ?ss INPUT)))
    )
    (modify ?g (mode EXPANDED))
)


(defrule goal-expander-retrieve-wp
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class GET-STORED-WP) (mode SELECTED)
              (parent ?parent-id) (params robot ?robot ss ?ss wp ?wp shelf ?shelf slot ?slot))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
=>
   (assert
        (plan (id GET-STORED-WP-PLAN) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name lock)
                                    (param-values ?ss))
        (plan-action (id 2) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name prepare-ss-to-retrieve)
                                    (param-names m wp op shelf slot)
                                    (param-values ?ss ?wp RETRIEVE ?shelf ?slot))
        (plan-action (id 3) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name ss-retrieve-wp)
                                    (param-names m wp shelf slot)
                                    (param-values ?ss ?wp ?shelf ?slot))
        (plan-action (id 4) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name unlock)
                                    (param-values ?ss))
        (plan-action (id 5) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (param-names r from from-side to)
                                    (param-values ?robot ?curr-location ?curr-side (wait-pos ?ss OUTPUT)))
        (plan-action (id 6) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?ss OUTPUT))
        (plan-action (id 7) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?ss OUTPUT) WAIT ?ss OUTPUT))
        (plan-action (id 8) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name wp-get)
                                    (param-names r wp m side)
                                    (param-values ?robot ?wp ?ss OUTPUT))
        (plan-action (id 9) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?ss OUTPUT))
        (plan-action (id 10) (plan-id GET-STORED-WP-PLAN) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (param-names r from from-side to)
                                    (param-values ?robot ?ss OUTPUT (wait-pos ?ss OUTPUT)))
    )
    (modify ?g (mode EXPANDED))
)

(defrule goal-expander-refill-shelf
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
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
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
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
  ?p <- (goal (mode DISPATCHED) (id ?parent))
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
   ?p <- (goal (mode DISPATCHED) (id ?parent))
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
    ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
        (action-name bs-dispense-trash)
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
              (action-name bs-dispense-for-order)
              (param-names r m side ord wp basecol)
              (param-values ?robot ?bs ?bs-side ?order ?spawned-wp ?base-color))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
              (action-name bs-dispense-for-order)
              (param-names r m side ord wp basecol)
              (param-values ?robot ?bs ?bs-side ?order ?spawned-wp ?base-color))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
  ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 (wm-fact (key domain fact wp-for-order args? wp ?wp ord ?ord))
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
        (action-name request-ds-fulfill-order)
        (param-names r m wp ord)
        (param-values ?robot ?ds ?wp ?order))
)

 (assert
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
  ?p <- (goal (mode DISPATCHED) (id ?parent))
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
 ?p <- (goal (mode DISPATCHED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class PROCESS-MPS) (mode SELECTED) (parent ?parent)
             (params m ?mps $?other-goal-args))
  ?pre <- (wm-fact (key mps-handling prepare ?prepare-action ?mps args? $?prepare-params))
  ?pro <- (wm-fact (key mps-handling process ?process-action ?mps args? $?process-params))
  ; The DS might have multiple pending mps-handling facts, they can be
  ; distinguished by additional params (wp and order id) that are present
  ; both in the goal and the mps-handling facts
  (test (and (member$ ?other-goal-args ?prepare-params)
             (member$ ?other-goal-args ?process-params)))
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
