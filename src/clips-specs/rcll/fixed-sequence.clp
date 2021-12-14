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


(defrule goal-expander-spawn-ss-c0
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
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
   )

  (bind ?offset 1)
  (if (any-factp ((?at-output wm-fact))
                 (and (wm-key-prefix ?at-output:key (create$ domain fact wp-at))
                      (eq (wm-key-arg ?at-output:key m) ?mps)
                      (eq (wm-key-arg ?at-output:key side) OUTPUT)
                 ))
    then
      (assert
        (plan-action (id 6) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                     (action-name request-cs-retrieve-cap)
                     (param-values ?robot ?mps ?cc ?cap-color))
      )
    else
      (bind ?offset 4)
      (assert
        (plan-action (id 6) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
              (action-name lock) (param-values ?mps))
        (plan-action (id 7) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                     (action-name prepare-cs)
                     (param-values ?mps RETRIEVE_CAP))
        (plan-action (id 8) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                     (action-name cs-retrieve-cap)
                     (param-values ?mps ?cc ?cap-color))
        (plan-action (id 9) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                     (action-name unlock)
                     (param-values ?mps))
      )
  )
  (assert
      (plan-action (id (+ ?offset 6)) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                   (action-name location-unlock)
                   (param-values ?mps INPUT))
      (plan-action (id (+ ?offset 7)) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
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


(defrule goal-expander-discard
 ?p <- (goal (mode DISPATCHED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class DISCARD) (mode SELECTED)
             (parent ?parent)
             (params robot ?robot
                     wp ?wp
             ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (bind ?plan-id DISCARD-PLAN)
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
          (action-name wp-discard)
          (param-names r cc )
          (param-values ?robot ?wp))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-get-and-discard
 ?p <- (goal (mode DISPATCHED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class GET-AND-DISCARD) (mode SELECTED)
             (parent ?parent)
             (params robot ?robot
                     wp ?wp
                     mps ?wp-loc
                     side ?wp-side
             ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (bind ?offset 0)
  (bind ?plan-id GET-AND-DISCARD-PLAN)
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id)))
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?wp))))
    then
      (bind ?offset 8)
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?wp-loc ?wp-side)))
        (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?wp-loc ?wp-side))
        (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?wp-loc ?wp-side) WAIT ?wp-loc ?wp-side))
        (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name lock)
              (param-values ?wp-loc))
        (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (param-names r wp m side)
              (param-values ?robot ?wp ?wp-loc ?wp-side))
        (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name unlock)
              (param-values ?wp-loc))
        (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?wp-loc ?wp-side))
        (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?wp-loc ?wp-side (wait-pos ?wp-loc ?wp-side))))
  )
  (assert
    (plan-action (id (+ ?offset 1)) (plan-id ?plan-id) (goal-id ?goal-id)
          (action-name wp-discard)
          (param-names r cc )
          (param-values ?robot ?wp))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-get-shelf-and-fill-rs
 ?p <- (goal (mode DISPATCHED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class ?class&GET-SHELF-AND-FILL-RS)
             (mode SELECTED) (parent ?parent)
             (params robot ?robot
                     cs ?cs
                     rs ?rs
                     wp ?cc
                     spot ?spot
                     rs-before ?rs-before
                     rs-after ?rs-after
             )
       )
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (bind ?offset 1)
  (bind ?plan-id (sym-cat ?class -PLAN))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id)))
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?cc))))
    then
      (bind ?offset 8)
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?cs INPUT)))
        (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?cs INPUT))
        (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?cs INPUT) WAIT ?cs INPUT))
      ;   (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      ;         (action-name lock)
      ;         (param-values ?cs))
        (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get-shelf)
              (param-values ?robot ?cc ?cs ?spot))
      ;   (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
      ;         (action-name unlock)
      ;         (param-values ?cs))
        (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?cs INPUT))
        (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?cs INPUT (wait-pos ?rs INPUT))))
     else
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?rs INPUT)))
      )
  )
  (assert
    (plan-action (id (+ ?offset 1)) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?rs INPUT))
    (plan-action (id (+ ?offset 2)) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?rs INPUT) WAIT ?rs INPUT))
    (plan-action (id (+ ?offset 3)) (plan-id ?plan-id) (goal-id ?goal-id)
          (action-name wp-put-slide-cc)
          (param-names r wp m rs-before rs-after)
          (param-values ?robot ?cc ?rs ?rs-before ?rs-after))
    (plan-action (id (+ ?offset 4)) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?rs INPUT))
    (plan-action (id (+ ?offset 5)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?rs INPUT (wait-pos ?rs INPUT)))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-fill-rs
 ?p <- (goal (mode DISPATCHED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class ?class&FILL-RS|
                                         GET-AND-FILL-RS)
             (mode SELECTED) (parent ?parent)
             (params robot ?robot
                      mps ?mps
                      wp ?wp
                      rs-before ?rs-before
                      rs-after ?rs-after
             ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (bind ?plan-id (sym-cat ?class -PLAN))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id)))

  (bind ?offset 1)
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?wp))))
    then
      (do-for-fact ((?wp-at wm-fact))
                 (and (wm-key-prefix ?wp-at:key (create$ domain fact wp-at))
                      (eq (wm-key-arg ?wp-at:key wp) ?wp))
      (bind ?offset 8)
      (bind ?wp-loc (wm-key-arg ?wp-at:key m))
      (bind ?wp-side (wm-key-arg ?wp-at:key side))
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?wp-loc ?wp-side)))
        (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-lock)
              (param-values ?wp-loc ?wp-side))
        (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?wp-loc ?wp-side) WAIT ?wp-loc ?wp-side))
        (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name lock)
              (param-values ?wp-loc))
        (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (param-names r wp m side)
              (param-values ?robot ?wp ?wp-loc ?wp-side))
        (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name unlock)
              (param-values ?wp-loc))
        (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-unlock)
              (param-values ?wp-loc ?wp-side))
        (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?wp-loc ?wp-side (wait-pos ?mps INPUT))))
      )
     else
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps INPUT)))
      )
  )
  (assert
    (plan-action (id (+ ?offset 1)) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?mps INPUT))
    (plan-action (id (+ ?offset 2)) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?mps INPUT) WAIT ?mps INPUT))
    (plan-action (id (+ ?offset 3)) (plan-id ?plan-id) (goal-id ?goal-id)
          (action-name wp-put-slide-cc)
          (param-names r wp m rs-before rs-after)
          (param-values ?robot ?wp ?mps ?rs-before ?rs-after))
    (plan-action (id (+ ?offset 4)) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?mps INPUT))
    (plan-action (id (+ ?offset 5)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?mps INPUT (wait-pos ?mps INPUT)))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-get-base-and-fill-rs
 ?p <- (goal (mode DISPATCHED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id)
             (class GET-BASE-AND-FILL-RS)
             (params robot ?robot
                      bs ?bs
                      bs-side ?bs-side
                      base-color ?base-color
                      wp ?spawned-wp
                      rs ?rs
                      rs-before ?rs-before
                      rs-after ?rs-after
       ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (assert
  (plan (id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?curr-location ?curr-side (wait-pos ?bs ?bs-side)))
  (plan-action (id 2) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?bs ?bs-side))
  (plan-action (id 3) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot (wait-pos ?bs ?bs-side) WAIT ?bs ?bs-side))
  (plan-action (id 4) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name lock)
        (param-values ?bs))
  (plan-action (id 5) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name prepare-bs)
        (param-values ?bs ?bs-side ?base-color))
  (plan-action (id 6) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name bs-dispense-trash)
        (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
  (plan-action (id 7) (plan-id  GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?spawned-wp ?bs ?bs-side))
  (plan-action (id 8) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name unlock)
        (param-values ?bs))
  (plan-action (id 9) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?bs ?bs-side))
  (plan-action (id 10) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?bs ?bs-side (wait-pos ?rs INPUT)))
    (plan-action (id 11) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?rs INPUT))
    (plan-action (id 12) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?rs INPUT) WAIT ?rs INPUT))
    (plan-action (id 13) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name wp-put-slide-cc)
          (param-names r wp m rs-before rs-after)
          (param-values ?robot ?spawned-wp ?rs ?rs-before ?rs-after))
    (plan-action (id 14) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?rs INPUT))
    (plan-action (id 15) (plan-id GET-BASE-AND-FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name go-wait)
        (param-names r from from-side to)
        (param-values ?robot ?rs INPUT (wait-pos ?rs INPUT)))

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
  (bind ?offset 1)
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
  )

  (if (any-factp ((?at-output wm-fact))
                 (and (wm-key-prefix ?at-output:key (create$ domain fact wp-at))
                      (eq (wm-key-arg ?at-output:key m) ?mps)
                      (eq (wm-key-arg ?at-output:key side) OUTPUT)
                 ))
    then
      (assert
   (plan-action (id (+ ?offset 4)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
         (action-name request-cs-mount-cap)
         (param-values ?robot ?mps ?spawned-wp ?cap-color))
      )
      (bind ?offset (+ ?offset 1))
    else
      (assert
   (plan-action (id (+ ?offset 4)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
                (action-name lock) (param-values ?mps))
   (plan-action (id (+ ?offset 5)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
                (action-name prepare-cs)
                (param-values ?mps MOUNT_CAP))
   (plan-action (id (+ ?offset 6)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
                (action-name cs-mount-cap)
                (param-values ?mps ?spawned-wp ?cap-color))
   (plan-action (id (+ ?offset 7)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
                (action-name unlock)
                (param-values ?mps))
      )
      (bind ?offset (+ ?offset 4))
  )
  (assert
  (plan-action (id (+ ?offset 4)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps INPUT))
  (plan-action (id (+ ?offset 5)) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
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
  (bind ?offset 1)
  (bind ?mount-ring-param-values ?mps ?spawned-wp ?ring-color ?rs-before ?rs-after ?rs-req)
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
  )
  (if (any-factp ((?at-output wm-fact))
                 (and (wm-key-prefix ?at-output:key (create$ domain fact wp-at))
                      (eq (wm-key-arg ?at-output:key m) ?mps)
                      (eq (wm-key-arg ?at-output:key side) OUTPUT)
                 ))
    then
  (assert
   (plan-action (id (+ ?offset 4)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
         (action-name request-rs-mount-ring)
         (param-values ?robot ?mps ?spawned-wp ONE ?ring-color
                        RING_NONE RING_NONE RING_NONE
                        ?rs-req))
  )
  (bind ?offset (+ ?offset 1))
    else
  (assert
  (plan-action (id (+ ?offset 4)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?mps))

  (plan-action (id (+ ?offset 5)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
               (action-name prepare-rs)
               (param-values ?mps ?ring-color ?rs-before ?rs-after ?rs-req))
  (plan-action (id (+ ?offset 6)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
               (action-name rs-mount-ring1)
               (param-values ?mount-ring-param-values))
  (plan-action (id (+ ?offset 7)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
               (action-name unlock)
               (param-values ?mps))
  )
  (bind ?offset (+ ?offset 4))
  )
  (assert
  (plan-action (id (+ ?offset 4)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps INPUT))
  (plan-action (id (+ ?offset 5)) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
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
  (bind ?offset 1)
  (bind ?mount-ring-action-name (sym-cat rs-mount-ring (sym-to-int ?ring-pos)))
  (switch (sym-to-int ?ring-pos)
    (case 1 then
      (bind ?mount-ring-param-values ?rs ?wp ?curr-ring-color ?rs-before ?rs-after ?rs-req))
    (case 2 then
      (bind ?mount-ring-param-values ?rs ?wp ?curr-ring-color ?ring1-color ?rs-before ?rs-after ?rs-req))
    (case 3 then
      (bind ?mount-ring-param-values ?rs ?wp ?curr-ring-color ?ring1-color ?ring2-color ?rs-before ?rs-after ?rs-req))
   (default
      (printout t "ERROR, plan-action params of request-rs-mount-ring are wrong" crlf)))
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
    )

  (if (any-factp ((?at-output wm-fact))
                 (and (wm-key-prefix ?at-output:key (create$ domain fact wp-at))
                      (eq (wm-key-arg ?at-output:key m) ?rs)
                      (eq (wm-key-arg ?at-output:key side) OUTPUT)
                 ))
    then
  (assert
     (plan-action (id (+ ?offset 4)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
           (action-name request-rs-mount-ring)
           (param-values ?robot ?rs ?wp ?ring-pos ?curr-ring-color
                             ?ring1-color ?ring2-color ?ring3-color ?rs-req))
  )
  (bind ?offset (+ ?offset 1))
    else
  (assert
  (plan-action (id (+ ?offset 4)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?rs))
  (plan-action (id (+ ?offset 5)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
               (action-name prepare-rs)
               (param-values ?rs ?curr-ring-color ?rs-before ?rs-after ?rs-req))
  (plan-action (id (+ ?offset 6)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
               (action-name ?mount-ring-action-name)
               (param-values ?mount-ring-param-values))
  (plan-action (id (+ ?offset 7)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
               (action-name unlock)
               (param-values ?rs))
  )
  (bind ?offset (+ ?offset 4))
  )
  (assert
      (plan-action (id (+ ?offset 4)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
            (action-name location-unlock)
            (param-values ?rs INPUT))
      (plan-action (id (+ ?offset 5)) (plan-id MOUNT-NEXT-RING-PLAN) (goal-id ?goal-id)
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
  (bind ?offset 1)
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
  )
  (if (any-factp ((?at-output wm-fact))
                 (and (wm-key-prefix ?at-output:key (create$ domain fact wp-at))
                      (eq (wm-key-arg ?at-output:key m) ?mps)
                      (eq (wm-key-arg ?at-output:key side) OUTPUT)
                 ))
    then
  (assert
     (plan-action (id (+ ?offset 4)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
           (action-name request-cs-mount-cap)
           (param-values ?robot ?mps ?wp ?cap-color))
  )
  (bind ?offset (+ ?offset 1))
    else
  (assert
     (plan-action (id (+ ?offset 4)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
                  (action-name lock) (param-values ?mps))
     (plan-action (id (+ ?offset 5)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
                  (action-name prepare-cs)
                  (param-values ?mps MOUNT_CAP))
     (plan-action (id (+ ?offset 6)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
                  (action-name cs-mount-cap)
                  (param-values ?mps ?wp ?cap-color))
     (plan-action (id (+ ?offset 7)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
                  (action-name unlock)
                  (param-values ?mps))
  )
  (bind ?offset (+ ?offset 4))
  )
  (assert
    (plan-action (id (+ ?offset 4)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name location-unlock)
          (param-values ?mps INPUT))
    (plan-action (id (+ ?offset 5)) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
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
 (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
 (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
 (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
 (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
 (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
 ;Order-CEs
 (wm-fact (key domain fact wp-for-order args? wp ?wp ord ?ord))
 (wm-fact (key domain fact order-complexity args? ord ?ord com ?complexity))
 (wm-fact (key domain fact order-base-color args? ord ?ord col ?base-color))
 (wm-fact (key domain fact order-ring1-color args? ord ?ord col ?ring1-color))
 (wm-fact (key domain fact order-ring2-color args? ord ?ord col ?ring2-color))
 (wm-fact (key domain fact order-ring3-color args? ord ?ord col ?ring3-color))
 (wm-fact (key domain fact order-cap-color args? ord ?ord col ?cap-color))
 (wm-fact (key domain fact order-gate args? ord ?ord gate ?gate))
 =>
  (bind ?offset 1)
 (bind ?param-values (create$ ?ord ?wp ?ds ?gate ?base-color ?cap-color))
 (switch ?complexity
  (case C1 then
      (bind ?param-values (create$ ?ord ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color)))
  (case C2 then
      (bind ?param-values (create$ ?ord ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color ?ring2-color)))
  (case C3 then
      (bind ?param-values (create$ ?ord ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)))
 )
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
               (param-values ?ds ?order))
  (plan-action (id (+ ?offset 6)) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
               (action-name (sym-cat fulfill-order- (lowcase ?complexity)))
               (param-values ?param-values))
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
