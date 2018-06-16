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
(defrule goal-expander-acquire-token
  ?g <- (goal (id ?goal-id) (class ACQUIRE-TOKEN) (mode SELECTED)
                            (params token-name ?token-name))
=>
  (assert
    (plan (id ACQUIRE-TOKEN-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ACQUIRE-TOKEN-PLAN) (goal-id ?goal-id)
                    (action-name lock)
                    (param-values ?token-name)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-send-beacon-signal
  ?p <- (goal (mode EXPANDED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class BEACONACHIEVE) (mode SELECTED)
              (parent ?parent-id))
=>
  (assert
    (plan (id BEACONPLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id BEACONPLAN) (goal-id ?goal-id)
      (action-name send-beacon)))
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-wp-spawn
  ?p <- (goal (mode EXPANDED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class WPSPAWN-ACHIEVE) (mode SELECTED)
              (parent ?parent-id))
=>
  (assert
    (plan (id SPAWNPLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id SPAWNPLAN) (goal-id ?goal-id)
      (action-name noop)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-refill-shelf
  ?p <- (goal (mode EXPANDED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class REFILL-SHELF-ACHIEVE) (mode SELECTED)
              (parent ?parent-id))
  =>
  (assert
     (plan (id REFILLPLAN) (goal-id ?goal-id))
     (plan-action (id 1) (plan-id REFILLPLAN) (goal-id ?goal-id)
        (action-name noop)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-exploration
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class EXPLORATION))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact self args? r ?r))
  (wm-fact (id ?id&: (eq ?id (str-cat "/config/rcll/route/" ?team-color "/" ?r))) (values $?route))
  =>
  (assert (plan (goal-id ?goal-id) (id EXPLORATION-PLAN)))
  (bind ?action-id 3)
  (foreach ?node ?route
	(assert (plan-action (id ?action-id) (goal-id ?goal-id) (plan-id EXPLORATION-PLAN) (action-name move-node) (param-values ?r ?node)))
	(bind ?action-id (+ ?action-id 3))
  )
  (modify ?g (mode EXPANDED))
)



(defrule goal-expander-enter-field
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD-ACHIEVE))
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
  ?p <- (goal (mode EXPANDED) (id ?parent))
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
   ?p <- (goal (mode EXPANDED) (id ?parent))
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
    ?p <- (goal (mode EXPANDED) (id ?parent))
    ?g <- (goal (id ?goal-id) (class FILL-CAP) (mode SELECTED) (parent ?parent)
                (params robot ?robot
                        mps ?mps
                        cc ?cc
                ))
    (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
    (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?shelf-spot))
    (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
    =>
   (assert
        (plan (id FILL-CAP-PLAN) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (param-values ?mps INPUT))
        (plan-action (id 2) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot ?curr-location ?curr-side ?mps INPUT))
        (plan-action (id 3) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name lock)
                                    (param-values ?mps))
        (plan-action (id 4) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name wp-get-shelf)
                                    (param-names r cc m spot)
                                    (param-values ?robot ?cc ?mps ?shelf-spot))
        (plan-action (id 5) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name wp-put)
                                    (param-names r wp m)
                                    (param-values ?robot ?cc ?mps))
        (plan-action (id 6) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name prepare-cs)
                                    (param-names m op)
                                    (param-values ?mps RETRIEVE_CAP))
        (plan-action (id 7) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name cs-retrieve-cap)
                                    (param-names m cc capcol)
                                    (param-values ?mps ?cc ?cap-color))
        (plan-action (id 8) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name unlock)
                                    (param-values ?mps))
        (plan-action (id 9) (plan-id FILL-CAP-PLAN) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?mps INPUT))

    )
    (modify ?g (mode EXPANDED))
)

(defrule goal-remove-workpiece-from-mps
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class CLEAR-MPS) (mode SELECTED) (parent ?parent)
             (params robot ?robot
                      mps ?mps
                      wp ?wp
                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (assert
  (plan (id CLEAR-MPS-PLAN) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps OUTPUT))
  (plan-action (id 2) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?curr-location ?curr-side ?mps OUTPUT))
  (plan-action (id 3) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?mps))
  (plan-action (id 4) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?wp ?mps OUTPUT))
  (plan-action (id 5) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?mps))
  (plan-action (id 6) (plan-id CLEAR-MPS-PLAN) (goal-id ?goal-id)
                              (action-name location-unlock)
                              (param-values ?mps OUTPUT))
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
 ?p <- (goal (mode EXPANDED) (id ?parent))
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
 ?p <- (goal (mode EXPANDED) (id ?parent))
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
                                    (action-name location-lock)
                                    (param-values ?mps INPUT))
    (plan-action (id 2) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot ?curr-location ?curr-side ?mps INPUT))
    (plan-action (id 3) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?mps))
    (plan-action (id 4) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
          (action-name wp-put-slide-cc)
          (param-names r wp m rs-before rs-after)
          (param-values ?robot ?wp ?mps ?rs-before ?rs-after))
    (plan-action (id 5) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?mps))
    (plan-action (id 6) (plan-id FILL-RS-PLAN) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (param-values ?mps INPUT))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-fill-rs-from-bs
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id)
             (class FILL-RS-FROM-BS)
             (params robot ?robot
                      mps ?mps
                      bs ?bs
                      bs-side ?bs-side
                      base-color ?base-color
                      rs-before ?rs-before
                      rs-after ?rs-after
                      wp ?spawned-wp
       ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (assert
  (plan (id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?bs ?bs-side))
  (plan-action (id 2) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot ?curr-location ?curr-side ?bs ?bs-side))
  (plan-action (id 3) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?bs))
  (plan-action (id 4) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name prepare-bs)
        (param-names m side bc)
        (param-values ?bs ?bs-side ?base-color))
  (plan-action (id 5) (plan-id  FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name bs-dispense)
        (param-names r m side wp basecol)
        (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
  (plan-action (id 6) (plan-id  FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?spawned-wp ?bs ?bs-side))
  (plan-action (id 7) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?bs))
  (plan-action (id 8) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?bs ?bs-side))
  (plan-action (id 9) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps INPUT))
   (plan-action (id 10) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
                                  (action-name move)
                                  (param-names r from from-side to to-side)
                                  (param-values ?robot ?bs ?bs-side ?mps INPUT))
  (plan-action (id 11) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name lock)
        (param-values ?mps))
  (plan-action (id 12) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name wp-put-slide-cc)
        (param-names r wp m rs-before rs-after)
        (param-values ?robot ?spawned-wp ?mps ?rs-before ?rs-after))
  (plan-action (id 13) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name unlock)
        (param-values ?mps))
  (plan-action (id 14) (plan-id FILL-RS-FROM-BS-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps INPUT))
 )
 (modify ?g (mode EXPANDED))
)


(defrule goal-expander-fill-rs-from-shelf
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id)
             (class FILL-RS-FROM-SHELF)
             (params robot ?robot
                      mps ?mps
                      cs ?cs
                      wp ?wp
                      spot ?spot
                      rs-before ?rs-before
                      rs-after ?rs-after
       ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
 (assert
    (plan (id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name location-lock)
          (param-values ?cs INPUT))
    (plan-action (id 2) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name move)
          (param-names r from from-side to to-side )
          (param-values ?robot ?curr-location ?curr-side ?cs INPUT))
    (plan-action (id 3) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name lock) (param-values ?cs))
    (plan-action (id 4) (plan-id  FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name wp-get-shelf)
          (param-names r cc m spot)
          (param-values ?robot ?wp ?cs ?spot))
    (plan-action (id 5) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name unlock) (param-values ?cs))
    (plan-action (id 6) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name location-unlock)
          (param-values ?cs INPUT))
    (plan-action (id 7) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name location-lock)
          (param-values ?mps INPUT))
     (plan-action (id 8) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot ?cs INPUT ?mps INPUT))
    (plan-action (id 9) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name lock)
          (param-values ?mps))
    (plan-action (id 10) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name wp-put-slide-cc)
          (param-names r wp m rs-before rs-after)
          (param-values ?robot ?wp ?mps ?rs-before ?rs-after))
    (plan-action (id 11) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name unlock)
          (param-values ?mps))
    (plan-action (id 12) (plan-id FILL-RS-FROM-SHELF-PLAN) (goal-id ?goal-id)
          (action-name location-unlock)
          (param-values ?mps INPUT))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-produce-c0
 ?p <- (goal (mode EXPANDED) (id ?parent))
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
 (assert
  (plan (id PRODUCE-C0-PLAN) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?bs ?bs-side))
  (plan-action (id 2) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot ?curr-location ?curr-side ?bs ?bs-side))
  (plan-action (id 3) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?bs))
  (plan-action (id 4) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name prepare-bs)
        (param-names m side bc)
        (param-values ?bs ?bs-side ?base-color))
  (plan-action (id 5) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name bs-dispense)
        (param-names r m side wp basecol)
        (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
  (plan-action (id 6) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?spawned-wp ?bs ?bs-side))
  (plan-action (id 7) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?bs))
  (plan-action (id 8) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?bs ?bs-side))
  (plan-action (id 9) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps INPUT))
  (plan-action (id 10) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?bs ?bs-side ?mps INPUT))
  (plan-action (id 11) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?mps))
  (plan-action (id 12) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?spawned-wp ?mps))
  (plan-action (id 13) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name prepare-cs)
        (param-names m op)
        (param-values ?mps MOUNT_CAP))
  (plan-action (id 14) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name cs-mount-cap)
        (param-names m wp capcol)
        (param-values ?mps ?spawned-wp ?cap-color ))
  (plan-action (id 15) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?mps))
  (plan-action (id 16) (plan-id PRODUCE-C0-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps INPUT))
 )
 (modify ?g (mode EXPANDED))
)

(defrule goal-mount-first-ring
 ?p <- (goal (mode EXPANDED) (id ?parent))
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
 (assert
  (plan (id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?bs ?bs-side))
  (plan-action (id 2) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot ?curr-location ?curr-side ?bs ?bs-side))
  (plan-action (id 3) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?bs))
  (plan-action (id 4) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name prepare-bs)
        (param-names m side bc)
        (param-values ?bs ?bs-side ?base-color))
  (plan-action (id 5) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name bs-dispense)
        (param-names r m side wp basecol)
        (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
  (plan-action (id 6) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?spawned-wp ?bs ?bs-side))
  (plan-action (id 7) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?bs))
  (plan-action (id 8) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?bs ?bs-side))
  (plan-action (id 9) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps INPUT))
  (plan-action (id 10) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side )
        (param-values ?robot ?bs ?bs-side ?mps INPUT))
  (plan-action (id 11) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?mps))
  (plan-action (id 12) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?spawned-wp ?mps))
  (plan-action (id 13) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name prepare-rs)
        (param-names m rc rs-before rs-after r-req)
        (param-values ?mps ?ring-color ?rs-before ?rs-after ?rs-req))
   (plan-action (id 14) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name rs-mount-ring1)
        (param-names m wp col rs-before rs-after r-req)
        (param-values ?mps ?spawned-wp ?ring-color ?rs-before ?rs-after ?rs-req))
  (plan-action (id 15) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?mps))
  (plan-action (id 16) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps INPUT))
 )
 (modify ?g (mode EXPANDED))
)

(defrule goal-mount-second-ring
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id)
             (class MOUNT-SECOND-RING)
                                             (params robot ?robot
                                                      prev-rs ?prev-rs
                                                      prev-rs-side ?prev-rs-side
                                                      wp ?wp
                                                      rs ?rs
                                                      ring1-color ?ring1-color
                                                      ring2-color ?ring2-color
                                                      rs-before ?rs-before
                                                      rs-after ?rs-after
                                                      rs-req ?rs-req
                                                      order ?order
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
    (assert
      (plan (id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id))
      (plan-action (id 1) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name location-lock)
            (param-values ?prev-rs ?prev-rs-side))
      (plan-action (id 2) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name move)
            (param-names r from from-side to to-side)
            (param-values ?robot ?curr-location ?curr-side ?prev-rs ?prev-rs-side))
      (plan-action (id 3) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name lock)
            (param-values ?prev-rs))
      (plan-action (id 4) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name wp-get)
            (param-names r wp m side)
            (param-values ?robot ?wp ?prev-rs ?prev-rs-side))
      (plan-action (id 5) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name unlock)
            (param-values ?prev-rs))
      (plan-action (id 6) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name location-unlock)
            (param-values ?prev-rs ?prev-rs-side))
      (plan-action (id 7) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name location-lock)
            (param-values ?rs INPUT))
      (plan-action (id 8) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name move)
            (param-names r from from-side to to-side )
            (param-values ?robot ?prev-rs ?prev-rs-side ?rs INPUT))
      (plan-action (id 9) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name lock)
            (param-values ?rs))
      (plan-action (id 10) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name wp-put)
            (param-names r wp m)
            (param-values ?robot ?wp ?rs))
      (plan-action (id 11) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name prepare-rs)
            (param-names m rc rs-before rs-after r-req)
            (param-values ?rs ?ring2-color ?rs-before ?rs-after ?rs-req))
       (plan-action (id 12) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name rs-mount-ring2)
            (param-names m wp col col1 rs-before rs-after r-req)
            (param-values ?rs ?wp ?ring2-color ?ring1-color ?rs-before ?rs-after ?rs-req))
      (plan-action (id 13) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name unlock)
            (param-values ?rs))
      (plan-action (id 14) (plan-id MOUNT-SECOND-RING-PLAN) (goal-id ?goal-id)
            (action-name location-unlock)
            (param-values ?rs INPUT))
     )
    (modify ?g (mode EXPANDED))
)


(defrule goal-mount-third-ring
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id ?goal-id)
             (class MOUNT-THIRD-RING)
                                             (params robot ?robot
                                                      prev-rs ?prev-rs
                                                      prev-rs-side ?prev-rs-side
                                                      wp ?wp
                                                      rs ?rs
                                                      ring1-color ?ring1-color
                                                      ring2-color ?ring2-color
                                                      ring3-color ?ring3-color
                                                      rs-before ?rs-before
                                                      rs-after ?rs-after
                                                      rs-req ?rs-req
                                                      order ?order
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
    (assert
      (plan (id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id))
      (plan-action (id 1) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name location-lock)
            (param-values ?prev-rs ?prev-rs-side))
      (plan-action (id 2) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name move)
            (param-names r from from-side to to-side)
            (param-values ?robot ?curr-location ?curr-side ?prev-rs ?prev-rs-side))
      (plan-action (id 3) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name lock)
            (param-values ?prev-rs))
      (plan-action (id 4) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name wp-get)
            (param-names r wp m side)
            (param-values ?robot ?wp ?prev-rs ?prev-rs-side))
      (plan-action (id 5) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name unlock)
            (param-values ?prev-rs))
      (plan-action (id 6) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name location-unlock)
            (param-values ?prev-rs ?prev-rs-side))
      (plan-action (id 7) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name location-lock)
            (param-values ?rs INPUT))
      (plan-action (id 8) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name move)
            (param-names r from from-side to to-side )
            (param-values ?robot ?prev-rs ?prev-rs-side ?rs INPUT))
      (plan-action (id 9) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name lock)
            (param-values ?rs))
      (plan-action (id 10) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name wp-put)
            (param-names r wp m)
            (param-values ?robot ?wp ?rs))
      (plan-action (id 11) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name prepare-rs)
            (param-names m rc rs-before rs-after r-req)
            (param-values ?rs ?ring3-color ?rs-before ?rs-after ?rs-req))
       (plan-action (id 12) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name rs-mount-ring3)
            (param-names m wp col col1 col2 rs-before rs-after r-req)
            (param-values ?rs ?wp ?ring3-color ?ring1-color ?ring2-color ?rs-before ?rs-after ?rs-req))
      (plan-action (id 13) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name unlock)
            (param-values ?rs))
      (plan-action (id 14) (plan-id MOUNT-THIRD-RING-PLAN) (goal-id ?goal-id)
            (action-name location-unlock)
            (param-values ?rs INPUT))
     )
    (modify ?g (mode EXPANDED))
)


(defrule goal-produce-cx
 ?p <- (goal (mode EXPANDED) (id ?parent))
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
   (assert
    (plan (id PRODUCE-CX-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name location-lock)
          (param-values ?rs OUTPUT))
    (plan-action (id 2) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name move)
          (param-names r from from-side to to-side )
          (param-values ?robot ?curr-location ?curr-side ?rs OUTPUT))
    (plan-action (id 3) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name lock)
          (param-values ?rs))
    (plan-action (id 4) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name wp-get)
          (param-names r wp m side)
          (param-values ?robot ?wp ?rs OUTPUT))
    (plan-action (id 5) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name unlock)
          (param-values ?rs))
    (plan-action (id 6) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name location-unlock)
          (param-values ?rs OUTPUT))
    (plan-action (id 7) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name location-lock)
          (param-values ?mps INPUT))
    (plan-action (id 8) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name move)
          (param-names r from from-side to to-side)
          (param-values ?robot ?rs OUTPUT ?mps INPUT))
    (plan-action (id 9) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name lock)
          (param-values ?mps))
    (plan-action (id 10) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name prepare-cs)
          (param-names m op)
          (param-values ?mps MOUNT_CAP))
    (plan-action (id 11) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name wp-put)
          (param-names r wp m)
          (param-values ?robot ?wp ?mps))
     (plan-action (id 12) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name cs-mount-cap)
          (param-names m wp capcol)
          (param-values ?mps ?wp ?cap-color))
    (plan-action (id 13) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name unlock)
          (param-values ?mps))
    (plan-action (id 14) (plan-id PRODUCE-CX-PLAN) (goal-id ?goal-id)
          (action-name location-unlock)
          (param-values ?mps INPUT))
   )
  (modify ?g (mode EXPANDED))
)



(defrule goal-reset-mps
  ?p <- (goal (mode EXPANDED) (id ?parent))
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
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class DELIVER) (mode SELECTED) (parent ?parent)
             (params robot ?robot
                          mps ?mps
                          order ?order
                          wp ?wp
                          ds ?ds
                          ds-gate ?gate
                          base-color ?base-color
                          cap-color ?cap-color
       ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (assert
  (plan (id DELIVER-PLAN) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?mps OUTPUT))
  (plan-action (id 2) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?curr-location ?curr-side ?mps OUTPUT))
  (plan-action (id 3) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?mps))
  (plan-action (id 4) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?wp ?mps OUTPUT))
  (plan-action (id 5) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?mps))
  (plan-action (id 6) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?mps OUTPUT))
  (plan-action (id 7) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name location-lock)
        (param-values ?ds INPUT))
  (plan-action (id 8) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?mps OUTPUT ?ds INPUT))
  (plan-action (id 9) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name lock) (param-values ?ds))
  (plan-action (id 10) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name prepare-ds)
        (param-names m gate)
        (param-values ?ds ?gate))
  (plan-action (id 11) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?wp ?ds))
  (plan-action (id 12) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name fulfill-order-c0)
        (param-names ord wp m g basecol capcol)
        (param-values ?order ?wp ?ds ?gate ?base-color  ?cap-color))
  (plan-action (id 13) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name unlock) (param-values ?ds))
  (plan-action (id 14) (plan-id DELIVER-PLAN) (goal-id ?goal-id)
        (action-name location-unlock)
        (param-values ?ds INPUT))
 )
 (modify ?g (mode EXPANDED))
)
