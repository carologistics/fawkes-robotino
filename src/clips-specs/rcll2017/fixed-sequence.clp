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
  ?p <- (goal (mode EXPANDED) (id ?parent-id))
  ?g <- (goal (mode SELECTED) (parent ?parent-id) (id BEACONACHIEVE))
=>
  (assert
    (plan (id BEACONPLAN) (goal-id BEACONACHIEVE))
    (plan-action (id 1) (plan-id BEACONPLAN) (goal-id BEACONACHIEVE)
      (action-name send-beacon)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-wp-spawn
  ?p <- (goal (mode EXPANDED) (id ?parent-id))
  ?g <- (goal (mode SELECTED) (parent ?parent-id) (id WPSPAWN-ACHIEVE))
=>
  (assert
    (plan (id SPAWNPLAN) (goal-id WPSPAWN-ACHIEVE))
    (plan-action (id 1) (plan-id SPAWNPLAN) (goal-id WPSPAWN-ACHIEVE)
      (action-name noop)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-refill-shelf
  ?p <- (goal (mode EXPANDED) (id ?parent-id))
  ?g <- (goal (mode SELECTED) (parent ?parent-id) (id REFILL-SHELF-ACHIEVE))
  =>
  (assert
     (plan (id REFILLPLAN) (goal-id REFILL-SHELF-ACHIEVE))
     (plan-action (id 1) (plan-id REFILLPLAN) (goal-id REFILL-SHELF-ACHIEVE)
        (action-name noop)))
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-enter-field
  ?g <- (goal (mode SELECTED) (id ENTER-FIELD-ACHIEVE))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact robot-waiting args? r ?robot))
=>
  (assert
    (plan (id ENTER-FIELD-PLAN) (goal-id ENTER-FIELD-ACHIEVE))
    (plan-action (id 1) (plan-id ENTER-FIELD-PLAN) (goal-id ENTER-FIELD-ACHIEVE)
                                 (action-name enter-field)
                                 (param-names r team-color)
                                 (param-values ?robot ?team-color))
    )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-go-wait
   "Feed a CS with a cap from its shelf so that afterwards
   it can directly put the cap on a product."
    ?p <- (goal (mode EXPANDED) (id ?parent))
    ?g <- (goal (mode SELECTED) (parent ?parent) (id GO-WAIT)
                                                (params r ?robot
                                                  point ?waitpoint
                                                        ))
    (wm-fact (key domain fact self args? r ?robot))
    (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
    =>
   (assert
        (plan (id GO-WAIT-PLAN) (goal-id GO-WAIT))
        (plan-action (id 1) (plan-id GO-WAIT-PLAN) (goal-id GO-WAIT)
                                    (action-name go-wait)
                                    (param-names r to from)
                                    (param-values ?robot ?waitpoint ?curr-location))
    )
    (modify ?g (mode EXPANDED))
)


(defrule goal-expander-prefill-cap-station
   "Feed a CS with a cap from its shelf so that afterwards
   it can directly put the cap on a product."
    ?p <- (goal (mode EXPANDED) (id ?parent))
    ?g <- (goal (mode SELECTED) (parent ?parent) (id FILL-CAP)
                                                (params robot ?robot
                                                        mps ?mps
                                                        cc ?cc
                                                        ))
    (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
    (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?shelf-spot))
    (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
    =>
   (assert
        (plan (id FILL-CAP-PLAN) (goal-id FILL-CAP))
        (plan-action (id 1) (plan-id FILL-CAP-PLAN) (goal-id FILL-CAP)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot ?curr-location ?curr-side ?mps INPUT))
        (plan-action (id 2) (plan-id FILL-CAP-PLAN) (goal-id FILL-CAP)
                                    (action-name wp-get-shelf)
                                    (param-names r cc m spot)
                                    (param-values ?robot ?cc ?mps ?shelf-spot))
        (plan-action (id 3) (plan-id FILL-CAP-PLAN) (goal-id FILL-CAP)
                                    (action-name prepare-cs)
                                    (param-names m op)
                                    (param-values ?mps RETRIEVE_CAP))
        (plan-action (id 4) (plan-id FILL-CAP-PLAN) (goal-id FILL-CAP)
                                    (action-name wp-put)
                                    (param-names r wp m)
                                    (param-values ?robot ?cc ?mps))
        (plan-action (id 5) (plan-id FILL-CAP-PLAN) (goal-id FILL-CAP)
                                    (action-name cs-retrieve-cap)
                                    (param-names m cc capcol)
                                    (param-values ?mps ?cc ?cap-color))

    )
    (modify ?g (mode EXPANDED))
)

(defrule goal-remove-empty-base-from-cs
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id CLEAR-CS)
                                             (params robot ?robot
                                                      mps ?mps
                                                      wp ?wp
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (assert
  (plan (id CLEAR-CS-PLAN) (goal-id CLEAR-CS))
  (plan-action (id 1) (plan-id CLEAR-CS-PLAN) (goal-id CLEAR-CS)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?curr-location ?curr-side ?mps OUTPUT))
  (plan-action (id 2) (plan-id CLEAR-CS-PLAN) (goal-id CLEAR-CS)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?wp ?mps OUTPUT))

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
 ?g <- (goal (mode SELECTED) (parent ?parent) (id DISCARD-UNKNOWN)
                                             (params robot ?robot
                                                    wp ?wp
                                                    ))
  =>
  (assert
    (plan (id DISCARD-UNKNOWN-PLAN) (goal-id DISCARD-UNKNOWN))
    (plan-action (id 1) (plan-id DISCARD-UNKNOWN-PLAN) (goal-id DISCARD-UNKNOWN)
          (action-name wp-discard)
          (param-names r cc )
          (param-values ?robot ?wp))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-fill-rs
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id FILL-RS)
                                             (params robot ?robot
                                                      mps ?mps
                                                      wp ?wp
                                                      rs-before ?rs-before
                                                      rs-after ?rs-after
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (assert
    (plan (id FILL-RS-PLAN) (goal-id FILL-RS))
     (plan-action (id 1) (plan-id FILL-RS-PLAN) (goal-id FILL-RS)
                                    (action-name move)
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot ?curr-location ?curr-side ?mps INPUT))
    (plan-action (id 2) (plan-id FILL-RS-PLAN) (goal-id FILL-RS)
          (action-name wp-put-slide-cc)
          (param-names r wp m rs-before rs-after)
          (param-values ?robot ?wp ?mps ?rs-before ?rs-after))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-produce-c0
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id PRODUCE-C0)
                                             (params robot ?robot
                                                      bs ?bs
                                                      bs-side ?bs-side
                                                      bs-color ?base-color
                                                      mps ?mps
                                                      cs-color ?cap-color
                                                      order ?order
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (bind ?spawned-wp NA)
 (do-for-fact ((?wp-spawned-by wm-fact))
            (and (wm-key-prefix ?wp-spawned-by:key (create$ domain fact wp-spawned-by))
                 (eq (wm-key-arg ?wp-spawned-by:key r) ?robot))
   (bind ?spawned-wp (wm-key-arg ?wp-spawned-by:key wp))
 )

 (if (eq ?spawned-wp NA)
  then
   (printout t "No Spawned WP found for " ?robot " Failing goal" crlf)
   (modify ?g (mode FINISHED) (outcome FAILED)
              (message "Could not expand goal, No spawned WP found!!"))
  else
   (assert
    (plan (id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0))
    (plan-action (id 1) (plan-id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0)
          (action-name move)
          (param-names r from from-side to to-side )
          (param-values ?robot ?curr-location ?curr-side ?bs ?bs-side))
    (plan-action (id 2) (plan-id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0)
          (action-name prepare-bs)
          (param-names m side bc)
          (param-values ?bs ?bs-side ?base-color))
    (plan-action (id 3) (plan-id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0)
          (action-name bs-dispense)
          (param-names r m side wp basecol)
          (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
    (plan-action (id 4) (plan-id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0)
          (action-name wp-get)
          (param-names r wp m side)
          (param-values ?robot ?spawned-wp ?bs ?bs-side))
    (plan-action (id 5) (plan-id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0)
          (action-name move)
          (param-names r from from-side to to-side)
          (param-values ?robot ?bs ?bs-side ?mps INPUT))
    (plan-action (id 6) (plan-id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0)
          (action-name prepare-cs)
          (param-names m op)
          (param-values ?mps MOUNT_CAP))
    (plan-action (id 7) (plan-id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0)
          (action-name wp-put)
          (param-names r wp m)
          (param-values ?robot ?spawned-wp ?mps))
     (plan-action (id 8) (plan-id PRODUCE-C0-PLAN) (goal-id PRODUCE-C0)
          (action-name cs-mount-cap)
          (param-names m wp capcol)
          (param-values ?mps ?spawned-wp ?cap-color ))
   )
  (modify ?g (mode EXPANDED))
  )
)

(defrule goal-mount-first-ring
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id MOUNT-FIRST-RING)
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
                                                      ))
 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 =>
 (bind ?spawned-wp NA)
 (do-for-fact ((?wp-spawned-by wm-fact))
            (and (wm-key-prefix ?wp-spawned-by:key (create$ domain fact wp-spawned-by))
                 (eq (wm-key-arg ?wp-spawned-by:key r) ?robot))
   (bind ?spawned-wp (wm-key-arg ?wp-spawned-by:key wp))
 )

 (if (eq ?spawned-wp NA)
  then
   (printout t "No Spawned WP found for " ?robot " Failing goal" crlf)
   (modify ?g (mode FINISHED) (outcome FAILED)
              (message "Could not expand goal, No spawned WP found!!"))
  else
     (assert
      (plan (id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING))
      (plan-action (id 1) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING)
            (action-name move)
            (param-names r from from-side to to-side )
            (param-values ?robot ?curr-location ?curr-side ?bs ?bs-side))
      (plan-action (id 2) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING)
            (action-name prepare-bs)
            (param-names m side bc)
            (param-values ?bs ?bs-side ?base-color))
      (plan-action (id 3) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING)
            (action-name bs-dispense)
            (param-names r m side wp basecol)
            (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
      (plan-action (id 4) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING)
            (action-name wp-get)
            (param-names r wp m side)
            (param-values ?robot ?spawned-wp ?bs ?bs-side))
      (plan-action (id 5) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING)
            (action-name move)
            (param-names r from from-side to to-side )
            (param-values ?robot ?bs ?bs-side ?mps INPUT))
      (plan-action (id 6) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING)
            (action-name prepare-rs)
            (param-names m rc rs-before rs-after rs-req)
            (param-values ?mps ?ring-color ?rs-before ?rs-after ?rs-req))
      (plan-action (id 7) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING)
            (action-name wp-put)
            (param-names r wp m)
            (param-values ?robot ?spawned-wp ?mps))
       (plan-action (id 8) (plan-id MOUNT-FIRST-RING-PLAN) (goal-id MOUNT-FIRST-RING)
            (action-name rs-mount-ring1)
            (param-names m wp col rs-before rs-after rs-req)
            (param-values ?mps ?spawned-wp ?ring-color ?rs-before ?rs-after ?rs-req))
     )
    (modify ?g (mode EXPANDED))
  )
)

(defrule goal-reset-mps
  ?p <- (goal (mode EXPANDED) (id ?parent))
  ?g <- (goal (mode SELECTED) (parent ?parent) (id RESET-MPS)
                                             (params r ?robot
                                                      m ?mps))
  =>
  (assert
    (plan (id RESET-MPS-PLAN) (goal-id RESET-MPS))
      (plan-action (id 1) (plan-id RESET-MPS-PLAN) (goal-id RESET-MPS)
        (action-name reset-mps)
        (param-names m)
        (param-values ?mps))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-deliver
 ?p <- (goal (mode EXPANDED) (id ?parent))
 ?g <- (goal (mode SELECTED) (parent ?parent) (id DELIVER)
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
  (plan (id DELIVER-PLAN) (goal-id DELIVER))
  (plan-action (id 1) (plan-id DELIVER-PLAN) (goal-id DELIVER)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?curr-location ?curr-side ?mps OUTPUT))
  (plan-action (id 2) (plan-id DELIVER-PLAN) (goal-id DELIVER)
        (action-name wp-get)
        (param-names r wp m side)
        (param-values ?robot ?wp ?mps OUTPUT))
  (plan-action (id 3) (plan-id DELIVER-PLAN) (goal-id DELIVER)
        (action-name move)
        (param-names r from from-side to to-side)
        (param-values ?robot ?mps OUTPUT ?ds INPUT))
  (plan-action (id 4) (plan-id DELIVER-PLAN) (goal-id DELIVER)
        (action-name prepare-ds)
        (param-names m gate)
        (param-values ?ds ?gate))
  (plan-action (id 5) (plan-id DELIVER-PLAN) (goal-id DELIVER)
        (action-name wp-put)
        (param-names r wp m)
        (param-values ?robot ?wp ?ds))
  (plan-action (id 6) (plan-id DELIVER-PLAN) (goal-id DELIVER)
        (action-name fulfill-order-c0)
        (param-names ord wp m g basecol capcol)
        (param-values ?order ?wp ?ds ?gate ?base-color  ?cap-color))
 )
 (modify ?g (mode EXPANDED))
)
