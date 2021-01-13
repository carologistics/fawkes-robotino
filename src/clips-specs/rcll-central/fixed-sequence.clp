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

; ========================= administrative plans =============================

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

; ========================= process mps =============================

(defrule goal-expander-process-mps
 ?p <- (goal (mode DISPATCHED) (id ?parent))
 ?g <- (goal (id ?goal-id) (class PROCESS-MPS) (mode SELECTED) (parent ?parent)
             (params m ?mps $?other-goal-args))
  ?pre <- (wm-fact (key mps-handling prepare ?prepare-action ?mps args? $?prepare-params))
  ?pro <- (wm-fact (key mps-handling process ?process-action ?mps args? $?process-params))
  ; The DS might have multiple pending mps-handling facts, they can be
  ; distinguished by additional params (wp and order id) that are present
  ; both in the goal and the mps-handling facts
  ;(test (and (member$ ?other-goal-args ?prepare-params)
  ;           (member$ ?other-goal-args ?process-params)))
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
    (printout t "Process mps " ?process-action "with " ?process-param-values crlf)
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

; ========================= enter-field plan =============================

(defrule goal-expander-enter-field
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
              (params r ?robot team-color ?team-color))
=>
  (assert
    (plan (id ENTER-FIELD-PLAN) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ENTER-FIELD-PLAN) (goal-id ?goal-id)
                                 (action-name enter-field)
                                 (skiller (remote-skiller ?robot))
                                 (param-names r team-color)
                                 (param-values ?robot ?team-color))
    )
  (modify ?g (mode EXPANDED))
)

; ========================= visit-station plan =============================

(defrule goal-expander-visit-station
  "Move robot to station"
   ?g <- (goal (id ?goal-id) (class VISIT-STATION) (mode SELECTED)
               (params r ?robot station ?station side ?side
         ))
   (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
   =>
   (printout t "Expanding " ?goal-id crlf)
   ; give robot-dependent plan-ids to avoid multiple plans with the same name
   ; at the same time. (causes problems with execution-monitoring)
   (bind ?plan-id (sym-cat VISIT-STATION-PLAN- ?robot - (gensym*)))
   (bind ?destination (str-cat ?station (if (eq ?side INPUT) then -I else -O)))
   (assert
        (plan (id ?plan-id ) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?plan-id ) (goal-id ?goal-id)
                     (action-name go-wait)
                     (skiller (remote-skiller ?robot))
                     (param-names r from from-side to)
                     (param-values ?robot ?curr-location ?curr-side ?destination))
   )
   (modify ?g (mode EXPANDED))
)

; ========================= production plans =============================

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
    (bind ?plan-id (sym-cat FILL-CAP-PLAN- ?robot - (gensym*)))
    (assert
        (plan (id ?plan-id) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (skiller (remote-skiller ?robot))
                                    (param-names r from from-side to)
                                    (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps INPUT)))
        (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name location-lock)
                                    (skiller (remote-skiller ?robot))
                                    (param-values ?mps INPUT))
        (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name move)
                                    (skiller (remote-skiller ?robot))
                                    (param-names r from from-side to to-side)
                                    (param-values ?robot (wait-pos ?mps INPUT) WAIT ?mps INPUT))
        (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name wp-get-shelf)
                                    (skiller (remote-skiller ?robot))
                                    (param-names r cc m spot)
                                    (param-values ?robot ?cc ?mps ?shelf-spot))
        (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name wp-put)
                                    (skiller (remote-skiller ?robot))
                                    (param-names r wp m)
                                    (param-values ?robot ?cc ?mps))
        (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name request-cs-retrieve-cap)
                                    (skiller (remote-skiller ?robot))
                                    (param-values ?robot ?mps ?cc ?cap-color))
        (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name location-unlock)
                                    (skiller (remote-skiller ?robot))
                                    (param-values ?mps INPUT))
        (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
                                    (action-name go-wait)
                                    (skiller (remote-skiller ?robot))
                                    (param-names r from from-side to)
                                    (param-values ?robot ?mps INPUT (wait-pos ?mps INPUT)))
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
  (bind ?plan-id (sym-cat PRODUCE-C0-PLAN- ?robot - (gensym*)))
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?spawned-wp))))
    then
      (bind ?offset 10)
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?bs ?bs-side)))
        (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-lock)
              (skiller (remote-skiller ?robot))
              (param-values ?bs ?bs-side))
        (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side )
              (param-values ?robot (wait-pos ?bs ?bs-side) WAIT ?bs ?bs-side))
        (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name lock) 
              (skiller (remote-skiller ?robot)) 
              (param-values ?bs))
        (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-bs)
              (skiller (remote-skiller ?robot))
              (param-names m side bc)
              (param-values ?bs ?bs-side ?base-color))
        (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name bs-dispense)
              (skiller (remote-skiller ?robot))
              (param-names r m side wp basecol)
              (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
        (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?spawned-wp ?bs ?bs-side))
        (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name unlock) 
              (skiller (remote-skiller ?robot)) 
              (param-values ?bs))
        (plan-action (id 9) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-unlock)
              (skiller (remote-skiller ?robot))
              (param-values ?bs ?bs-side))
        (plan-action (id 10) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?bs ?bs-side (wait-pos ?mps INPUT))))
    else
      (bind ?offset 1)
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps INPUT))))
  )
  (assert
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id (+ ?offset 1)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-lock)
        (skiller (remote-skiller ?robot))
        (param-values ?mps INPUT))
  (plan-action (id (+ ?offset 2)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name move)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to to-side)
        (param-values ?robot (wait-pos ?mps INPUT) WAIT ?mps INPUT))
  (plan-action (id (+ ?offset 3)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name wp-put)
        (skiller (remote-skiller ?robot))
        (param-names r wp m)
        (param-values ?robot ?spawned-wp ?mps))
  (plan-action (id (+ ?offset 4)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name request-cs-mount-cap)
        (skiller (remote-skiller ?robot))
        (param-values ?robot ?mps ?spawned-wp ?cap-color))
  (plan-action (id (+ ?offset 5)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-unlock)
        (skiller (remote-skiller ?robot))
        (param-values ?mps INPUT))
  (plan-action (id (+ ?offset 6)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name go-wait)
        (skiller (remote-skiller ?robot))
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
 (bind ?plan-id (sym-cat CLEAR-MPS-PLAN- ?robot - (gensym*)))
 (assert  
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name go-wait)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to)
        (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps ?side)))
  (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-lock)
        (skiller (remote-skiller ?robot))
        (param-values ?mps ?side))
  (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name move)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to to-side)
        (param-values ?robot (wait-pos ?mps ?side) WAIT ?mps ?side))
  (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name wp-get)
        (skiller (remote-skiller ?robot))
        (param-names r wp m side)
        (param-values ?robot ?wp ?mps ?side))
  (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
                              (action-name location-unlock)
                              (skiller (remote-skiller ?robot))
                              (param-values ?mps ?side))
  (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name go-wait)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to)
        (param-values ?robot ?mps ?side (wait-pos ?mps ?side)))
  (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name wp-discard)
        (skiller (remote-skiller ?robot))
        (param-names r cc)
        (param-values ?robot ?wp))
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
 (wm-fact (key order meta wp-for-order args? wp ?wp ord ?ord))
 (wm-fact (key domain fact order-complexity args? ord ?ord com ?complexity))
 =>
  (bind ?plan-id (sym-cat DELIVER-PLAN- ?robot - (gensym*)))
  (if (not (any-factp ((?holding wm-fact))
                 (and (wm-key-prefix ?holding:key (create$ domain fact holding))
                      (eq (wm-key-arg ?holding:key r) ?robot)
                      (eq (wm-key-arg ?holding:key wp) ?wp))))
    then
      (bind ?offset 8)
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps OUTPUT)))
        (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-lock)
              (skiller (remote-skiller ?robot))
              (param-values ?mps OUTPUT))
        (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot (wait-pos ?mps OUTPUT) WAIT ?mps OUTPUT))
        (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name lock)
              (skiller (remote-skiller ?robot)) (param-values ?mps))
        (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?mps OUTPUT))
        (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name unlock)
              (skiller (remote-skiller ?robot)) (param-values ?mps))
        (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name location-unlock)
              (skiller (remote-skiller ?robot))
              (param-values ?mps OUTPUT))
        (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?mps OUTPUT (wait-pos ?ds INPUT))))
    else
      (bind ?offset 1)
      (assert
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?curr-location ?curr-side (wait-pos ?ds INPUT))))
  )
 (assert
  (plan (id ?plan-id) (goal-id ?goal-id))
  (plan-action (id (+ ?offset 1)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-lock)
        (skiller (remote-skiller ?robot))
        (param-values ?ds INPUT))
  (plan-action (id (+ ?offset 2)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name move)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to to-side)
        (param-values ?robot (wait-pos ?ds INPUT) WAIT ?ds INPUT))
  (plan-action (id (+ ?offset 3)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name lock)
        (skiller (remote-skiller ?robot)) (param-values ?ds))
  (plan-action (id (+ ?offset 4)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name wp-put)
        (skiller (remote-skiller ?robot))
        (param-names r wp m)
        (param-values ?robot ?wp ?ds))
  (plan-action (id (+ ?offset 5)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name request-ds-fulfill-order)
        (skiller (remote-skiller ?robot))
        (param-names r m wp ord)
        (param-values ?robot ?ds ?wp ?order))
)

 (assert
   (plan-action (id (+ ?offset 7)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name unlock)
        (skiller (remote-skiller ?robot)) (param-values ?ds))
   (plan-action (id (+ ?offset 8)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name location-unlock)
        (skiller (remote-skiller ?robot))
        (param-values ?ds INPUT))
   (plan-action (id (+ ?offset 9)) (plan-id ?plan-id) (goal-id ?goal-id)
        (action-name go-wait)
        (skiller (remote-skiller ?robot))
        (param-names r from from-side to)
        (param-values ?robot ?ds INPUT (wait-pos ?ds INPUT)))
 )
 (modify ?g (mode EXPANDED))
)

; ========================= no-progress plans =============================
(defrule goal-expander-wait
  ?p <- (goal (mode DISPATCHED) (id ?parent))
  ?g <- (goal (id ?goal-id) (class WAIT) (parent ?parent) (mode SELECTED)
              (params r ?robot
                      point ?waitpoint))
  =>
  (bind ?plan-id (sym-cat WAIT-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name wait)
                 (skiller (remote-skiller ?robot))
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
   (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
   =>
   (bind ?plan-id (sym-cat GO-WAIT-PLAN- ?robot - (gensym*)))
   (assert
        (plan (id ?plan-id) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                     (action-name go-wait)
                     (skiller (remote-skiller ?robot))
                     (param-names r from from-side to)
                     (param-values ?robot ?curr-location ?curr-side ?waitpoint))
   )
   (modify ?g (mode EXPANDED))
)
