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

(defrule handle-mps-action
  ?g <- (goal (id ?goal-id) (class HANDLE-MPS) (params ?mps) (mode SELECTED))
  ?pre <- (wm-fact (key mps-handling prepare ?prepare-action ?mps args? $?prepare-params))
  ?pro <- (wm-fact (key mps-handling process ?process-action ?mps args? $?process-params))
  =>
  (bind ?prepare-param-values (values-from-name-value-list ?prepare-params))
  (bind ?process-param-values (values-from-name-value-list ?process-params))

  (bind ?plan-id (sym-cat HANDLE-MPS-PLAN-(gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name lock)
      (param-values ?mps)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name ?prepare-action)
      (param-values ?prepare-param-values)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name ?process-action)
      (param-values ?process-param-values)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name unlock)
      (param-values ?mps)
    )
  )
  (retract ?pre ?pro)
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-reset-mps
  ?g <- (goal (id ?goal-id) (class RESET-MPS) (params m ?mps) (mode SELECTED))
  =>
  (bind ?plan-id (sym-cat RESET-MPS-PLAN-(gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name reset-mps)
      (param-values ?mps)
    )
  )
  (modify ?g (mode EXPANDED))
)


; ========================= enter-field plan =============================

(defrule goal-expander-enter-field
"Robot moves from start position into field"
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
              (params r ?robot team-color ?team-color)
        )
=>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  ; robots 2 and 3 perform a short wait to let previous robot clear
  (if (eq ?robot robot1)
  then
    (assert
      (plan (id ENTER-FIELD-PLAN) (goal-id ?goal-id))
      (plan-action (id 1) (plan-id ENTER-FIELD-PLAN) (goal-id ?goal-id)
                                  (action-name enter-field)
                                  (skiller (remote-skiller ?robot))
                                  (param-names r team-color)
                                  (param-values ?robot ?team-color))
    )
  else
    (assert
      (plan (id ENTER-FIELD-PLAN) (goal-id ?goal-id))
      (plan-action (id 1) (plan-id ENTER-FIELD-PLAN) (goal-id ?goal-id)
                                  (action-name wait-at)
                                  (skiller (remote-skiller ?robot))
                                  (param-names r loc side)
                                  (param-values ?robot START INPUT))
      (plan-action (id 2) (plan-id ENTER-FIELD-PLAN) (goal-id ?goal-id)
                                  (action-name enter-field)
                                  (skiller (remote-skiller ?robot))
                                  (param-names r team-color)
                                  (param-values ?robot ?team-color))
    )
  )
  (modify ?g (mode EXPANDED))
)

; ========================= production plans =============================

; ========== base station plans ==========

(defrule goal-expander-get-base
  "Get base from base station and move to the target station. The robot will still
  be holding the base after execution.
  "
  ?g <- (goal (id ?goal-id) (class GET-BASE) (mode SELECTED)
              (params robot ?robot
                      bs ?base-station
                      bs-side ?bs-side
                      bs-color ?base-color
                      target-station ?target-station
                      wp ?wp
              )
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; station is not recovering from a fail
  (not (goal (class CLEAN-BS)))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: base-color " ?base-color crlf)
  (bind ?plan-id (sym-cat GET-BASE-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name go-wait)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to)
      (param-values ?robot ?curr-location ?curr-side (wait-pos ?base-station ?bs-side))
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name location-lock)
      (skiller (remote-skiller ?robot))
      (param-values ?base-station ?bs-side)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name request-bs-dispense)
      (skiller (remote-skiller ?robot))
      (param-names m side wp basecol)
      (param-values ?base-station ?bs-side ?wp ?base-color)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot (wait-pos ?base-station ?bs-side) WAIT ?base-station ?bs-side)
    )
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get)
      (skiller (remote-skiller ?robot))
      (param-names r wp m side)
      (param-values ?robot ?wp ?base-station ?bs-side)
    )
    (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name location-unlock)
      (skiller (remote-skiller ?robot))
      (param-values ?base-station ?bs-side)    
    )
    (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name go-wait)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to)
      (param-values ?robot ?base-station ?bs-side (wait-pos ?target-station INPUT))
    )
  )
  (modify ?g (mode EXPANDED))
)

; ========== ring station plans ==========

(defrule goal-expander-fill-rs
  "Fill a ring station with one additional base"
  ?g <- (goal (id ?goal-id) (class FILL-RS) (mode SELECTED)
              (params robot ?robot rs ?ring-station wp ?wp)
        )

  ; workaround to fix parallel issue with rs-filled-with
  (not (goal (class FILL-RS) (mode EXPANDED|COMMITTED|DISPATCHED|FINISHED)
             (params $? rs ?ring-station $?)))
  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; get rs status
  (wm-fact (key domain fact rs-filled-with args? m ?ring-station n ?rs-before))
  (wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: ring-station " ?ring-station ", current fill " ?rs-before crlf)
  (bind ?plan-id (sym-cat FILL-RS-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name location-lock)
      (skiller (remote-skiller ?robot))
      (param-values ?ring-station INPUT)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?curr-location ?curr-side ?ring-station INPUT)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-put-slide-cc)
      (skiller (remote-skiller ?robot))
      (param-names r wp m rs-before rs-after)
      (param-values ?robot ?wp ?ring-station ?rs-before ?rs-after)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name location-unlock)
      (skiller (remote-skiller ?robot))
      (param-values ?ring-station INPUT)
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-mount-ring
  "Mount certain ring on workpiece."
  ?g <- (goal (id ?goal-id) (class MOUNT-RING) (mode SELECTED)
              (params robot ?robot
                      rs ?ring-station
                      ring-mount-index ?ring-mount-index
                      ring-color ?ring-color
                      ring-base-req ?ring-base-req
                      wp ?wp
              )
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; get current wp ring colors
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?current-ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?current-ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?current-ring3-color))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: ring-station " ?ring-station ", index " ?ring-mount-index 
              ", color " ?ring-color crlf)
  (bind ?plan-id (sym-cat MOUNT-RING-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side )
      (param-values ?robot ?curr-location ?curr-side ?ring-station INPUT)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-put)
      (skiller (remote-skiller ?robot))
      (param-names r wp m)
      (param-values ?robot ?wp ?ring-station)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name request-rs-mount-ring)
      (skiller (remote-skiller ?robot))
      (param-values ?robot ?ring-station ?wp ?ring-mount-index ?ring-color
                      ?current-ring1-color ?current-ring2-color ?current-ring3-color
                      ?ring-base-req)
    )
  )
  (modify ?g (mode EXPANDED))
)

; ========== cap station plans ==========

(defrule goal-expander-fill-cs
  "Retrieve cap at cap station and discard the unneeded base."
  ?g <- (goal (id ?goal-id) (class FILL-CS) (mode SELECTED)
              (params robot ?robot mps ?cap-station)
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; get cc location
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?shelf-spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  ; station not already buffered
  (not (wm-fact (key domain fact cs-buffered args? m ?cap-station col ?cap-color)))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: cap-station " ?cap-station crlf)
  (bind ?plan-id (sym-cat FILL-CS-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name go-wait)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to)
      (param-values ?robot ?curr-location ?curr-side (wait-pos ?cap-station INPUT))
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot (wait-pos ?cap-station INPUT) WAIT ?cap-station INPUT)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get-shelf)
      (skiller (remote-skiller ?robot))
      (param-names r cc m spot)
      (param-values ?robot ?cc ?cap-station ?shelf-spot)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-put)
      (skiller (remote-skiller ?robot))
      (param-names r wp m)
      (param-values ?robot ?cc ?cap-station)
    )
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name request-cs-retrieve-cap)
      (param-values ?robot ?cap-station ?cc ?cap-color)
    )
    (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?cap-station INPUT ?cap-station OUTPUT)
    )
    (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get)
      (skiller (remote-skiller ?robot))
      (param-names r wp m side)
      (param-values ?robot ?cc ?cap-station OUTPUT)
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-buffer-cs
  "Retrieve a cap at a cap station without discarding cc."
  ?g <- (goal (id ?goal-id) (class BUFFER-CS) (mode SELECTED)
              (params robot ?robot mps ?cap-station)
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; get cc location
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?shelf-spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  ; station not already buffered
  (not (wm-fact (key domain fact cs-buffered args? m ?cap-station col ?cap-color)))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: cap-station " ?cap-station crlf)
  (bind ?plan-id (sym-cat BUFFER-CS-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name go-wait)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to)
      (param-values ?robot ?curr-location ?curr-side (wait-pos ?cap-station INPUT))
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot (wait-pos ?cap-station INPUT) WAIT ?cap-station INPUT)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get-shelf)
      (skiller (remote-skiller ?robot))
      (param-names r cc m spot)
      (param-values ?robot ?cc ?cap-station ?shelf-spot)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-put)
      (skiller (remote-skiller ?robot))
      (param-names r wp m)
      (param-values ?robot ?cc ?cap-station)
    )
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name request-cs-retrieve-cap)
      (param-values ?robot ?cap-station ?cc ?cap-color)
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-cs-fast-forward
  "Cap station already buffered, FILL-CS / BUFFER-CS can be fast forwarded"
  ?g <- (goal (id ?goal-id) (class FILL-CS|BUFFER-CS) (mode SELECTED)
              (params robot ?robot mps ?cap-station)
        )

  ; check if cap-station is buffered            
  (wm-fact (key domain fact cs-buffered args? m ?cap-station col ?cap-color))
  =>
  (printout t "Fast-forwarding " ?goal-id " for robot " ?robot crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule goal-expander-mount-cap
  "Mount prepared cap on workpiece."
  ?g <- (goal (id ?goal-id) (class MOUNT-CAP) (mode SELECTED)
              (params robot ?robot cs ?cap-station cap-color ?cap-color wp ?wp)
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: cap-station " ?cap-station crlf)
  (bind ?plan-id (sym-cat MOUNT-CAP-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?curr-location ?curr-side ?cap-station INPUT)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-put)
      (skiller (remote-skiller ?robot))
      (param-names r wp m)
      (param-values ?robot ?wp ?cap-station)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name request-cs-mount-cap)
      (skiller (remote-skiller ?robot))
      (param-names r mps wp capcol)
      (param-values ?robot ?cap-station ?wp ?cap-color)
    )
  )
  (modify ?g (mode EXPANDED))
)

; ========== delivery station plans ==========

(defrule goal-expander-deliver
  "Deliver the finished product"
  ?g <- (goal (id ?goal-id) (class DELIVER) (mode SELECTED)
              (params robot ?robot
                      order ?order
                      ds ?ds
                      wp ?wp
              )
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ;wp facts
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color-wp))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color-wp))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color-wp))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color-wp))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color-wp))
  ;Order-CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))

  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: order " ?order crlf)
  (bind ?plan-id (sym-cat DELIVER-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name go-wait)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to)
      (param-values ?robot ?curr-location ?curr-side (wait-pos ?ds INPUT))
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wait-until-delivery)
      (skiller (remote-skiller ?robot))
      (param-values ?robot ?order)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name location-lock)
      (skiller (remote-skiller ?robot))
      (param-values ?ds INPUT)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot (wait-pos ?ds INPUT) WAIT ?ds INPUT)
    )
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-put)
      (skiller (remote-skiller ?robot))
      (param-names r wp m)
      (param-values ?robot ?wp ?ds)
    )
    (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name request-ds-fulfill-order)
      (param-values ?robot ?ds ?wp ?order)
    )
    (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name location-unlock)
      (skiller (remote-skiller ?robot))
      (param-values ?ds INPUT)
    )
  )
  (modify ?g (mode EXPANDED))
)


; ========================= pickup/clear logic plans =============================

(defrule goal-expander-pickup-wp
  "Pickup a wp at some output"
  ?g <- (goal (id ?goal-id) (class PICKUP-WP) (mode SELECTED)
              (params robot ?robot wp ?wp)
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; get wp location
  (wm-fact (key domain fact wp-at args? wp ?wp m ?destination side ?destination-side&:(eq ?destination-side OUTPUT)))
  ; only either pickup-wp or clear-output should be defined for the same station/wp
  (not (goal (class CLEAR-OUTPUT) (params $? mps ?destination mps-side ?destination-side $?) (mode EXPANDED|COMMITTED|DISPATCHED)))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: wp " ?wp " wp at " ?destination " " ?destination-side crlf)
  (bind ?plan-id (sym-cat PICKUP-WP-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?curr-location ?curr-side ?destination ?destination-side)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get)
      (skiller (remote-skiller ?robot))
      (param-names r wp m side)
      (param-values ?robot ?wp ?destination ?destination-side)
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-pickup-wp-fast-forward
  "Immediatley finish a pickup-wp goal if a robot is already holding the wp"
  ?g <- (goal (id ?goal-id) (class PICKUP-WP) (mode SELECTED)
              (params $?p1 wp ?wp $?p2)
        )

  ; some robot is holding the wp
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
  =>
  (printout t "Fast-forwarding " ?goal-id crlf)
  (printout t "Fast-forward Details: held by robot " ?robot crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)

(defrule goal-expander-clear-output
  "Clear the output of some station"
  ?g <- (goal (id ?goal-id) (class CLEAR-OUTPUT) (mode SELECTED)
              (params robot ?robot mps ?destination mps-side ?destination-side)
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; get wp at target station
  (wm-fact (key domain fact wp-at args? wp ?wp m ?destination side ?destination-side))
  ; only either pickup-wp or clear-output should be defined for the same station/wp
  (not (goal (class PICKUP-WP) (params $? wp ?wp $?) (mode EXPANDED|COMMITTED|DISPATCHED)))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: target station " ?destination " " ?destination-side 
              ", wp " ?wp crlf)
  (bind ?plan-id (sym-cat CLEAR-OUTPUT-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?curr-location ?curr-side ?destination ?destination-side)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get)
      (skiller (remote-skiller ?robot))
      (param-names r wp m side)
      (param-values ?robot ?wp ?destination ?destination-side)
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-clear-output-fast-forward
  "Immediatley complete a clear-output goal if there is no workpiece at the output"
  ?g <- (goal (id ?goal-id) (class CLEAR-OUTPUT) (mode SELECTED)
              (params $?p1 mps ?mps mps-side ?mps-side $?p2)
        )

  ; nothing to pickup at target station           
  (not (wm-fact (key domain fact wp-at args? wp ? m ?mps side ?mps-side)))
  ; machine not still processing some action
  (wm-fact (key domain fact mps-state args? m ?mps s ~BROKEN&~PROCESSING&~DOWN))
  (not (wm-fact (key mps-handling prepare prepare-cs ?mps args? m ?mps op ? )))
  (not (wm-fact (key mps-handling prepare prepare-rs ?mps args? m ?mps rc ? rs-before ? rs-after ? r-req ? )))
  (not (goal (class HANDLE-MPS) (params ?mps)))
  =>
  (printout t "Fast-forwarding " ?goal-id crlf)
  (printout t "Fast-forward Details: nothing to clear at " ?mps " " ?mps-side crlf)
  (modify ?g (mode FINISHED) (outcome COMPLETED))
)


; ========================= utility plans =============================

; ========== wp handling plans ==========

(defrule goal-expander-drop-wp
  "Robot drops whatever workpiece he is currently holding."
  ?g <- (goal (id ?goal-id) (class DROP-WP) (mode SELECTED)
              (params robot ?robot wp ?wp)
        )
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: dropped wp " ?wp crlf)
  (bind ?plan-id (sym-cat DROP-WP-PLAN-(gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (skiller (remote-skiller ?robot))
      (action-name wp-discard)
      (param-values ?robot ?wp)
    )
  )
  (modify ?g (mode EXPANDED))
)

; ========== positioning plans ==========

(defrule goal-expander-clear-station
  "Remove robot from station by moving to the waiting position"
  ?g <- (goal (id ?goal-id) (class CLEAR-STATION) (mode SELECTED)
              (params robot ?robot)
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: location " ?curr-location " " ?curr-side crlf)
  (bind ?plan-id (sym-cat CLEAR-STATION-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name go-wait)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to)
      (param-values ?robot ?curr-location ?curr-side (wait-pos ?curr-location ?curr-side))
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-go-wait
  "Expand goal to call a simple go-wait action."
  ?g <- (goal (id ?goal-id) (class GO-WAIT) (mode SELECTED) 
              (params robot ?robot mps ?mps mps-side ?mps-side)
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: destination " ?mps " " ?mps-side crlf)
  (bind ?plan-id (sym-cat GO-WAIT-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name go-wait)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to)
      (param-values ?robot ?curr-location ?curr-side (wait-pos ?mps ?mps-side))
    )
  )
  (modify ?g (mode EXPANDED))
)

; ========== storage plans ==========

;TODO: currently not used due to SS being unusable
(defrule goal-expander-store-wp
  "Expand goal to store a wp at some station"
  ?g <- (goal (id ?goal-id) (class STORE-WP) (mode SELECTED) 
              (params robot ?robot wp ?wp mps ?mps mps-side ?mps-side)
        )

  ; get current robot location
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (printout t "Expanding " ?goal-id " for robot " ?robot crlf)
  (printout t "Expansion Details: wp " ?wp ", destination " ?mps " "  ?mps-side crlf)
  (bind ?plan-id (sym-cat STORE-WP-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?curr-location ?curr-side ?mps ?mps-side)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-put)
      (skiller (remote-skiller ?robot))
      (param-names r wp m)
      (param-values ?robot ?wp ?mps)
    )
  )
  (modify ?g (mode EXPANDED))
)