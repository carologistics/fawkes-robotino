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

; ========================= enter-field plan =============================

(defrule goal-expander-enter-field
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
              (params r ?robot team-color ?team-color))
=>
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

(defrule goal-expander-fill-cs
  "Retrieve cap at cap station and discard the unneeded base."
  ?g <- (goal (id ?goal-id) (class FILL-CS) (mode SELECTED)
              (params robot ?robot mps ?cap-station cc ?cc))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))

  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cap-station spot ?shelf-spot))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  =>
  (bind ?plan-id (sym-cat FILL-CS-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?curr-location ?curr-side ?cap-station INPUT)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get-shelf)
      (skiller (remote-skiller ?robot))
      (param-names r cc m spot)
      (param-values ?robot ?cc ?cap-station ?shelf-spot)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-put)
      (skiller (remote-skiller ?robot))
      (param-names r wp m)
      (param-values ?robot ?cc ?cap-station)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name request-cs-retrieve-cap)
      (param-values ?robot ?cap-station ?cc ?cap-color)
    )
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?cap-station INPUT ?cap-station OUTPUT)
    )
    (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get)
      (skiller (remote-skiller ?robot))
      (param-names r wp m side)
      (param-values ?robot ?cc ?cap-station OUTPUT)
    )
    (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-discard)
      (skiller (remote-skiller ?robot))
      (param-names r cc)
      (param-values ?robot ?cc)
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-get-base
  "Get base from base station and bring it to the cap station. The robot will
  be holding the base after execution."
  ?g <- (goal (id ?goal-id) (class GET-BASE) (mode SELECTED)
              (params robot ?robot
                      bs ?base-station
                      bs-side ?bs-side
                      bs-color ?base-color
                      cs ?cap-station
                      wp ?wp)
        )
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (bind ?plan-id (sym-cat GET-BASE-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?curr-location ?curr-side ?base-station ?bs-side)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name lock)
      (skiller (remote-skiller ?robot))
      (param-values ?base-station)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name prepare-bs)
      (skiller (remote-skiller ?robot))
      (param-names m side bc)
      (param-values ?base-station ?bs-side ?base-color)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name bs-dispense)
      (skiller (remote-skiller ?robot))
      (param-names m side wp basecol)
      (param-values ?base-station ?bs-side ?wp ?base-color)
    )
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get)
      (skiller (remote-skiller ?robot))
      (param-names r wp m side)
      (param-values ?robot ?wp ?base-station ?bs-side)
    )
    (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name unlock)
      (skiller (remote-skiller ?robot))
      (param-values ?base-station)    
    )
    (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name go-wait)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to)
      (param-values ?robot ?base-station ?bs-side (wait-pos ?cap-station INPUT))
    )
  )
  (modify ?g (mode EXPANDED))        
)

(defrule goal-expander-mount-cap
  "Mount prepared cap on base."
  ?g <- (goal (id ?goal-id) (class MOUNT-CAP) (mode SELECTED)
              (params robot ?robot cs ?cap-station cap-color ?cap-color wp ?wp)
        )
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
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
      (action-name lock)
      (skiller (remote-skiller ?robot))
      (param-values ?cap-station)
    )
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name prepare-cs)
      (skiller (remote-skiller ?robot))
      (param-names m op)
      (param-values ?cap-station MOUNT_CAP)
    )
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name cs-mount-cap)
      (skiller (remote-skiller ?robot))
      (param-names m wp capcol)
      (param-values ?cap-station ?wp ?cap-color)
    )
    (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name unlock)
      (skiller (remote-skiller ?robot))
      (param-values ?cap-station)
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-deliver-c0
  "Pick up the finished c0 product and deliver it"
  ?g <- (goal (id ?goal-id) (class DELIVER-C0) (mode SELECTED)
              (params robot ?robot 
                    order ?order
                    ds ?ds
                    cs ?cap-station
                    wp ?wp 
                    base-color ?base-color 
                    cap-color ?cap-color))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
  (bind ?plan-id (sym-cat DELIVER-C0-PLAN- ?robot - (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?curr-location ?curr-side ?cap-station OUTPUT)
    )
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name wp-get)
      (skiller (remote-skiller ?robot))
      (param-names r wp m side)
      (param-values ?robot ?wp ?cap-station OUTPUT)
    )
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
      (action-name move)
      (skiller (remote-skiller ?robot))
      (param-names r from from-side to to-side)
      (param-values ?robot ?cap-station OUTPUT ?ds INPUT)
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
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-clear-station
  "Remove robot from station by moving to the waiting position"
  ?g <- (goal (id ?goal-id) (class CLEAR-STATION) (params robot ?robot) (mode SELECTED))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  =>
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
