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


(defrule goal-expander-prepare-cap
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class PREPARE-CAP) (params cc ?cc))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (meta r ?robot)))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?shelf-spot))
=>
  (printout t "Expanding PREPARE-CAP with " ?robot " " ?cap-color crlf)
  (bind ?plan-id (sym-cat PREPARE-CAP-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?mps INPUT))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get-shelf)
              (skiller (remote-skiller ?robot))
              (param-names r cc m spot)
              (param-values ?robot ?cc ?mps ?shelf-spot))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?cc ?mps))
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-cs)
              (skiller (remote-skiller ?robot))
              (param-names m op)
              (param-values ?mps RETRIEVE_CAP))
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name cs-retrieve-cap)
              (skiller (remote-skiller ?robot))
              (param-values ?mps ?cc ?cap-color))
  )
  (modify ?g (mode EXPANDED) (meta r ?robot))
)

(defrule goal-expander-discard-base
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class DISCARD-BASE) (params cc ?cc))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (meta r ?robot)))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  (wm-fact (key domain fact wp-at args? wp ?cc m ?mps side OUTPUT))
=>
  (printout t "Expanding DISCARD-BASE with " ?robot " " ?cc " " ?mps crlf)
  (bind ?plan-id (sym-cat DISCARD-BASE-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?mps OUTPUT))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?cc ?mps OUTPUT))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-discard)
              (skiller (remote-skiller ?robot))
              (param-names r cc )
              (param-values ?robot ?cc))
  )
  (modify ?g (mode EXPANDED) (meta r ?robot))
)

(defrule goal-expander-fetch-base
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode SELECTED) (class FETCH-BASE) (params base-color ?base-color cap-color ?cap-color spawned-wp ?spawned-wp mps ?mps))
  (not (goal (parent ?parent-id) (class DISCARD-BASE)))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (meta r ?robot)))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
=>
  (printout t "Expanding FETCH-BASE with " ?robot " " ?base-color " " ?mps crlf)
  (bind ?plan-id (sym-cat FETCH-BASE-PLAN- (gensym*)))
  (bind ?bs C-BS)
  (bind ?bs-side INPUT)
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side )
              (param-values ?robot ?curr-location ?curr-side ?bs ?bs-side))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-bs)
              (skiller (remote-skiller ?robot))
              (param-names m side bc)
              (param-values ?bs ?bs-side ?base-color))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name spawn-wp)
              (skiller (remote-skiller ?robot))
              (param-values ?spawned-wp ?robot))
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name bs-dispense)
              (skiller (remote-skiller ?robot))
              (param-names r m side wp basecol)
              (param-values ?robot ?bs ?bs-side ?spawned-wp ?base-color))
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?spawned-wp ?bs ?bs-side))
    ; wait?
    (plan-action (id 6) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?bs ?bs-side ?mps INPUT))
    (plan-action (id 7) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?spawned-wp ?mps))
    (plan-action (id 8) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-cs)
              (skiller (remote-skiller ?robot))
              (param-names m op)
              (param-values ?mps MOUNT_CAP))
    (plan-action (id 9) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name cs-mount-cap)
              (skiller (remote-skiller ?robot))
              (param-values ?mps ?spawned-wp ?cap-color))
  )
  (modify ?g (mode EXPANDED) (meta r ?robot))
)


(defrule goal-expander-deliver
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class DELIVER) (params wp ?wp m ?mps side ?mps-side))
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (goal (meta r ?robot)))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side ?mps-side))
=>
  (printout t "Expanding DELIVER with " ?robot " " ?wp " " ?mps crlf)
  (bind ?plan-id (sym-cat DELIVER-PLAN- (gensym*)))
  (bind ?ds C-DS)
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?mps ?mps-side))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?mps ?mps-side))
    ; wait?
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?mps ?mps-side ?ds INPUT))
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?wp ?ds))
    ; (plan-action (id 28) (plan-id ?plan-id) (goal-id ?goal-id)
    ;           (action-name ds-fulfill-order)
    ;           (param-names m wp ord)
    ;           (param-values ?ds ?wp ?order))
  )
  (modify ?g (mode EXPANDED) (meta r ?robot))
)
