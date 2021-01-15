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
(bind ?planid (sym-cat ENTER-FIELD-PLAN- (gensym*)))
  (assert
    (plan (id ?planid) (goal-id ?goal-id))
    (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
                                 (action-name enter-field)
                                 (skiller (remote-skiller ?robot))
                                 (param-names r team-color)
                                 (param-values ?robot ?team-color))
    )
  (modify ?g (mode EXPANDED))
)

(defrule goal-produce-c0
 ?g <- (goal (id ?goal-id) (class PRODUCE-C0) (mode SELECTED) (params bs-color ?bs-color cs-color ?cs-color))

 (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
 (wm-fact (key domain fact mps-type args? m ?mps t CS))
 (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
 ;(wm-fact (key domain fact cs-can-perform args? m ?mps op MOUNT_CAP))

 (wm-fact (key domain fact mps-type args? m ?bs t BS))
 (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  
 (wm-fact (key refbox team-color) (value ?team-color))
 (wm-fact (key domain fact entered-field args? r ?robot))

 (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?shelf-spot))
 (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))

 (wm-fact (key domain fact can-hold args? r ?robot))

 =>
      (bind ?spawned-wp (sym-cat WP- (random-id)))
      (printout t "plan action series here " PRODUCE-C0 crlf)
      (bind ?planid (sym-cat PRODUCE-C0-PLAN- (gensym*)))
      (assert
        (plan (id ?planid) (goal-id ?goal-id))
        (plan-action (id 1) (plan-id ?planid) (goal-id ?goal-id)
              (action-name spawn-wp)
              (param-values ?spawned-wp ?robot))
        (plan-action (id 2) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?mps INPUT))
        (plan-action (id 3) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get-shelf)
              (skiller (remote-skiller ?robot))
              (param-names r cc m spot)
              (param-values ?robot ?cc ?mps ?shelf-spot))
        (plan-action (id 4) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?cc ?mps))
        (plan-action (id 5) (plan-id ?planid) (goal-id ?goal-id)
              (action-name request-cs-retrieve-cap)
              (skiller (remote-skiller ?robot))
              (param-values ?robot ?mps ?cc ?cap-color))
        (plan-action (id 6) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?cc ?mps INPUT))
        (plan-action (id 7) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-discard)
              (skiller (remote-skiller ?robot))
              (param-names r cc )
              (param-values ?robot ?cc))
        (plan-action (id 8) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side )
              (param-values ?robot ?mps INPUT ?bs INPUT))
        (plan-action (id 9) (plan-id ?planid) (goal-id ?goal-id)
              (action-name prepare-bs)
              (skiller (remote-skiller ?robot))
              (param-names m side bc)
              (param-values ?bs INPUT ?bs-color))
        (plan-action (id 10) (plan-id ?planid) (goal-id ?goal-id)
              (action-name bs-dispense)
              (skiller (remote-skiller ?robot))
              (param-names r m side wp basecol)
              (param-values ?robot ?bs INPUT ?spawned-wp ?bs-color))
        (plan-action (id 11) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?spawned-wp ?bs INPUT))
        (plan-action (id 12) (plan-id ?planid) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?bs INPUT ?mps INPUT))
        (plan-action (id 13) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?spawned-wp ?mps))
        (plan-action (id 14) (plan-id ?planid) (goal-id ?goal-id)
              (action-name request-cs-mount-cap)
              (skiller (remote-skiller ?robot))
              (param-values ?robot ?mps ?spawned-wp ?cs-color))
        (plan-action (id 15) (plan-id ?planid) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?spawned-wp ?mps INPUT))
        (plan-action (id 16) (plan-id ?planid) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?mps INPUT (wait-pos ?mps INPUT)))
        )
        (modify ?g (mode EXPANDED))
)
