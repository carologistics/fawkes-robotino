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
                 (action-name refill-shelf)
                 (param-values ?mps LEFT (sym-cat CC- (random-id)) ?col))
    (plan-action (id 2) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf)
                 (param-values ?mps MIDDLE (sym-cat CC- (random-id)) ?col))
    (plan-action (id 3) (plan-id REFILL-PLAN) (goal-id ?goal-id)
                 (action-name refill-shelf)
                 (param-values ?mps RIGHT (sym-cat CC- (random-id)) ?col))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-enter-field
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
              (params r ?robot team-color ?team-color))
  (not (exists
      (wm-fact (key domain fact entered-field args? r ?r))
      (wm-fact (key domain fact at args? r ?r m START side INPUT))
  ))
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
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class PREPARE-CAP) (params cap-color ?cap-color))
  (not (goal (mode ~SELECTED) (class PREPARE-CAP) (params cap-color ?cap-color)))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact can-hold args? r ?robot))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; CS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?cs s ~BROKEN))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))
  (wm-fact (key domain fact mps-side-free args? m ?cs side INPUT))
  (wm-fact (key domain fact mps-side-free args? m ?cs side OUTPUT))
  (wm-fact (key domain fact cs-can-perform args? m ?cs op RETRIEVE_CAP))
  (not (plan (mps ?cs)))
  ; wp facts
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))  
=>
  (bind ?plan-id (sym-cat PREPARE-CAP-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (mps ?cs))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?cs INPUT))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get-shelf)
              (skiller (remote-skiller ?robot))
              (param-names r cc m spot)
              (param-values ?robot ?cc ?cs ?shelf-spot))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?cc ?cs))
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-cs)
              (skiller (remote-skiller ?robot))
              (param-names m op)
              (param-values ?cs RETRIEVE_CAP))
    (plan-action (id 5) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name cs-retrieve-cap)
              (skiller (remote-skiller ?robot))
              (param-values ?cs ?cc ?cap-color))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-discard-base
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class DISCARD-BASE) (params cs ?cs))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact can-hold args? r ?robot))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side OUTPUT))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (not (plan (wp ?wp)))
=>
  (bind ?plan-id (sym-cat DISCARD-BASE-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (wp ?wp))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?cs OUTPUT))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?cs OUTPUT))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-discard)
              (skiller (remote-skiller ?robot))
              (param-names r cc )
              (param-values ?robot ?wp))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-transport
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode SELECTED) (class TRANSPORT)
              (params mps-to ?mps-to base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact can-hold args? r ?robot))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; wp facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps-from side ?mps-from-side))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  (not (plan (wp ?wp)))
  ; mps facts
  (wm-fact (key domain fact mps-side-free args? m ?mps-to side INPUT))
  (or  
    (wm-fact (key domain fact mps-type args? m ?mps-to t ~CS))
    (wm-fact (key domain fact cs-can-perform args? m ?mps-to op MOUNT_CAP))
  )
  (not (plan (mps ?mps-to)))
=>
  (bind ?plan-id (sym-cat TRANSPORT-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (mps ?mps-to) (wp ?wp))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?mps-from ?mps-from-side))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?mps-from ?mps-from-side))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?mps-from ?mps-from-side ?mps-to INPUT))
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?wp ?mps-to))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-create-base
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode SELECTED) (class CREATE-BASE) (params base-color ?base-color))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  ; BS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?bs t BS))
  (wm-fact (key domain fact mps-team args? m ?bs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?bs s ~BROKEN))
  (wm-fact (key domain fact mps-side-free args? m ?bs side INPUT))
  (not (plan (mps ?bs)))
=>
  (bind ?wp (sym-cat WP- (random-id)))
  (bind ?plan-id (sym-cat FETCH-BASE-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (mps ?bs))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-bs)
              (skiller (remote-skiller ?robot))
              (param-names m side bc)
              (param-values ?bs INPUT ?base-color))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name spawn-wp)
              (skiller (remote-skiller ?robot))
              (param-values ?wp ?robot))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name bs-dispense)
              (skiller (remote-skiller ?robot))
              (param-names r m side wp basecol)
              (param-values ?robot ?bs INPUT ?wp ?base-color))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-mount-cap
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode SELECTED) (class MOUNT-CAP) (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  ; wp facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side INPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (not (plan (wp ?wp)))
  ; CS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?cs s ~BROKEN))
  (wm-fact (key domain fact mps-side-free args? m ?cs side OUTPUT))
  (wm-fact (key domain fact cs-buffered args? m ?cs col ?cap-color))
  (not (plan (mps ?cs)))
=>
  (bind ?plan-id (sym-cat MOUNT-CAP-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (mps ?cs) (wp ?wp))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-cs)
              (skiller (remote-skiller ?robot))
              (param-names m op)
              (param-values ?cs MOUNT_CAP))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name cs-mount-cap)
              (skiller (remote-skiller ?robot))
              (param-values ?cs ?wp ?cap-color))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-deliver
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class DELIVER) (params order ?order))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  ; Order facts
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
  ; DS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?ds t DS))
  (wm-fact (key domain fact mps-team args? m ?ds col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?ds s ~BROKEN))
  (not (plan (mps ?ds)))
  ; wp facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?ds side INPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  (not (plan (wp ?wp)))
=>
  (bind ?plan-id (sym-cat DELIVER-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (mps ?ds) (wp ?wp))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-ds)
              (skiller (remote-skiller ?robot))
              (param-names m ord)
              (param-values ?ds ?order))
  )
  (if (eq ?complexity C0) then
    (assert (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name fulfill-order-c0)
                  (skiller (remote-skiller ?robot))
                  (param-names ord wp m g basecol capcol)
                  (param-values ?order ?wp ?ds ?gate ?base-color ?cap-color))
    )
  )
  (if (eq ?complexity C1) then
    (assert (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name fulfill-order-c1)
                  (skiller (remote-skiller ?robot))
                  (param-names ord wp m g basecol capcol ring1col)
                  (param-values ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color))
    )
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-mount-r1
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class MOUNT-RING1) (params base-color ?base-color ring1-color ?ring1-color))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  ; RS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?rs s ~BROKEN))
  (not (plan (mps ?rs)))
  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring1-color&~RING_NONE rn ?bases-needed))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side INPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (not (plan (wp ?wp)))
=>
  (bind ?plan-id (sym-cat MOUNT-RING1-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id))
    ; Prepare RS
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-rs)
              (skiller (remote-skiller ?robot))
              (param-names m rc rs-before rs-after r-req)
              (param-values ?rs ?ring1-color ?bases-filled ?bases-remain ?bases-needed))
    ; Mount ring
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name rs-mount-ring1)
              (skiller (remote-skiller ?robot))
              (param-names m wp col rs-before rs-after r-req)
              (param-values ?rs ?wp ?ring1-color ?bases-filled ?bases-remain ?bases-needed))
  )
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-feed-rs
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode SELECTED) (class FEED-RS)
              (params ring-color ?ring-color))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (wm-fact (key domain fact can-hold args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps-from side ?mps-from-side))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (not (plan (wp ?wp)))
  ; RS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?rs s ~BROKEN))
  (not (plan (mps ?rs)))
  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring-color rn ?bases-needed))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-needed
                                         subtrahend ?bases-filled
                                         difference ?bases-remain&ONE|TWO|THREE))
  (wm-fact (key domain fact rs-inc args? summand ?bases-filled sum ?bases-after))
=>
  (bind ?plan-id (sym-cat FEED-RS-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (mps ?rs) (wp ?wp))
    ; Move to MPS where WP is
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?mps-from ?mps-from-side))
    ; Grap WP
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-get)
              (skiller (remote-skiller ?robot))
              (param-names r wp m side)
              (param-values ?robot ?wp ?mps-from ?mps-from-side))
    ; Move to RS
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?mps-from ?mps-from-side ?rs INPUT))
    ; Feed base
    (plan-action (id 4) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-put-slide-cc)
              (skiller (remote-skiller ?robot))
              (param-names r wp m rs-before rs-after)
              (param-values ?robot ?wp ?rs ?bases-filled ?bases-after))
  )
  (modify ?g (mode EXPANDED))
)


(defrule goal-expander-go-wait
  (declare (salience ?*SALIENCE-GOAL-EXPAND-OPTIONAL*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class GO-WAIT) (params r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact at args? r ?robot m ?mps side ?mps-side&INPUT|OUTPUT))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
=>
  (printout t "Expanding GO_WAIT with " ?robot " " ?mps crlf)
  (bind ?plan-id (sym-cat GO_WAIT-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name go-wait)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to)
              (param-values ?robot ?mps ?mps-side (wait-pos ?mps ?mps-side)))
  )
  (modify ?g (mode EXPANDED))
)