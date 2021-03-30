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


(deftemplate idea
  (slot id (default-dynamic (gensym*)))
  (slot class (type SYMBOL))
  (slot goal-id (type SYMBOL))
  (slot priority (type FLOAT) (default -999.0))
  (slot build-time (type INTEGER) (default 999))
  (slot goal-priority (type FLOAT))
  (slot distance (type FLOAT))
  (slot base-color (type SYMBOL) (default BASE_NONE))
  (slot ring1-color (type SYMBOL) (default RING_NONE))
  (slot ring2-color (type SYMBOL) (default RING_NONE))
  (slot ring3-color (type SYMBOL) (default RING_NONE))
  (slot cap-color (type SYMBOL) (default RING_NONE))
  (multislot params)
)

(deffunction retract-ideas ()
  ; Retract all ideas.
  (printout t " ---------------------------- " crlf)
  (do-for-all-facts ((?i idea)) TRUE
		(retract ?i)
	)
  (refresh idea-production-fetch-cc)
  (refresh idea-production-transport)
  (refresh idea-production-transport-from-hand)
  (refresh idea-production-discard-base)
  (refresh idea-production-discard-base-urgent)
  (refresh idea-production-create-base)
  (refresh idea-production-feed-rs)
)

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
  (retract-ideas)
)

(defglobal
  ?*SALIENCE-MACHINE-GOAL-EXPAND* = (+ ?*SALIENCE-GOAL-EXPAND* 3)
  ?*SALIENCE-IDEA-PRODUCTION* = (+ ?*SALIENCE-GOAL-EXPAND* 2)
  ?*SALIENCE-IDEA-GOAL-EXPAND* = (+ ?*SALIENCE-GOAL-EXPAND* 1)
)


(defrule compute-idea-priority
  (declare (salience (+ 1 ?*SALIENCE-IDEA-PRODUCTION*)))
  ?i <- (idea (priority -999.0) (goal-id ?goal-id) (goal-priority ?goal-priority) (distance ?distance)
              (base-color ?wp-base-color) (ring1-color ?wp-ring1-color) (ring2-color ?wp-ring2-color) (ring3-color ?wp-ring3-color) (cap-color ?wp-cap-color)
  )
  (goal (id ?goal-id) (parent ?parent-id) (class ?class))
  (goal (id ?parent-id) (meta ?order))
  ; Order facts
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  ; Refbox facts
  (wm-fact (key refbox game-time) (values ?game-time ?))
  (wm-fact (key refbox order ?order delivery-begin) (value ?begin))
  (wm-fact (key refbox order ?order delivery-end) (value ?end))
  => 
  (bind ?order_prio 0.0)
  (if (or (eq ?class TRANSPORT) (eq ?class CREATE-BASE)) then
    (bind ?build-time 0)
    (if (neq ?base-color ?wp-base-color) then (bind ?build-time (+ ?build-time ?*TIME-GET-BASE*)))
    (if (neq ?ring1-color ?wp-ring1-color) then (bind ?build-time (+ ?build-time ?*TIME-MOUNT-RING*)))
    (if (neq ?ring2-color ?wp-ring2-color) then (bind ?build-time (+ ?build-time ?*TIME-MOUNT-RING*)))
    (if (neq ?ring3-color ?wp-ring3-color) then (bind ?build-time (+ ?build-time ?*TIME-MOUNT-RING*)))
    (if (neq ?cap-color ?wp-cap-color) then (bind ?build-time (+ ?build-time ?*TIME-MOUNT-CAP*)))
    (bind ?eta (+ ?game-time ?build-time))
    (printout t "ETA for " ?order " " ?goal-id ": " ?begin " "  ?eta " " ?end crlf)
    (bind ?order_prio (/ ?eta (* 17 60 10)))
    (if (<= ?begin ?eta ?end) then (bind ?order_prio (+ ?order_prio 0.9))) ; ETA is in delivery interval
    (if (> ?eta (* 17 60)) then (bind ?order_prio -1.0)) ; ETA is after end of game
    (if (> ?begin (+ ?eta 180)) then (bind ?order_prio -990.0)) ; ETA is more than 3 min before delivery begin
  )
  (bind ?priority (+ ?goal-priority (goal-distance-prio ?distance) ?order_prio))
  (printout t "Priority " ?priority crlf)
  (modify ?i (build-time ?build-time) (priority ?priority))
)


; Fetch CC
(defrule idea-production-fetch-cc
  (declare (salience ?*SALIENCE-IDEA-PRODUCTION*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class FETCH-CC) (params cap-color ?cap-color))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact can-hold args? r ?robot))
  ; CS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?cs s ~BROKEN))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))
  (wm-fact (key domain fact mps-side-free args? m ?cs side INPUT))
  (wm-fact (key domain fact cs-can-perform args? m ?cs op RETRIEVE_CAP))
  (not (plan (mps ?cs)))
=>
  (printout t "Formulated FETCH-CC idea " ?cs " with " ?robot crlf)
  (assert
        (idea (class FETCH-CC)
              (goal-id ?goal-id)
              (distance (node-distance (mps-node ?cs INPUT) ?robot))
              (goal-priority 0.0)
              (params cs ?cs robot ?robot))
  )
)

(defrule goal-expander-fetch-cc
  (declare (salience ?*SALIENCE-IDEA-GOAL-EXPAND*))
  (idea (class FETCH-CC) (priority ?prio&:(> ?prio -100.0)) (goal-id ?goal-id)
        (params cs ?cs robot ?robot))
  (not (idea (priority ?prio2&:(> ?prio2 ?prio))))
  ?g <- (goal (id ?goal-id))
  ; Robot facts
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; wp facts
  (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
  (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?cs spot ?shelf-spot))
=>
  (printout t "Expanded FETCH-CC idea " ?cs " with " ?robot " (" ?prio ")" crlf)
  (bind ?plan-id (sym-cat FETCH-CC-PLAN- (gensym*)))
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
  )
  (modify ?g (mode EXPANDED))
  (retract-ideas)
)


; Retrieve cap
(defrule goal-expander-retrieve-cap
  (declare (salience ?*SALIENCE-MACHINE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class RETRIEVE-CAP) (params cap-color ?cap-color))
  ; CS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?cs t CS))
  (wm-fact (key domain fact mps-team args? m ?cs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?cs s ~BROKEN))
  (wm-fact (key domain fact cs-color args? m ?cs col ?cap-color))
  (wm-fact (key domain fact mps-side-free args? m ?cs side OUTPUT))
  (wm-fact (key domain fact cs-can-perform args? m ?cs op RETRIEVE_CAP))
  (not (plan (mps ?cs)))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side INPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  (not (plan (wp ?wp)))
=>
  (bind ?plan-id (sym-cat RETRIEVE-CAP-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (mps ?cs) (wp ?wp) )
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-cs)
              (param-names m op)
              (param-values ?cs RETRIEVE_CAP))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name cs-retrieve-cap)
              (param-values ?cs ?wp ?cap-color))
  )
  (modify ?g (mode EXPANDED))
)


; Discard base
(defrule idea-production-discard-base
  (declare (salience ?*SALIENCE-IDEA-PRODUCTION*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class DISCARD-BASE) (params cs ?cs))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact can-hold args? r ?robot))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side OUTPUT))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (not (plan (wp ?wp)))
=>
  (printout t "Formulated DISCARD-BASE idea " ?cs " with " ?robot crlf)
  (assert
        (idea (class DISCARD-BASE)
              (goal-id ?goal-id)
              (distance (node-distance (mps-node ?cs OUTPUT) ?robot))
              (goal-priority -0.5)
              (params cs ?cs wp ?wp robot ?robot))
  )
)
(defrule idea-production-discard-base-urgent
  (declare (salience ?*SALIENCE-IDEA-PRODUCTION*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class DISCARD-BASE) (params cs ?cs))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact can-hold args? r ?robot))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?cs side OUTPUT))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (not (plan (wp ?wp)))
  ; CS facts
  (not (wm-fact (key domain fact mps-side-free args? m ?cs side INPUT))) ; WP already at input
=>
  (printout t "Formulated DISCARD-BASE (urgent) idea " ?cs " with " ?robot crlf)
  (assert
        (idea (class DISCARD-BASE)
              (goal-id ?goal-id)
              (distance (node-distance (mps-node ?cs OUTPUT) ?robot))
              (goal-priority 0.5)
              (params cs ?cs wp ?wp robot ?robot))
  )
)


(defrule goal-expander-discard-base
  (declare (salience ?*SALIENCE-IDEA-GOAL-EXPAND*))
  (idea (class DISCARD-BASE) (priority ?prio&:(> ?prio -100.0)) (goal-id ?goal-id)
        (params cs ?cs wp ?wp robot ?robot))
  (not (idea (priority ?prio2&:(> ?prio2 ?prio))))
  ?g <- (goal (id ?goal-id))
  ; Robot facts
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
=>
  (printout t "Expanded DISCARD-BASE idea " ?cs " with " ?robot " (" ?prio ")" crlf)
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
  (retract-ideas)
)


; Transport
(defrule idea-production-transport
  (declare (salience ?*SALIENCE-IDEA-PRODUCTION*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class TRANSPORT)
              (params mps-to ?mps-to base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact can-hold args? r ?robot))
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
  (printout t "Formulated TRANSPORT idea " ?wp " to " ?mps-to " with " ?robot crlf)
  (assert
        (idea (class TRANSPORT)
              (goal-id ?goal-id)
              (distance (* 0.5 (+ (node-distance (mps-node ?mps-from ?mps-from-side) ?robot)
                                  (node-distance (mps-node ?mps-from ?mps-from-side) (mps-node ?mps-to INPUT))
                               )))
              (goal-priority 0.1)
              (base-color ?base-color) (ring1-color ?ring1-color) (ring2-color ?ring2-color) (ring3-color ?ring3-color) (cap-color ?cap-color)
              (params mps-to ?mps-to wp ?wp robot ?robot))
  )
)

(defrule goal-expander-transport
  (declare (salience ?*SALIENCE-IDEA-GOAL-EXPAND*))
  (idea (class TRANSPORT) (priority ?prio&:(> ?prio -100.0)) (goal-id ?goal-id)
        (params mps-to ?mps-to wp ?wp robot ?robot))
  (not (idea (priority ?prio2&:(> ?prio2 ?prio))))
  ?g <- (goal (id ?goal-id))
  ; Robot facts
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  (wm-fact (key domain fact can-hold args? r ?robot))
  ; wp facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps-from side ?mps-from-side))
=>
  (printout t "Expanded TRANSPORT idea " ?wp " to " ?mps-to " with " ?robot " (" ?prio ")" crlf)
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
  (retract-ideas)
)

; Transport from hand
(defrule idea-production-transport-from-hand
  (declare (salience ?*SALIENCE-IDEA-PRODUCTION*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class TRANSPORT)
              (params mps-to ?mps-to base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
  ; Robot facts
  (wm-fact (key domain fact entered-field args? r ?robot))
  (not (plan (r ?robot)))
  (wm-fact (key domain fact holding args? r ?robot wp ?wp)) ; Robot can only have wp already in hand if previous transport failed
  ; wp facts
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
  (printout t "Formulated TRANSPORT (from hand) idea " ?wp " to " ?mps-to " with " ?robot crlf)
  (assert
        (idea (class TRANSPORT)
              (goal-id ?goal-id)
              (distance 0.0)
              (goal-priority 10.0)
              (base-color ?base-color) (ring1-color ?ring1-color) (ring2-color ?ring2-color) (ring3-color ?ring3-color) (cap-color ?cap-color)
              (params mps-to ?mps-to wp ?wp robot ?robot))
  )
)

(defrule goal-expander-transport-from-hand
  (declare (salience ?*SALIENCE-IDEA-GOAL-EXPAND*))
  (idea (class TRANSPORT) (priority ?prio&:(> ?prio -100.0)) (goal-id ?goal-id)
        (params mps-to ?mps-to wp ?wp robot ?robot))
  (not (idea (priority ?prio2&:(> ?prio2 ?prio))))
  ?g <- (goal (id ?goal-id))
  ; Robot facts
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; wp facts
  (wm-fact (key domain fact holding args? r ?robot wp ?wp))
=>
  (printout t "Expanded TRANSPORT (from-hand) idea " ?wp " to " ?mps-to " with " ?robot " (" ?prio ")" crlf)
  (bind ?plan-id (sym-cat TRANSPORT-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (mps ?mps-to) (wp ?wp))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?mps-to INPUT))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?wp ?mps-to))
  )
  (modify ?g (mode EXPANDED))
  (retract-ideas)
)


; Create base
(defrule idea-production-create-base
  (declare (salience ?*SALIENCE-IDEA-PRODUCTION*))
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
  (printout t "Formulated CREATE-BASE idea " ?base-color crlf)
  (assert
        (idea (class CREATE-BASE)
              (goal-id ?goal-id)
              (distance 0.0)
              (goal-priority -1.0)
              (base-color ?base-color)
              (params bs ?bs base-color ?base-color))
  )
)

(defrule goal-expander-create-base
  (declare (salience ?*SALIENCE-IDEA-GOAL-EXPAND*))
  (idea (class CREATE-BASE) (priority ?prio&:(> ?prio -100.0)) (goal-id ?goal-id)
        (params bs ?bs base-color ?base-color))
  (not (idea (priority ?prio2&:(> ?prio2 ?prio))))
  ?g <- (goal (id ?goal-id))
=>
  (printout t "Expanded CREATE-BASE idea " ?base-color " (" ?prio ")" crlf)
  
  (bind ?wp (sym-cat WP- (random-id)))
  (bind ?plan-id (sym-cat FETCH-BASE-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (mps ?bs))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-bs)
              (param-names m side bc)
              (param-values ?bs INPUT ?base-color))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name spawn-wp)
              (param-values ?wp))
    (plan-action (id 3) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name bs-dispense)
              (param-names m side wp basecol)
              (param-values ?bs INPUT ?wp ?base-color))
  )
  (modify ?g (mode EXPANDED))
  (retract-ideas)
)


; Mount cap
(defrule goal-expander-mount-cap
  (declare (salience ?*SALIENCE-MACHINE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (parent ?parent-id) (mode SELECTED) (class MOUNT-CAP) (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color cap-color ?cap-color))
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
    (plan (id ?plan-id) (goal-id ?goal-id) (mps ?cs) (wp ?wp))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-cs)
              (param-names m op)
              (param-values ?cs MOUNT_CAP))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name cs-mount-cap)
              (param-values ?cs ?wp ?cap-color))
  )
  (modify ?g (mode EXPANDED))
)


; Deliver
(defrule goal-expander-deliver
  (declare (salience ?*SALIENCE-MACHINE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class DELIVER) (params order ?order))
  ; Order facts
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
  (wm-fact (key refbox game-time) (values ?game-time ?))
  (wm-fact (key refbox order ?order delivery-begin) (value ?begin&:(> ?game-time ?begin)))
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
    (plan (id ?plan-id) (goal-id ?goal-id) (mps ?ds) (wp ?wp))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-ds)
              (param-names m ord)
              (param-values ?ds ?order))
  )
  (if (eq ?complexity C0) then
    (assert (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name fulfill-order-c0)
                  (param-names ord wp m g basecol capcol)
                  (param-values ?order ?wp ?ds ?gate ?base-color ?cap-color))
    )
  )
  (if (eq ?complexity C1) then
    (assert (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name fulfill-order-c1)
                  (param-names ord wp m g basecol capcol ring1col)
                  (param-values ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color))
    )
  )
  (if (eq ?complexity C2) then
    (assert (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name fulfill-order-c2)
                  (param-names ord wp m g basecol capcol ring1col ring2col)
                  (param-values ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color ?ring2-color))
    )
  )
  (if (eq ?complexity C3) then
    (assert (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
                  (action-name fulfill-order-c3)
                  (param-names ord wp m g basecol capcol ring1col ring2col ring3col)
                  (param-values ?order ?wp ?ds ?gate ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color))
    )
  )
  (modify ?g (mode EXPANDED))
)


; Mount ring 1
(defrule goal-expander-mount-r1
  (declare (salience ?*SALIENCE-MACHINE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class MOUNT-RING1) (params base-color ?base-color ring1-color ?ring1-color))
  ; RS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?rs s ~BROKEN))
  (wm-fact (key domain fact mps-side-free args? m ?rs side OUTPUT))
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
    (plan (id ?plan-id) (goal-id ?goal-id) (mps ?rs) (wp ?wp))
    ; Prepare RS
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-rs)
              (param-names m rc rs-before rs-after r-req)
              (param-values ?rs ?ring1-color ?bases-filled ?bases-remain ?bases-needed))
    ; Mount ring
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name rs-mount-ring1)
              (param-names m wp col rs-before rs-after r-req)
              (param-values ?rs ?wp ?ring1-color ?bases-filled ?bases-remain ?bases-needed))
  )
  (modify ?g (mode EXPANDED))
)


; Mount ring 2
(defrule goal-expander-mount-r2
  (declare (salience ?*SALIENCE-MACHINE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class MOUNT-RING2) (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color))
  ; RS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?rs s ~BROKEN))
  (wm-fact (key domain fact mps-side-free args? m ?rs side OUTPUT))
  (not (plan (mps ?rs)))
  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring2-color&~RING_NONE rn ?bases-needed))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side INPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (not (plan (wp ?wp)))
=>
  (bind ?plan-id (sym-cat MOUNT-RING2-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (mps ?rs) (wp ?wp))
    ; Prepare RS
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-rs)
              (param-names m rc rs-before rs-after r-req)
              (param-values ?rs ?ring2-color ?bases-filled ?bases-remain ?bases-needed))
    ; Mount ring
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name rs-mount-ring2)
              (param-names m wp col col1 rs-before rs-after r-req)
              (param-values ?rs ?wp ?ring2-color ?ring1-color ?bases-filled ?bases-remain ?bases-needed))
  )
  (modify ?g (mode EXPANDED))
)


; Mount ring 3
(defrule goal-expander-mount-r3
  (declare (salience ?*SALIENCE-MACHINE-GOAL-EXPAND*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class MOUNT-RING3) (params base-color ?base-color ring1-color ?ring1-color ring2-color ?ring2-color ring3-color ?ring3-color))
  ; RS facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-type args? m ?rs t RS))
  (wm-fact (key domain fact mps-team args? m ?rs col ?team-color))
  (wm-fact (key domain fact mps-state args? m ?rs s ~BROKEN))
  (wm-fact (key domain fact mps-side-free args? m ?rs side OUTPUT))
  (not (plan (mps ?rs)))
  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
  (wm-fact (key domain fact rs-ring-spec args? m ?rs r ?ring3-color&~RING_NONE rn ?bases-needed))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?bases-needed
                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?rs side INPUT))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col RING_NONE))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col CAP_NONE))
  (not (plan (wp ?wp)))
=>
  (bind ?plan-id (sym-cat MOUNT-RING2-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (mps ?rs) (wp ?wp))
    ; Prepare RS
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name prepare-rs)
              (param-names m rc rs-before rs-after r-req)
              (param-values ?rs ?ring3-color ?bases-filled ?bases-remain ?bases-needed))
    ; Mount ring
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name rs-mount-ring3)
              (param-names m wp col col1 col2 rs-before rs-after r-req)
              (param-values ?rs ?wp ?ring3-color ?ring1-color ?ring2-color ?bases-filled ?bases-remain ?bases-needed))
  )
  (modify ?g (mode EXPANDED))
)


; Feed RS
(defrule idea-production-feed-rs
  (declare (salience ?*SALIENCE-IDEA-PRODUCTION*))
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
=>
  (printout t "Formulated FEED-RS idea " ?rs " with " ?robot crlf)
  (assert
        (idea (class FEED-RS)
              (goal-id ?goal-id)
              (distance (* 0.5 (+ (node-distance (mps-node ?mps-from ?mps-from-side) ?robot)
                                  (node-distance (mps-node ?mps-from ?mps-from-side) (mps-node ?rs INPUT))
                               )))
              (goal-priority 1.0)
              (params rs ?rs wp ?wp robot ?robot))
  )
)

(defrule goal-expander-feed-rs
  (declare (salience ?*SALIENCE-IDEA-GOAL-EXPAND*))
  (idea (class FEED-RS) (priority ?prio&:(> ?prio -100.0)) (goal-id ?goal-id)
        (params rs ?rs wp ?wp robot ?robot))
  (not (idea (priority ?prio2&:(> ?prio2 ?prio))))
  ?g <- (goal (id ?goal-id))
  ; Robot facts
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
  ; WP facts
  (wm-fact (key domain fact wp-at args? wp ?wp m ?mps-from side ?mps-from-side))
  ; RS facts
  (wm-fact (key domain fact rs-filled-with args? m ?rs n ?bases-filled))
  (wm-fact (key domain fact rs-inc args? summand ?bases-filled sum ?bases-after))
=>
  (printout t "Expanded FEED-RS idea " ?rs " with " ?robot " (" ?prio ")" crlf)
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
  (retract-ideas)
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

; Put away
(defrule idea-production-put-away
  (declare (salience ?*SALIENCE-IDEA-PRODUCTION*))
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class PUT-AWAY)
              (params robot ?robot wp ?wp))
  ; mps facts
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key domain fact mps-team args? m ?mps-to col ?team-color))
  (wm-fact (key domain fact mps-type args? m ?mps-to t ~SS))
  (wm-fact (key domain fact mps-side-free args? m ?mps-to side INPUT))
  (not (plan (mps ?mps-to)))
=>
  (printout t "Formulated PUT-AWAY idea " ?wp " to " ?mps-to " with " ?robot crlf)
  (assert
        (idea (class PUT-AWAY)
              (goal-id ?goal-id)
              (priority (goal-distance-prio (node-distance (mps-node ?mps-to INPUT) ?robot)))
              (distance 0.0)
              (goal-priority 5.0)
              (params mps-to ?mps-to wp ?wp robot ?robot))
  )
)

(defrule goal-expander-put-away
  (declare (salience ?*SALIENCE-IDEA-GOAL-EXPAND*))
  (idea (class PUT-AWAY) (priority ?prio&:(> ?prio -100.0)) (goal-id ?goal-id)
        (params mps-to ?mps-to wp ?wp robot ?robot))
  (not (idea (priority ?prio2&:(> ?prio2 ?prio))))
  ?g <- (goal (id ?goal-id))
  ; Robot facts
  (wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
=>
  (printout t "Expanded PUT-AWAY idea " ?wp " to " ?mps-to " with " ?robot " (" ?prio ")" crlf)
  (bind ?plan-id (sym-cat PUT-AWAY-PLAN- (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (r ?robot) (mps ?mps-to) (wp ?wp))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name move)
              (skiller (remote-skiller ?robot))
              (param-names r from from-side to to-side)
              (param-values ?robot ?curr-location ?curr-side ?mps-to INPUT))
    (plan-action (id 2) (plan-id ?plan-id) (goal-id ?goal-id)
              (action-name wp-put)
              (skiller (remote-skiller ?robot))
              (param-names r wp m)
              (param-values ?robot ?wp ?mps-to))
  )
  (modify ?g (mode EXPANDED))
  (retract-ideas)
)
