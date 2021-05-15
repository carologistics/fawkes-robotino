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

(deffunction plan-assert-action (?name $?param-values)
" Assert an action with a unique id."
	(bind ?id-sym (gensym*))
	(bind ?id-str (sub-string 4 (length$ ?id-sym) (str-cat ?id-sym)))
	(assert (plan-action (id (string-to-field ?id-str)) (action-name ?name) (param-values $?param-values)))
)

(deffunction plan-assert-sequential (?plan-id ?goal-id ?robot $?action-tuples)
	(assert (plan (id ?plan-id) (goal-id ?goal-id)))
	(foreach ?pa $?action-tuples
		(modify ?pa (id ?pa-index) (plan-id ?plan-id) (goal-id ?goal-id)
		            (skiller (remote-skiller ?robot)))
	)
)

(deffunction plan-assert-safe-move (?robot
                                    ?curr-location
                                    ?curr-side
                                    ?mps
                                    ?mps-side
                                    $?actions)
	(return
	  (create$
	    (plan-assert-action go-wait
	      ?robot ?curr-location ?curr-side (wait-pos ?mps ?mps-side))
	    (plan-assert-action location-lock
	      ?mps ?mps-side)
	    (plan-assert-action move
	      ?robot (wait-pos ?mps ?mps-side) WAIT ?mps ?mps-side)
	    $?actions
	    (plan-assert-action location-unlock
	      ?mps ?mps-side)
	    (plan-assert-action go-wait
	      ?robot ?mps ?mps-side (wait-pos ?mps ?mps-side))
	  )
	)
)

(deffunction is-holding (?robot ?wp)
	(if (any-factp ((?holding wm-fact))
	           (and (wm-key-prefix ?holding:key (create$ domain fact holding))
	                (eq (wm-key-arg ?holding:key r) ?robot)
	                (eq (wm-key-arg ?holding:key wp) ?wp)))
	 then
		(return TRUE)
	 else
		(return FALSE)
	)
)

(defrule goal-expander-send-beacon-signal
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class SEND-BEACON) (mode SELECTED)
              (parent ?parent-id))
=>
	(plan-assert-sequential BEACONPLAN ?goal-id central
		(plan-assert-action send-beacon)
	)
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-refill-shelf
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class REFILL-SHELF) (mode SELECTED)
              (params mps ?mps) (parent ?parent-id))
  (wm-fact (key domain fact cs-color args? m ?mps col ?col))
  =>
	(plan-assert-sequential REFILL-PLAN ?goal-id central
		(plan-assert-action refill-shelf
		  ?mps LEFT (sym-cat CC- (random-id)) ?col)
		(plan-assert-action refill-shelf
		  ?mps MIDDLE (sym-cat CC- (random-id)) ?col)
		(plan-assert-action refill-shelf
		  ?mps RIGHT (sym-cat CC- (random-id)) ?col)
	)
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-enter-field
  ?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
              (params r ?robot team-color ?team-color))
=>
	(plan-assert-sequential ENTER-FIELD-PLAN ?goal-id ?robot
		(plan-assert-action enter-field ?robot ?team-color)
	)
  (modify ?g (mode EXPANDED))
)

(defrule goal-expander-prefill-cap-station
" Feed a CS with a cap from its shelf so that afterwards
   it can directly put the cap on a product."
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class FILL-CAP) (mode SELECTED) (parent ?parent)
	            (params robot ?robot
	                    mps ?mps
	                    cc ?cc
	                    cap-color ?cap-color
	            ))
	(wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?shelf-spot))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(plan-assert-sequential (sym-cat FILL-CAP-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-location ?curr-side ?mps INPUT
			(plan-assert-action wp-get-shelf ?robot ?cc ?mps ?shelf-spot)
			(plan-assert-action wp-put ?robot ?cc ?mps)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-transport-goals
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class ?class&MOUNT-CAP|MOUNT-RING|DELIVER)
	                          (mode SELECTED) (parent ?parent)
	                          (params robot ?robot
	                                   wp ?wp
	                                   wp-loc ?wp-loc
	                                   wp-side ?wp-side
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
		(if (not (is-holding ?robot ?wp))
		 then
			(create$ ; only last statement of if is returned
				(plan-assert-safe-move ?robot ?curr-location ?curr-side ?wp-loc ?wp-side
					(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side)
				)
				(plan-assert-safe-move ?robot ?wp-loc ?wp-side ?target-mps ?target-side
					(plan-assert-action wp-put ?robot ?wp ?target-mps)
				)
			)
		 else
			(plan-assert-safe-move ?robot ?curr-location ?curr-side ?target-mps ?target-side
				(plan-assert-action wp-put ?robot ?wp ?target-mps)
			)
		)
	)
	(modify ?g (mode EXPANDED))
)
