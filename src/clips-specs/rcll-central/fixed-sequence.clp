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
	(assert (plan-action (action-name ?name) (param-values $?param-values)))
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
