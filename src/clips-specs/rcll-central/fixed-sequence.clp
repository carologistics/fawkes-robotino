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

(deffunction plan-assert-sequential (?plan-name ?goal-id ?robot $?action-tuples)
	(bind ?plan-id (sym-cat ?plan-name (gensym*)))
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
	    ;(plan-assert-action location-lock
	    ;  ?mps ?mps-side)
	    (plan-assert-action move
	      ?robot (wait-pos ?mps ?mps-side) WAIT ?mps ?mps-side)
	    $?actions
	    ;(plan-assert-action location-unlock
	    ;  ?mps ?mps-side)
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
	            (params team-color ?team-color)
	            (meta $? assigned-to ?robot $?))
	=>
	(plan-assert-sequential ENTER-FIELD-PLAN ?goal-id ?robot
		(plan-assert-action enter-field ?robot ?team-color)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-pick-and-place
" Picks a wp from the output of the given mps
  feeds it into the input of the same mps
  moves back to the output of the mps "
	?g <- (goal (id ?goal-id) (class PICK-AND-PLACE) (mode SELECTED) (parent ?parent)
	            (params target-mps ?mps )
	            (meta $? assigned-to ?robot $?))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	=>
	(plan-assert-sequential (sym-cat PICK-AND-PLACE-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-action move ?robot ?curr-location ?curr-side ?mps OUTPUT)
		(plan-assert-action wp-get ?robot ?wp ?mps OUTPUT)
		(plan-assert-action move ?robot ?mps OUTPUT ?mps INPUT)
		(plan-assert-action wp-put ?robot ?wp ?mps)
		(plan-assert-action move ?robot ?mps INPUT ?mps OUTPUT)
		(plan-assert-action move-wp-input-output ?mps ?wp)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-buffer-cap
" Feed a CS with a cap from its shelf so that afterwards
   it can directly put the cap on a product."
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class BUFFER-CAP) (mode SELECTED) (parent ?parent)
	            (params target-mps ?mps
	                    cap-color ?cap-color
	            )
	            (meta $? assigned-to ?robot $?))
	(or
		(wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?throwaway-spot))
		(wm-fact (key domain fact holding args? r ?robot wp ?cc))
	)
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(bind ?shelf-spot nil)
	(if (not (is-holding ?robot ?cc))
		then
		(do-for-fact ((?wp-on-shelf wm-fact)) (and (wm-key-prefix ?wp-on-shelf:key (create$ domain fact wp-on-shelf))
		                                      (eq (wm-key-arg ?wp-on-shelf:key wp) ?cc)
		                                      (eq (wm-key-arg ?wp-on-shelf:key m) ?mps))
			(bind ?shelf-spot (wm-key-arg ?wp-on-shelf:key spot))
		)
	)

	(plan-assert-sequential (sym-cat BUFFER-CAP-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-location ?curr-side ?mps INPUT
			(if (not (is-holding ?robot ?cc)) then
				(create$
				    (plan-assert-action wp-get-shelf ?robot ?cc ?mps ?shelf-spot)
				)
			)
			(plan-assert-action wp-put ?robot ?cc ?mps)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-discard
" Pick up a product and discard it."
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class DISCARD) (mode SELECTED) (parent ?parent)
	            (params wp ?wp wp-loc ?wp-loc wp-side ?wp-side)
	            (meta $? assigned-to ?robot $?))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(plan-assert-sequential (sym-cat DISCARD-PLAN- (gensym*)) ?goal-id ?robot
		(if (not (is-holding ?robot ?wp))
		 then
			(create$ ; only last statement of if is returned
				(plan-assert-safe-move ?robot ?curr-location ?curr-side ?wp-loc ?wp-side
					(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side)
				)
			)
		)
		(plan-assert-action wp-discard ?robot ?wp)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-transport-goals
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class ?class&MOUNT-CAP|MOUNT-RING|DELIVER)
	                          (mode SELECTED) (parent ?parent)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   target-side ?target-side
	                                   $?params)
	                          (meta $? assigned-to ?robot $?))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(if (not (do-for-fact ((?wp-at wm-fact)) (and (wm-key-prefix ?wp-at:key (create$ domain fact wp-at)) (eq (wm-key-arg ?wp-at:key wp) ?wp))
		(bind ?wp-loc (wm-key-arg ?wp-at:key m))
		(bind ?wp-side (wm-key-arg ?wp-at:key side))))
		then
		(bind ?wp-loc (multifield-key-value ?params wp-loc))
		(bind ?wp-side (multifield-key-value ?params wp-side))
	)
	(if (eq ?wp-loc nil) 
		then 
			(printout error "The fields wp-loc and wp-side must either be set manually or there must be a wp-at fact!" crlf)
		else
			(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
				(if (not (is-holding ?robot ?wp))
				then
					(create$ ; only last statement of if is returned
						(plan-assert-safe-move ?robot ?curr-location ?curr-side ?wp-loc ?wp-side
							(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side)
						)
						(plan-assert-safe-move ?robot (wait-pos ?wp-loc ?wp-side) WAIT ?target-mps ?target-side
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
	)

	


(defrule goal-expander-instruct-cs-buffer-cap
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP) (mode SELECTED)
	            (params target-mps ?mps cap-color ?cap-color)
	            (meta $? assigned-to ?assigned $?))
	(wm-fact (key domain fact wp-at args? wp ?cap-carrier m ?mps side INPUT))
	=>
	(plan-assert-sequential INSTRUCT-TO-BUFFER-CAP-PLAN ?goal-id ?assigned
		(plan-assert-action prepare-cs ?mps RETRIEVE_CAP)
		(plan-assert-action cs-retrieve-cap ?mps ?cap-carrier ?cap-color)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-cs-mount-cap
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class INSTRUCT-CS-MOUNT-CAP) (mode SELECTED)
	            (params target-mps ?mps cap-color ?cap-color)
	            (meta $? assigned-to ?assigned $?))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
	=>
	(plan-assert-sequential INSTRUCT-TO-MOUNT-CAP-PLAN ?goal-id ?assigned
		(plan-assert-action prepare-cs ?mps MOUNT_CAP)
		(plan-assert-action cs-mount-cap ?mps ?wp ?cap-color)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-bs-dispense-base
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class INSTRUCT-BS-DISPENSE-BASE) (mode SELECTED)
	            (params wp ?wp target-mps ?mps  target-side ?side base-color ?base-color)
	            (meta $? assigned-to ?assigned $?))
	=>
	(plan-assert-sequential INSTRUCT-BS-DISPENSE-BASE-PLAN ?goal-id ?assigned
		(plan-assert-action prepare-bs ?mps ?side ?base-color)
		(plan-assert-action bs-dispense ?mps ?side ?wp ?base-color)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-ds-deliver
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DELIVER) (mode SELECTED)
	            (params wp ?wp target-mps ?mps)
	            (meta $? assigned-to ?assigned $?))
	(wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
	(wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
	(wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
	(wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
	(wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
	; Order-CEs
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
	(wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
	(wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
	(wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
	(wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
	(wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
	(wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
	=>
	(bind ?params (create$))
	(switch ?complexity
		(case C0 then
		    (bind ?params (create$ ?order ?wp ?mps ?gate ?base-color ?cap-color)))
		(case C1 then
		    (bind ?params (create$ ?order ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color)))
		(case C2 then
		    (bind ?params (create$ ?order ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color ?ring2-color)))
		(case C3 then
		    (bind ?params (create$ ?order ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)))
 )
	(plan-assert-sequential INSTRUCT-DS-DELIVER-PLAN ?goal-id ?assigned
		(plan-assert-action prepare-ds ?mps ?order)
		(plan-assert-action (sym-cat fulfill-order- (lowcase ?complexity)) ?params)
	)
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-navigation-challenge-move
	?g <- (goal (id ?goal-id) (class NAVIGATION-CHALLENGE-MOVE) (mode SELECTED)
				(params target ?target)
	            (meta $? assigned-to ?robot $?))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(plan-assert-sequential NAVIGATION-CHALLENGE-MOVE-PLAN ?goal-id ?robot
		(plan-assert-action go-wait
			?robot ?curr-location ?curr-side ?target);target is a waitpoint
		(plan-assert-action wait
			?robot ?target);wait there
	)
	(modify ?g (mode EXPANDED))
)

