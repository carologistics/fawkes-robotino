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
	(bind ?actions (create$))
	; action tuples might contain FALSE in some cases, filter them out
	(foreach ?pa $?action-tuples
		(if ?pa then
			(bind ?actions (append$ ?actions ?pa))
		)
	)
	(foreach ?pa $?actions
		(modify ?pa (id ?pa-index) (plan-id ?plan-id) (goal-id ?goal-id)
		            (skiller (remote-skiller ?robot)))
	)
)

(deffunction plan-assert-move (?robot
                                    ?curr-location
                                    ?curr-side
                                    ?mps
                                    ?mps-side
                                    $?actions)
	(return
	  (create$
	    (plan-assert-action move
	      ?robot ?curr-location ?curr-side ?mps ?mps-side)
	    $?actions
	  )
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
	    (plan-assert-action move
	      ?robot (wait-pos ?mps ?mps-side) WAIT ?mps ?mps-side)
	    $?actions
	    (plan-assert-action go-wait
	      ?robot ?mps ?mps-side (wait-pos ?mps ?mps-side))
	  )
	)
)

(deffunction plan-assert-move-wait-for-wp (?robot
                                                ?curr-location
                                                ?curr-side
                                                ?mps
                                                ?mps-side
                                                ?wp
                                                $?actions)
	(return
	  (create$
	    (plan-assert-action move
	      ?robot ?curr-location ?curr-side ?mps ?mps-side)
	    (plan-assert-action wait-for-wp
	      ?robot ?mps ?mps-side ?wp)
	    $?actions
	  )
	)
)

(deffunction plan-assert-safe-move-wait-for-wp (?robot
                                                ?curr-location
                                                ?curr-side
                                                ?mps
                                                ?mps-side
                                                ?wp
                                                $?actions)
	(return
	  (create$
	    (plan-assert-action go-wait
	      ?robot ?curr-location ?curr-side (wait-pos ?mps ?mps-side))
	    (plan-assert-action move
	      ?robot (wait-pos ?mps ?mps-side) WAIT ?mps ?mps-side)
	    (plan-assert-action wait-for-wp
	      ?robot ?mps ?mps-side ?wp)
	    $?actions
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

(defrule goal-expander-explore-zone
	?g <- (goal (id ?goal-id) (mode SELECTED) (class EXPLORE-ZONE)
	            (params z ?zn))
	(goal-meta (goal-id ?goal-id) (assigned-to ?r&~nil))
	(wm-fact (key refbox team-color) (value ?team-color))
	=>
	(plan-assert-sequential EXPLORE-ZONE ?goal-id ?r
		(plan-assert-action explore-zone ?zn)
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
	            (params zone ?zone))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	=>
	(plan-assert-sequential ENTER-FIELD-PLAN ?goal-id ?robot
		(plan-assert-action enter-field ?robot ?zone)
	)
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-move-out-of-way
	?g <- (goal (id ?goal-id) (mode SELECTED) (class MOVE-OUT-OF-WAY)
	            (params target-pos ?target-pos))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	=>
	(plan-assert-sequential MOVE-OUT-OF-WAY-PLAN ?goal-id ?robot
		(plan-assert-action go-wait ?robot ?curr-loc ?curr-side ?target-pos)
		(plan-assert-action wait ?robot ?target-pos WAIT)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-cleanup-wp
	?g <- (goal (id ?goal-id) (mode SELECTED) (class CLEANUP-WP))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact holding args? r ?robot wp ?wp))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	(wm-fact (key domain fact mps-type args? m ?target-mps t DS))
	(wm-fact (key domain fact mps-team args? m ?target-mps col ?col))
	(wm-fact (key refbox team-color) (value ?col))
	=>
	(plan-assert-sequential (sym-cat CLEANUP-WP-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-location ?curr-side ?target-mps INPUT
			(plan-assert-action wp-put ?robot ?wp ?target-mps INPUT (get-wp-complexity ?wp))
			(plan-assert-action wp-check ?robot ?wp ?target-mps INPUT THERE)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-pick-and-place
" Picks a wp from the output of the given mps
  feeds it into the input of the same mps
  moves back to the output of the mps "
	?g <- (goal (id ?goal-id) (class PICK-AND-PLACE) (mode SELECTED) (parent ?parent)
	            (params target-mps ?mps ))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	(or (wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	    (wm-fact (key domain fact holding args? r ?robot wp ?wp))
	)
	=>
	(plan-assert-sequential (sym-cat PICK-AND-PLACE-PLAN- (gensym*)) ?goal-id ?robot
		(if (not (is-holding ?robot ?wp)) then
			(create$
			    (plan-assert-action move ?robot ?curr-location ?curr-side ?mps OUTPUT)
			    (plan-assert-action wp-get ?robot ?wp ?mps OUTPUT (get-wp-complexity ?wp))
				(plan-assert-action wp-check ?robot ?wp ?mps OUTPUT ABSENT)
			    (plan-assert-action move ?robot ?mps OUTPUT ?mps INPUT)
			)
		 else
			(create$
			    (plan-assert-action move ?robot ?curr-location ?curr-side ?mps INPUT)
			)
		)

		(plan-assert-action wp-put ?robot ?wp ?mps INPUT (get-wp-complexity ?wp))
		(plan-assert-action wp-check ?robot ?wp ?mps INPUT THERE)
		(plan-assert-action move-wp-input-output ?mps ?wp)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-move-robot-to-output
" Moves the robot to the output of the given mps."
	?g <- (goal (id ?goal-id) (class MOVE) (mode SELECTED) (parent ?parent)
	            (params target-mps ?mps ))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	=>
	(plan-assert-sequential (sym-cat MOVE-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-action move ?robot ?curr-loc ?curr-side ?mps OUTPUT)
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
	            ))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(or
		(and (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps $?))
		     (not (wm-fact (key domain fact holding args? r ?robot $?)))
		)
		(and (wm-fact (key domain fact holding args? r ?robot wp ?cc))
		     (not (wm-fact (key domain fact wp-on-shelf args? wp ?cc $?)))
		)
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
		(plan-assert-move ?robot ?curr-location ?curr-side ?mps INPUT
			(if (not (is-holding ?robot ?cc)) then
				(create$
				    (plan-assert-action wp-get-shelf ?robot ?cc ?mps ?shelf-spot)
				)
			)
			(plan-assert-action wp-put ?robot ?cc ?mps INPUT (get-wp-complexity ?cc))
			(plan-assert-action wp-check ?robot ?cc ?mps INPUT THERE)
			(plan-assert-action wait-for-mps ?robot ?cc ?mps INPUT)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-reformulate-no-expansion
	(declare (salience ?*SALIENCE-LOW*))
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?id) (mode SELECTED) (priority ?p))
	?gm <- (goal-meta (goal-id ?id) (assigned-to ?robot&:(and (neq ?robot central) (neq ?robot nil))))
	=>
	(printout error "Goal " ?id " stuck on SELECTED, reformulate with lower priority" crlf)
	(modify ?g (mode FORMULATED) (priority (- ?p 1)))
	(set-robot-to-waiting ?robot)
	(modify ?gm (assigned-to nil))
)

(defrule goal-expander-transport-goals
	?g <- (goal (id ?goal-id) (class ?class&MOUNT-CAP|
	                                       MOUNT-RING|DELIVER|DISCARD)
	                          (mode SELECTED) (parent ?parent)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   $?))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(domain-fact (name at) (param-values ?robot ?curr-location ?curr-side))
	(domain-fact (name wp-at) (param-values ?wp ?wp-loc ?wp-side))
	=>
	(if (neq ?class DELIVER) then
		(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
			(create$ ; only last statement of if is returned
					(plan-assert-move-wait-for-wp ?robot ?curr-location ?curr-side ?wp-loc ?wp-side ?wp
						(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side (get-wp-complexity ?wp))
						(plan-assert-action wp-check ?robot ?wp ?wp-loc ?wp-side ABSENT)
					)
					(plan-assert-move ?robot ?wp-loc ?wp-side ?target-mps INPUT
						(plan-assert-action wp-put ?robot ?wp ?target-mps INPUT (get-wp-complexity ?wp))
						(plan-assert-action wp-check ?robot ?wp ?target-mps INPUT THERE)
						(plan-assert-action wait-for-mps ?robot ?wp ?target-mps INPUT)
					)
			)
		)
	else
		(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
			(create$ ; only last statement of if is returned
					(plan-assert-move-wait-for-wp ?robot ?curr-location ?curr-side ?wp-loc ?wp-side ?wp
						(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side (get-wp-complexity ?wp))
						(plan-assert-action wp-check ?robot ?wp ?wp-loc ?wp-side ABSENT)
					)
					(plan-assert-move ?robot ?wp-loc ?wp-side ?target-mps INPUT
						(plan-assert-action wp-put ?robot ?wp ?target-mps INPUT (get-wp-complexity ?wp))
						(plan-assert-action wait-for-mps ?robot ?wp ?target-mps INPUT)
					)
			)
		)

	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-transport-goals-holding
	?g <- (goal (id ?goal-id) (class ?class&MOUNT-CAP|
	                                       MOUNT-RING|DELIVER|DISCARD)
	                          (mode SELECTED) (parent ?parent)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   $?))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(domain-fact (name at) (param-values ?robot ?curr-location ?curr-side))
	(domain-fact (name holding) (param-values ?robot ?wp))
	=>
	(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-move ?robot ?curr-location ?curr-side ?target-mps INPUT
			(plan-assert-action wp-put ?robot ?wp ?target-mps INPUT (get-wp-complexity ?wp))
			(plan-assert-action wp-check ?robot ?wp ?target-mps INPUT THERE)
			(plan-assert-action wait-for-mps ?robot ?wp ?target-mps INPUT)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-pay-for-rings-with-base-holding
	?g <- (goal (id ?goal-id) (class ?class&PAY-FOR-RINGS-WITH-BASE)
	                          (mode SELECTED) (parent ?parent)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   $?))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(domain-fact (name at) (param-values ?robot ?curr-location ?curr-side))
	(domain-fact (name holding) (param-values ?robot ?wp))
	(domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))
	(domain-fact (name rs-filled-with) (param-values ?target-mps ?rs-before))
	=>
	(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-move ?robot ?curr-location ?curr-side ?target-mps INPUT
			(plan-assert-action wp-put-slide-cc ?robot ?wp ?target-mps ?rs-before ?rs-after)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-transport-goals-base-station
	?g <- (goal (id ?goal-id) (class ?class&MOUNT-CAP|
	                                       MOUNT-RING|DELIVER|DISCARD)
	                          (mode SELECTED) (parent ?parent)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   $?))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(domain-fact (name at) (param-values ?robot ?curr-location ?curr-side))
	(domain-fact (name mps-type) (param-values ?bs BS))
	(wm-fact (key refbox team-color) (value ?team-color))
	(domain-fact (name mps-team) (param-values ?bs ?team-color))
	(domain-fact (name wp-unused) (param-values ?wp))
	(domain-fact (name wp-base-color) (param-values ?wp BASE_NONE))
	=>
	(bind ?wp-loc ?bs)
	(bind ?wp-side INPUT)
  (do-for-fact ((?g goal)) (and (eq ?g:class INSTRUCT-BS-DISPENSE-BASE) (eq ?g:mode FORMULATED) (member$ ?wp ?g:params))
    (bind  ?wp-side (get-param-by-arg ?g:params target-side))
  )
	(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
		(create$ ; only last statement of if is returned
				(plan-assert-move-wait-for-wp ?robot ?curr-location ?curr-side ?wp-loc ?wp-side ?wp
					(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side (get-wp-complexity ?wp))
					(plan-assert-action wp-check ?robot ?wp ?wp-loc ?wp-side ABSENT)
				)
				(plan-assert-move ?robot ?wp-loc ?wp-side ?target-mps INPUT
					(plan-assert-action wp-put ?robot ?wp ?target-mps INPUT (get-wp-complexity ?wp))
					(plan-assert-action wp-check ?robot ?wp ?target-mps INPUT THERE)
					(plan-assert-action wait-for-mps ?robot ?wp ?target-mps INPUT)
				)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-pay-for-rings-with-base-base-station
	?g <- (goal (id ?goal-id) (class ?class&PAY-FOR-RINGS-WITH-BASE)
	                          (mode SELECTED) (parent ?parent)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   $?))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(domain-fact (name at) (param-values ?robot ?curr-location ?curr-side))
	(domain-fact (name mps-type) (param-values ?bs BS))
	(wm-fact (key refbox team-color) (value ?team-color))
	(domain-fact (name mps-team) (param-values ?bs ?team-color))
	(domain-fact (name wp-unused) (param-values ?wp))
	(domain-fact (name wp-base-color) (param-values ?wp BASE_NONE))

	(domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))
	(domain-fact (name rs-filled-with) (param-values ?target-mps ?rs-before))
	=>
	(bind ?wp-loc ?bs)
	(bind ?wp-side INPUT)
  (do-for-fact ((?g goal)) (and (eq ?g:class INSTRUCT-BS-DISPENSE-BASE) (eq ?g:mode FORMULATED) (member$ ?wp ?g:params))
    (bind  ?wp-side (get-param-by-arg ?g:params target-side))
  )
  (do-for-fact ((?g goal)) (and (eq ?g:class INSTRUCT-BS-DISPANSE-BASE) (eq ?g:mode FORMULATED) (member$ ?wp ?g:params))
    (bind  ?wp-side (get-param-by-arg ?g:params target-mps))
  )
	(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
		(create$ ; only last statement of if is returned
			(plan-assert-move-wait-for-wp ?robot ?curr-location ?curr-side ?wp-loc ?wp-side ?wp
				(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side (get-wp-complexity ?wp))
				(plan-assert-action wp-check ?robot ?wp ?wp-loc ?wp-side ABSENT)
			)
			(plan-assert-move ?robot ?wp-loc ?wp-side ?target-mps INPUT
				(plan-assert-action wp-put-slide-cc ?robot
				 ?wp ?target-mps ?rs-before ?rs-after)
			)
			(plan-assert-action wait-for-mps ?robot ?wp ?target-mps INPUT)
		)
	)
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-pay-for-rings-with-base
	?g <- (goal (id ?goal-id) (class ?class&PAY-FOR-RINGS-WITH-BASE)
	                          (mode SELECTED) (parent ?parent)
	                          (params  wp ?wp
	                                   target-mps ?target-mps
	                                   $?))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(domain-fact (name at) (param-values ?robot ?curr-location ?curr-side))
	(domain-fact (name wp-at) (param-values ?wp ?wp-loc ?wp-side))
	(domain-fact (name mps-type) (param-values ?bs BS))
	(wm-fact (key refbox team-color) (value ?team-color))
	(domain-fact (name mps-team) (param-values ?bs ?team-color))
	(domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))
	(domain-fact (name rs-filled-with) (param-values ?target-mps ?rs-before))
	=>
	(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
		(create$ ; only last statement of if is returned
			(plan-assert-move-wait-for-wp ?robot ?curr-location ?curr-side ?wp-loc ?wp-side ?wp
				(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side (get-wp-complexity ?wp))
				(plan-assert-action wp-check ?robot ?wp ?wp-loc ?wp-side ABSENT)
			)
			(plan-assert-move ?robot ?wp-loc ?wp-side ?target-mps INPUT
				(plan-assert-action wp-put-slide-cc ?robot
				 ?wp ?target-mps ?rs-before ?rs-after)
			)
			(plan-assert-action wait-for-mps ?robot ?wp ?target-mps INPUT)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-get-shelf-to-fill-rs-holding
	 ?g <- (goal (id ?goal-id) (class ?class&PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	             (mode SELECTED) (parent ?parent)
	             (params target-mps ?target-mps;rs
	                     cap-color ?cap-color
	                     $?))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(domain-fact (name at) (param-values ?robot ?curr-location ?curr-side))
	(domain-fact (name holding) (param-values ?robot ?wp))

	(domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))
	(domain-fact (name rs-filled-with) (param-values ?target-mps ?rs-before))
	 =>
	(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-move ?robot ?curr-location ?curr-side ?target-mps INPUT
			(plan-assert-action wp-put-slide-cc ?robot ?wp ?target-mps ?rs-before ?rs-after)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-get-shelf-to-fill-rs
	 ?g <- (goal (id ?goal-id) (class ?class&PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
	             (mode SELECTED) (parent ?parent)
	             (params target-mps ?target-mps;rs
	                     cap-color ?cap-color
	                     $?))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(domain-fact (name at) (param-values ?robot ?curr-location ?curr-side))
	(domain-fact (name can-hold) (param-values ?robot))
	(domain-fact (name wp-on-shelf) (param-values ?wp ?wp-loc ?shelf-spot))
	(domain-fact (name wp-cap-color) (param-values ?wp ?cap-color))

	(domain-fact (name rs-inc) (param-values ?rs-before ?rs-after))
	(domain-fact (name rs-filled-with) (param-values ?target-mps ?rs-before))
	 =>
	(plan-assert-sequential (sym-cat ?class -PLAN- (gensym*)) ?goal-id ?robot
		(create$ ; only last statement of if is returned
			(plan-assert-move ?robot ?curr-location ?curr-side ?wp-loc INPUT
				(plan-assert-action wp-get-shelf ?robot ?wp ?wp-loc ?shelf-spot)
			)
			(plan-assert-move ?robot  ?wp-loc INPUT ?target-mps INPUT
				(plan-assert-action wp-put-slide-cc ?robot ?wp ?target-mps ?rs-before ?rs-after)
			)
		)
	)
	(modify ?g (mode EXPANDED))
)


; ----------------------- MPS Instruction GOALS -------------------------------

(defrule goal-expander-instruct-cs-buffer-cap
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP) (mode SELECTED)
	            (params target-mps ?mps cap-color ?cap-color))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact wp-at args? wp ?cap-carrier m ?mps side INPUT))
	=>
	(plan-assert-sequential INSTRUCT-TO-BUFFER-CAP-PLAN ?goal-id ?robot
		(plan-assert-action prepare-cs ?mps RETRIEVE_CAP)
		(plan-assert-action cs-retrieve-cap ?mps ?cap-carrier ?cap-color)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-cs-mount-cap
	?g <- (goal (id ?goal-id) (class INSTRUCT-CS-MOUNT-CAP) (mode SELECTED)
	            (params target-mps ?mps cap-color ?cap-color))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
	=>
	(plan-assert-sequential INSTRUCT-TO-MOUNT-CAP-PLAN ?goal-id ?robot
		(plan-assert-action prepare-cs ?mps MOUNT_CAP)
		(plan-assert-action cs-mount-cap ?mps ?wp ?cap-color)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-bs-dispense-base
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class INSTRUCT-BS-DISPENSE-BASE) (mode SELECTED)
	            (params wp ?wp target-mps ?mps  target-side ?side base-color ?base-color))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	=>
	(plan-assert-sequential INSTRUCT-BS-DISPENSE-BASE-PLAN ?goal-id ?robot
		(plan-assert-action prepare-bs ?mps ?side ?base-color)
		(plan-assert-action bs-dispense ?mps ?side ?wp ?base-color)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-ds-deliver
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DELIVER) (mode SELECTED)
	            (params wp ?wp target-mps ?mps))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
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
	(plan-assert-sequential INSTRUCT-DS-DELIVER-PLAN ?goal-id ?robot
		(plan-assert-action prepare-ds ?mps ?order)
		(plan-assert-action (sym-cat fulfill-order- (lowcase ?complexity)) ?params)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-ds-discard
	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DISCARD) (mode SELECTED)
	            (params wp ?wp target-mps ?mps))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	=>
	(plan-assert-sequential INSTRUCT-DS-DISCARD-PLAN ?goal-id ?robot
		(plan-assert-action prepare-ds ?mps O0)
		(plan-assert-action fulfill-order-discard (create$ O0 ?wp ?mps))
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-navigation-challenge-move
	?g <- (goal (id ?goal-id) (class NAVIGATION-CHALLENGE-MOVE) (mode SELECTED)
				(params zone ?zone))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(plan-assert-sequential NAVIGATION-CHALLENGE-MOVE-PLAN ?goal-id ?robot
		(plan-assert-action go-wait
			?robot ?curr-location ?curr-side ?zone)
		(plan-assert-action wait-for-reached
			?robot ?zone) ; wait there until refbox confirmation
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-rs-mount-ring
	?g <- (goal (id ?goal-id) (class INSTRUCT-RS-MOUNT-RING) (mode SELECTED)
	            (params target-mps ?mps
	                    ring-color ?ring-color))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
	(wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring-color rn ?req))
	(wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before))
	(wm-fact (key domain fact rs-sub args? minuend ?rs-before subtrahend
	                                       ?req difference ?rs-after))
	(wm-fact (key wp meta next-step args? wp ?wp) (value ?step&RING1|RING2|RING3))
	=>
	(bind ?num (string-to-field ( sub-string 5 5 ?step) ))
	(bind ?prev-rings (create$ ))
	(loop-for-count (?count 1 (- ?num 1))
	   (do-for-fact ((?ring wm-fact))
	      (and (wm-key-prefix ?ring:key (create$ domain fact (sym-cat wp-ring ?count -color)))
	           (eq (wm-key-arg ?ring:key wp) ?wp))
	      (bind ?prev-rings (append$ ?prev-rings (wm-key-arg ?ring:key col)))
	))
	(plan-assert-sequential INSTRUCT-TO-MOUNT-RING-PLAN ?goal-id ?robot
		(plan-assert-action prepare-rs
		      ?mps ?ring-color ?rs-before ?rs-after ?req )
		(plan-assert-action
		      (sym-cat rs-mount-ring (sub-string 5 5 ?step) )
		      ?mps ?wp ?ring-color ?prev-rings ?rs-before ?rs-after ?req )
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-deliver-rc21
	?g <- (goal (id ?goal-id) (class DELIVER-RC21)
	                          (mode SELECTED) (parent ?parent)
	                          (params  wp ?wp))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	(wm-fact (key refbox team-color) (value ?team-color))
	(wm-fact (key config rcll challenge-flip-insertion) (value ?flip))
	=>
	(bind ?dropzone M-INS-OUT)
	(if (or (and (eq ?team-color CYAN) (neq ?flip TRUE))
	        (and (eq ?team-color MAGENTA) (eq ?flip TRUE))) then
		(bind ?dropzone C-INS-OUT)
	)

	(plan-assert-sequential (sym-cat DELIVER-RC21-PLAN- (gensym*)) ?goal-id ?robot
		(if (not (is-holding ?robot ?wp))
		then
			(bind ?wp-loc nil)
			(bind ?wp-side nil)

			(do-for-fact ((?wp-at wm-fact))
						(and (wm-key-prefix ?wp-at:key (create$ domain fact wp-at))
							(eq (wm-key-arg ?wp-at:key wp) ?wp))
						(bind ?wp-loc (wm-key-arg ?wp-at:key m))
						(bind ?wp-side (wm-key-arg ?wp-at:key side)))

			(create$
				(plan-assert-move-wait-for-wp ?robot ?curr-location ?curr-side ?wp-loc ?wp-side ?wp
					(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side (get-wp-complexity ?wp))
					(plan-assert-action wp-check ?robot ?wp ?wp-loc ?wp-side ABSENT)
				)
				(plan-assert-action go-wait
					?robot (wait-pos ?wp-loc ?wp-side) WAIT ?dropzone)
				(plan-assert-action wp-discard ?robot ?wp)
			)
		else
			(create$
				(plan-assert-action go-wait
					?robot ?curr-location ?curr-side ?dropzone)
				(plan-assert-action wp-discard ?robot ?wp)
			)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-wait-nothing-executable
	?g <- (goal (id ?goal-id) (class WAIT-NOTHING-EXECUTABLE) (mode SELECTED))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(plan-assert-sequential WAIT-NOTHING-EXECUTABLE- ?goal-id ?robot
		(plan-assert-action wait ?robot ?curr-location ?curr-side)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-exploration-challenge-move
	?g <- (goal (id ?goal-id) (class EXPLORATION-MOVE) (mode SELECTED)
	            (params zone ?zone))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(plan-assert-sequential EXPLORATION-MOVE-PLAN ?goal-id ?robot
		(plan-assert-action explore-and-turn
			?robot ?curr-location ?curr-side ?zone)
		(plan-assert-action wait ?robot ?zone WAIT)
	)
	(modify ?g (mode EXPANDED))
)
