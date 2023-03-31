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
	(bind ?plan-id (sym-cat ?plan-name - (gensym*)))
	(assert (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL)))
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

(defrule goal-expander-enter-field
	?g <- (goal (id ?goal-id) (mode SELECTED) (class ENTER-FIELD)
	            (params team-color ?team-color))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	=>
	(plan-assert-sequential ENTER-FIELD-PLAN ?goal-id ?robot
		(plan-assert-action enter-field ?robot ?team-color)
	)
	(modify ?g (mode EXPANDED))
)



;----------------------------------------------------------------------
(defrule goal-expander-buffer-cap-goal
	?g <- (goal (id ?goal-id) (class BUFFER-CAP-GOAL) (mode SELECTED) (parent ?parent)
	            (params target-cs ?cs cc ?cc))
	?m <-(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	; ccg with color 
	(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	=>
	(bind ?shelf-spot nil)
	(do-for-fact ((?wp-on-shelf wm-fact)) (and (wm-key-prefix ?wp-on-shelf:key (create$ domain fact wp-on-shelf))
	                                      (eq (wm-key-arg ?wp-on-shelf:key wp) ?cc)
	                                      (eq (wm-key-arg ?wp-on-shelf:key m) ?cs))
		(bind ?shelf-spot (wm-key-arg ?wp-on-shelf:key spot)))
	(plan-assert-sequential BUFFER-CAP-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?cs INPUT
			(plan-assert-action wp-get-shelf ?robot ?cc ?cs ?shelf-spot)
			(plan-assert-action wp-put ?robot ?cc ?cs INPUT)
			(plan-assert-action prepare-cs ?cs RETRIEVE_CAP)
			(plan-assert-action cs-retrieve-cap ?cs ?cc ?cap-color)))
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-discard
	?g <- (goal (id ?goal-id) (class DISCARD-GOAL) (mode SELECTED)
	            (params target-cs ?cs cc ?cc))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	=>
	(plan-assert-sequential BUFFER-CAP-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?cs OUTPUT
			(plan-assert-action wp-get ?robot ?cc ?cs OUTPUT)
			(plan-assert-action wp-discard ?robot ?cc)))
	(modify ?g (mode EXPANDED))
)

; ----------------------- MPS Instruction GOALS -------------------------------
; debug this mount expander            1
(defrule goal-expander-instruct-cs-mount-cap
	?g <- (goal (id ?goal-id) (class MOUNT-CAP-GOAL) (mode SELECTED)
	            (params target-mps ?mps cap-color ?cap-color wp ?wp))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
;	(wm-fact (key domain fact holding args? robot ?robot workpiece ?wp))
	; fact: cap mounted
	; (wm-fact (key domain fact cs-buffered args? mps ?mps cap-color ?cap-color))

	=>
	(plan-assert-sequential INSTRUCT-TO-MOUNT-CAP-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?mps INPUT
			(plan-assert-action wp-put ?robot ?wp ?mps INPUT)
			(plan-assert-action prepare-cs ?mps MOUNT_CAP)
			; wp cc
			(plan-assert-action cs-mount-cap ?mps ?wp ?cap-color)))
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-go-get-mounted-base
	?g <- (goal (id ?goal-id) (class GET-MOUNTED-BASE-GOAL) (mode SELECTED)
	            (params wp ?wp target-mps ?mps))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	=>
	(plan-assert-sequential INSTRUCT-TO-MOUNT-CAP-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?mps OUTPUT
			(plan-assert-action wp-get ?robot ?wp ?mps OUTPUT)))
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-go-get-ring-mounted-base
	?g <- (goal (id ?goal-id) (class GET-RING-MOUNTED-BASE-GOAL) (mode SELECTED)
	            (params wp ?wp target-mps ?mps))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side OUTPUT))
	=>
	(plan-assert-sequential INSTRUCT-TO-GET-MOUNTED-BASE-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?mps OUTPUT
			(plan-assert-action wp-get ?robot ?wp ?mps OUTPUT)))
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-bs-dispense-base
	; ?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class PRE-GET-BASE-GOAL) (mode SELECTED)
	            (params wp ?wp target-mps ?mps  target-side ?side base-color ?base-color))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	=>
	(plan-assert-sequential PRE-GET-BASE-GOAL-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?mps ?side
			(plan-assert-action prepare-bs ?mps ?side ?base-color)
			(plan-assert-action bs-dispense ?mps ?side ?wp ?base-color)
			(plan-assert-action wp-get ?robot ?wp ?mps ?side)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-ds-deliver
	;?p <- (goal (mode DISPATCHED) (id ?parent))
	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DELIVER) (mode SELECTED)
	            (params wp ?wp target-mps ?mps))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))

	(wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
	(wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
	; Order-CEs
	; (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key domain fact order-complexity args? ord ?order com C1))
	(wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
	(wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
	(wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
	(wm-fact (key domain fact order-gate args? ord ?order gate ?gate))

	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))

	=>
	(plan-assert-sequential INSTRUCT-DS-DELIVER-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?mps INPUT
			(plan-assert-action wp-put ?robot ?wp ?mps INPUT)
			(plan-assert-action prepare-ds ?mps ?order)
			(plan-assert-action fulfill-order-c1 ?order ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color)
	))
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-instruct-rs-mount-ring
	?g <- (goal (id ?goal-id) (class INSTRUCT-RS-MOUNT-RING) (mode SELECTED)
	            (params target-mps ?mps
	                    ring-color ?ring-color wp ?wp))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	(wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring-color rn ?req))
	(wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before))
	(wm-fact (key domain fact rs-sub args? minuend ?rs-before subtrahend
	                                       ?req difference ?rs-after))
	;(wm-fact (key wp meta next-step args? wp ?wp) (value ?step&RING1|RING2|RING3))
	=>
	;(bing ?rs-after (int-to-sym (- (sym-to-int ?rs-before) (sym-to-int ?req))))
	(plan-assert-sequential INSTRUCT-TO-MOUNT-RING-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?mps INPUT
			(plan-assert-action wp-put ?robot ?wp ?mps INPUT)
			(plan-assert-action prepare-rs
				?mps ?ring-color ?rs-before ?rs-after ?req )
			(plan-assert-action rs-mount-ring1 ?mps ?wp ?ring-color ?rs-before ?rs-after ?req )))
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-rs-payment
	?g <- (goal (id ?goal-id) (class RS-PAYMENT) (mode SELECTED)
	            (params rs ?rs cs ?cs bs ?bs
	                    ring-color ?ring-color))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact wp-on-shelf args?  wp ?cc m ?cs spot ?cc-spot))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	(wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring-color rn ?req))
	(wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before))
	=>
	
	(bind ?rs-after1 (int-to-sym (+ (sym-to-int ?rs-before) 1)))
	(if (eq ?req TWO)
	then
		(bind ?rs-after2 (int-to-sym (+ (sym-to-int ?rs-before) 2)))
	 	(bind ?wp-for-pyament(sym-cat AB- (gensym*)) )
	 	(plan-assert-sequential RING-PAYMENT-TWO ?goal-id ?robot
			(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?cs INPUT
				(plan-assert-action wp-get-shelf ?robot ?cc ?cs ?cc-spot)
				(plan-assert-action move ?robot ?cs INPUT ?rs INPUT)	
				(plan-assert-action wp-put-slide-cc ?robot ?cc ?rs ?rs-before ?rs-after1)

				(plan-assert-action spawn-wp ?wp-for-pyament ?robot)
				(plan-assert-action prepare-bs ?bs OUTPUT BASE_CLEAR)
				(plan-assert-action bs-dispense ?bs OUTPUT ?wp-for-pyament  BASE_CLEAR)
				(plan-assert-action move ?robot ?rs INPUT ?bs OUTPUT)
				(plan-assert-action wp-get ?robot ?wp-for-pyament  ?bs OUTPUT)
				(plan-assert-action move ?robot ?bs OUTPUT ?rs INPUT)	
				(plan-assert-action wp-put-slide-cc ?robot ?wp-for-pyament ?rs-after1 ?rs-after2)
			)
	 	)
	 else
		(if (eq ?req ONE)
		then
			(plan-assert-sequential RING-PAYMENT-ONE ?goal-id ?robot
				(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?cs INPUT
					(plan-assert-action wp-get-shelf ?robot ?cc ?cs ?cc-spot)
					(plan-assert-action move ?robot ?cs INPUT ?rs INPUT)	
					(plan-assert-action wp-put-slide-cc ?robot ?cc ?rs ?rs-before ?rs-after1)
				)
			)
		 else
			 (plan-assert-safe-move ?robot ?curr-loc ?curr-side START INPUT)
	 	)
	 )
	(modify ?g (mode RETRACTED))
)

(defrule goal-expander-c1-order
	?g <- (goal (id ?goal-id) (mode FORMULATED) (class C1-ORDER))
	=>
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-base-cap-ready

	?g <- (goal (id ?goal-id) (mode FORMULATED) (class BASE-CAP-READY))
	=>
	(modify ?g (mode COMMITTED))
)

(defrule goal-expander-mount-ring-then-get-wp
	?g <- (goal (id ?goal-id) (mode FORMULATED) (class MOUNT-RING-THEN-GET-WP-GOAL))
	=>
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-mount-cap-then-get-wp
	?g <- (goal (id ?goal-id) (mode FORMULATED) (class MOUNT-CAP-THEN-GET-WP-GOAL))
	=>
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-buffer-cap-discard

	?g <- (goal (id ?goal-id) (mode FORMULATED) (class BUFFER-CAP-DISCARD-GOAL))
	=>
	(modify ?g (mode EXPANDED))
)
