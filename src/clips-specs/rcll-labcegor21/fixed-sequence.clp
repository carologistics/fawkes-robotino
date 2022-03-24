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

(defrule goal-expander-transport
	?g <- (goal (id ?goal-id) (class TRANSPORT) (mode SELECTED) (parent ?parent)
	            (params wp ?wp wp-next-step ?wp-step dst-mps ?dst-mps dst-side ?dst-side))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	
	(wm-fact (key domain fact wp-at args? wp ?wp m ?src-mps side ?src-side))
	(wm-fact (key wp meta next-step args? wp ?wp) (value ?wp-step))
	
	(not (wm-fact (key domain fact wp-at args? wp ? m ?dst-mps side ?dst-side)))
	=>
	(plan-assert-sequential (sym-cat TRANSPORT-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-location ?curr-side ?src-mps ?src-side
			(plan-assert-action wp-get ?robot ?wp ?src-mps ?src-side)
		)
		(plan-assert-safe-move ?robot (wait-pos ?src-mps ?src-side) WAIT ?dst-mps ?dst-side
			(plan-assert-action wp-put ?robot ?wp ?dst-mps ?dst-side)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-transport-create
	?g <- (goal (id ?goal-id) (class TRANSPORT) (mode SELECTED) (parent ?parent)
	            (params wp ?wp wp-next-step ?wp-step dst-mps ?dst-mps dst-side ?dst-side))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	
	(wm-fact (key domain fact wp-base-color args? wp ?wp col BASE_NONE))
	(wm-fact (key wp meta next-step args? wp ?wp) (value ?wp-step))
	
	(not (wm-fact (key domain fact wp-at args? wp ? m ?dst-mps side ?dst-side)))

	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
	=>
	(plan-assert-sequential (sym-cat TRANSPORT- (gensym*)) ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-location ?curr-side C-BS OUTPUT
			(plan-assert-action lock-mps C-BS)
			(plan-assert-action prepare-bs C-BS OUTPUT ?base-color)
			(plan-assert-action bs-dispense C-BS OUTPUT ?wp ?base-color)
			(plan-assert-action wp-get ?robot ?wp C-BS OUTPUT)
			(plan-assert-action unlock-mps C-BS)
		)
		(plan-assert-safe-move ?robot (wait-pos C-BS OUTPUT) WAIT ?dst-mps ?dst-side
			(plan-assert-action wp-put ?robot ?wp ?dst-mps ?dst-side)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-buffer-cap
" Feed a CS with a cap from its shelf so that afterwards
   it can directly put the cap on a product."
	?g <- (goal (id ?goal-id) (class BUFFER-CAP) (mode SELECTED) (parent ?parent)
	            (params target-mps ?mps cap-color ?cap-color ))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps $?))
	(not (wm-fact (key domain fact holding args? r ?robot $?)))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	=>
	(bind ?shelf-spot nil)
	(do-for-fact ((?wp-on-shelf wm-fact)) (and (wm-key-prefix ?wp-on-shelf:key (create$ domain fact wp-on-shelf))
		                                      (eq (wm-key-arg ?wp-on-shelf:key wp) ?cc)
		                                      (eq (wm-key-arg ?wp-on-shelf:key m) ?mps))
			(bind ?shelf-spot (wm-key-arg ?wp-on-shelf:key spot))
	)

	(plan-assert-sequential (sym-cat BUFFER-CAP-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-location ?curr-side ?mps INPUT
			(plan-assert-action wp-get-shelf ?robot ?cc ?mps ?shelf-spot)
			(plan-assert-action wp-put ?robot ?cc ?mps INPUT)
			(plan-assert-action prepare-cs ?mps RETRIEVE_CAP)
			(plan-assert-action cs-retrieve-cap ?mps ?cc ?cap-color)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-discard
" Pick up a product and discard it."
	?g <- (goal (id ?goal-id) (class DISCARD) (mode SELECTED) (parent ?parent) (params wp ?wp))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?wp-loc side ?wp-side))
	=>
	(plan-assert-sequential (sym-cat DISCARD-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-location ?curr-side ?wp-loc ?wp-side
			(plan-assert-action wp-get ?robot ?wp ?wp-loc ?wp-side)
		)
		(plan-assert-action wp-discard ?robot ?wp)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-mount-cap
	?g <- (goal (id ?goal-id) (class MOUNT-CAP) (mode SELECTED) (parent ?parent)
	            (params wp ?wp cap-mps ?cap-mps cap-color ?cap-color))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	=>
	(plan-assert-sequential (sym-cat MOUNT-CAP-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-action prepare-cs ?cap-mps MOUNT_CAP)
		(plan-assert-action cs-mount-cap ?cap-mps ?wp ?cap-color)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-mount-ring
	?g <- (goal (id ?goal-id) (class MOUNT-RING) (mode SELECTED) (parent ?parent)
	            (params wp ?wp ring-mps ?ring-mps ring-color ?ring-color ring-nr ?ring-nr))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact rs-ring-spec args? m ?ring-mps r ?ring-color rn ?req))
	(wm-fact (key domain fact rs-filled-with args? m ?ring-mps n ?rs-before))
	(wm-fact (key domain fact rs-sub args? minuend ?rs-before subtrahend ?req difference ?rs-after))
	=>
	(bind ?prev-rings (create$ ))
	(loop-for-count (?count 1 (- ?ring-nr 1))
	   (do-for-fact ((?ring wm-fact))
	      (and (wm-key-prefix ?ring:key (create$ domain fact (sym-cat wp-ring ?count -color)))
	           (eq (wm-key-arg ?ring:key wp) ?wp))
	      (bind ?prev-rings (append$ ?prev-rings (wm-key-arg ?ring:key col)))
	))
	(plan-assert-sequential (sym-cat MOUNT-RING-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-action prepare-rs ?ring-mps ?ring-color ?rs-before ?rs-after ?req)
		(plan-assert-action (sym-cat rs-mount-ring ?ring-nr)
		      ?ring-mps ?wp ?ring-color ?prev-rings ?rs-before ?rs-after ?req )
	)
	(modify ?g (mode EXPANDED))
)

(defrule update-prepare-rs-count
	"Ensure the prepare-rs action always has the correct counts"
	?action <- (plan-action (action-name prepare-rs)
							(param-values ?mps ?color ?before ?after ?req)
							(state FORMULATED))
	(wm-fact (key domain fact rs-filled-with args? m ?mps n ?actual-before&~?before))
	(wm-fact (key domain fact rs-sub args? minuend ?actual-before subtrahend ?req difference ?actual-after))
	=>
	(modify ?action (param-values ?mps ?color ?actual-before ?actual-after ?req))
	(printout t "Update ring count for prepare-rs" crlf)
)

(defrule update-mount-ring-count
	"Ensure the  rs-mount-ring action always has the correct counts."
	?action <- (plan-action (action-name rs-mount-ring1|rs-mount-ring2|rs-mount-ring3)
							(param-values ?mps ?wp ?color $?prev-rings ?before ?after ?req)
							(state FORMULATED))
	(wm-fact (key domain fact rs-filled-with args? m ?mps n ?actual-before&~?before))
	(wm-fact (key domain fact rs-sub args? minuend ?actual-before subtrahend ?req difference ?actual-after))
	=>
	(modify ?action (param-values ?mps ?wp ?color $?prev-rings ?actual-before ?actual-after ?req))
	(printout t "Update ring count for rs-mount-ring" crlf)
)

(defrule goal-expander-pay-ring
" Retrieve a workpiece and use it to pay for a ring."
	?g <- (goal (id ?goal-id) (class PAY-RING) (mode SELECTED) (parent ?parent)
	            (params wp ?wp src-mps ?src-mps ring-mps ?ring-mps))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-location side ?curr-side))
	(wm-fact (key domain fact rs-inc args? summand ?rs-before sum ?rs-after))
	(wm-fact (key domain fact rs-filled-with args? m ?ring-mps n ?rs-before))
	(wm-fact (key domain fact mps-type args? m ?src-mps t ?src-type))
	=>
	(plan-assert-sequential (sym-cat PAY-RING-PLAN- (gensym*)) ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-location ?curr-side ?src-mps OUTPUT
			(if (eq ?src-type BS) then
				(create$ (plan-assert-action lock-mps ?src-mps)
				         (plan-assert-action prepare-bs ?src-mps OUTPUT BASE_RED)
						 (plan-assert-action bs-dispense ?src-mps OUTPUT ?wp BASE_RED)
				)
			)
			(plan-assert-action wp-get ?robot ?wp ?src-mps OUTPUT)
			(if (eq ?src-type BS) then
				(plan-assert-action unlock-mps ?src-mps)
			)
		)
		(plan-assert-safe-move ?robot (wait-pos ?src-mps OUTPUT) WAIT ?ring-mps INPUT
			(plan-assert-action wp-put-slide-cc ?robot ?wp ?ring-mps ?rs-before ?rs-after)
		)
	)
	(modify ?g (mode EXPANDED))
)

(defrule update-put-slide-cc-count
	"Ensure the wp-put-slide-cc action always has the correct counts."
	?action <- (plan-action (action-name wp-put-slide-cc) (param-values ?robot ?wp ?mps ?before ?after)
							(state FORMULATED))
	(wm-fact (key domain fact rs-filled-with args? m ?mps n ?actual-before))
	(wm-fact (key domain fact rs-inc args? summand ?actual-before sum ?actual-after))
	(test (neq ?actual-before ?before))
	=>
	(modify ?action (param-values ?robot ?wp ?mps ?actual-before ?actual-after))
	(printout t "Update ring count for payment " ?before " -> " ?actual-before crlf)
)

(defrule goal-expander-deliver
	?g <- (goal (id ?goal-id) (class DELIVER) (mode SELECTED)
	            (params wp ?wp mps ?mps))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	; WP-CES
	(wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
	(wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
	(wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
	(wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
	(wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
	(wm-fact (key domain fact wp-at args? wp ?wp m ?wp-mps side ?wp-side))
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
	(plan-assert-sequential DELIVER-PLAN ?goal-id ?robot
		(plan-assert-action prepare-ds ?mps ?order)
		(plan-assert-action (sym-cat fulfill-order- (lowcase ?complexity)) ?params)
	)
	(modify ?g (mode EXPANDED))
)


