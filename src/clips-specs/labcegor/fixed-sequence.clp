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


(defrule goal-expander-test-goal-1
" Moves the robot to the output of the given mps."
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode SELECTED) (parent ?parent)
	            (params bs ?bs base-color ?bs-color workpiece ?wp))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	=>
	(bind ?shelf-spot nil)
	(do-for-fact ((?wp-on-shelf wm-fact)) (and (wm-key-prefix ?wp-on-shelf:key (create$ domain fact wp-on-shelf))
	                                      (eq (wm-key-arg ?wp-on-shelf:key wp) ?cc)
	                                      (eq (wm-key-arg ?wp-on-shelf:key m) ?cs))
		(bind ?shelf-spot (wm-key-arg ?wp-on-shelf:key spot))
	)
	(plan-assert-sequential BUFFER-CAP-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?bs INPUT
			(plan-assert-action prepare-bs ?bs INPUT ?bs-color)
			(plan-assert-action bs-dispense ?bs OUTPUT ?wp ?bs-color)
		)
	)
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-test-goal-2
" Moves the robot to the output of the given mps."
	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode SELECTED) (parent ?parent)
	            (params target-cs ?cs cc ?cc))
	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
	(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
	=>
	(bind ?shelf-spot nil)
	(do-for-fact ((?wp-on-shelf wm-fact)) (and (wm-key-prefix ?wp-on-shelf:key (create$ domain fact wp-on-shelf))
	                                      (eq (wm-key-arg ?wp-on-shelf:key wp) ?cc)
	                                      (eq (wm-key-arg ?wp-on-shelf:key m) ?cs))
		(bind ?shelf-spot (wm-key-arg ?wp-on-shelf:key spot))
	)
	(plan-assert-sequential BUFFER-CAP-PLAN ?goal-id ?robot
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?cs INPUT
			(plan-assert-action wp-get-shelf ?robot ?cc ?cs ?shelf-spot)
			(plan-assert-action wp-put ?robot ?cc ?cs INPUT)
			(plan-assert-action prepare-cs ?cs RETRIEVE_CAP)
			(plan-assert-action cs-retrieve-cap ?cs ?cc ?cap-color)
		)
	)
	(modify ?g (mode EXPANDED))
)


; ----------------------- MPS Instruction GOALS -------------------------------

;(defrule goal-expander-instruct-cs-buffer-cap
;	?g <- (goal (id ?goal-id) (class INSTRUCT-CS-BUFFER-CAP) (mode SELECTED)
;	            (params target-mps ?mps cap-color ?cap-color))
;	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;	(wm-fact (key domain fact wp-at args? wp ?cap-carrier m ?mps side INPUT))
;	=>
;	(plan-assert-sequential INSTRUCT-TO-BUFFER-CAP-PLAN ?goal-id ?robot
;		(plan-assert-action prepare-cs ?mps RETRIEVE_CAP)
;		(plan-assert-action cs-retrieve-cap ?mps ?cap-carrier ?cap-color)
;	)
;	(modify ?g (mode EXPANDED))
;)
;
;(defrule goal-expander-instruct-cs-mount-cap
;	?g <- (goal (id ?goal-id) (class INSTRUCT-CS-MOUNT-CAP) (mode SELECTED)
;	            (params target-mps ?mps cap-color ?cap-color))
;	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
;	=>
;	(plan-assert-sequential INSTRUCT-TO-MOUNT-CAP-PLAN ?goal-id ?robot
;		(plan-assert-action prepare-cs ?mps MOUNT_CAP)
;		(plan-assert-action cs-mount-cap ?mps ?wp ?cap-color)
;	)
;	(modify ?g (mode EXPANDED))
;)
;
;(defrule goal-expander-instruct-bs-dispense-base
;	;?p <- (goal (mode DISPATCHED) (id ?parent))
;	?g <- (goal (id ?goal-id) (class INSTRUCT-BS-DISPENSE-BASE) (mode SELECTED)
;	            (params wp ?wp target-mps ?mps  target-side ?side base-color ?base-color))
;	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;	=>
;	(plan-assert-sequential INSTRUCT-BS-DISPENSE-BASE-PLAN ?goal-id ?robot
;		(plan-assert-action prepare-bs ?mps ?side ?base-color)
;		(plan-assert-action bs-dispense ?mps ?side ?wp ?base-color)
;	)
;	(modify ?g (mode EXPANDED))
;)
;
;(defrule goal-expander-instruct-ds-deliver
;	;?p <- (goal (mode DISPATCHED) (id ?parent))
;	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DELIVER) (mode SELECTED)
;	            (params wp ?wp target-mps ?mps))
;	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;	(wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
;	(wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
;	(wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
;	(wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
;	(wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
;	; Order-CEs
;	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
;	(wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
;	(wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
;	(wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
;	(wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
;	(wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
;	(wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
;	(wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
;	=>
;	(bind ?params (create$))
;	(switch ?complexity
;		(case C0 then
;		    (bind ?params (create$ ?order ?wp ?mps ?gate ?base-color ?cap-color)))
;		(case C1 then
;		    (bind ?params (create$ ?order ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color)))
;		(case C2 then
;		    (bind ?params (create$ ?order ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color ?ring2-color)))
;		(case C3 then
;		    (bind ?params (create$ ?order ?wp ?mps ?gate ?base-color ?cap-color ?ring1-color ?ring2-color ?ring3-color)))
; )
;	(plan-assert-sequential INSTRUCT-DS-DELIVER-PLAN ?goal-id ?robot
;		(plan-assert-action prepare-ds ?mps ?order)
;		(plan-assert-action (sym-cat fulfill-order- (lowcase ?complexity)) ?params)
;	)
;	(modify ?g (mode EXPANDED))
;)

;(defrule goal-expander-instruct-rs-mount-ring
;	?g <- (goal (id ?goal-id) (class INSTRUCT-RS-MOUNT-RING) (mode SELECTED)
;	            (params target-mps ?mps
;	                    ring-color ?ring-color))
;	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;	(wm-fact (key domain fact wp-at args? wp ?wp m ?mps side INPUT))
;	(wm-fact (key domain fact rs-ring-spec args? m ?mps r ?ring-color rn ?req))
;	(wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before))
;	(wm-fact (key domain fact rs-sub args? minuend ?rs-before subtrahend
;	                                       ?req difference ?rs-after))
;	(wm-fact (key wp meta next-step args? wp ?wp) (value ?step&RING1|RING2|RING3))
;	=>
;	(bind ?num (string-to-field ( sub-string 5 5 ?step) ))
;	(bind ?prev-rings (create$ ))
;	(loop-for-count (?count 1 (- ?num 1))
;	   (do-for-fact ((?ring wm-fact))
;	      (and (wm-key-prefix ?ring:key (create$ domain fact (sym-cat wp-ring ?count -color)))
;	           (eq (wm-key-arg ?ring:key wp) ?wp))
;	      (bind ?prev-rings (append$ ?prev-rings (wm-key-arg ?ring:key col)))
;	))
;	(plan-assert-sequential INSTRUCT-TO-MOUNT-RING-PLAN ?goal-id ?robot
;		(plan-assert-action prepare-rs
;		      ?mps ?ring-color ?rs-before ?rs-after ?req )
;		(plan-assert-action
;		      (sym-cat rs-mount-ring (sub-string 5 5 ?step) )
;		      ?mps ?wp ?ring-color ?prev-rings ?rs-before ?rs-after ?req )
;	)
;	(modify ?g (mode EXPANDED))
;)
