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
	(wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
	(wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
	(wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
	(wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
	; Order-CEs
	(wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
	(wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
	(wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
	(wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
	(wm-fact (key domain fact order-gate args? ord ?order gate ?gate))

	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))

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
		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?mps INPUT
			(plan-assert-action wp-put ?robot ?wp ?mps INPUT)
			(plan-assert-action prepare-ds ?mps ?order)
			(plan-assert-action (sym-cat fulfill-order- (lowcase ?complexity)) ?params)
	))
	(modify ?g (mode EXPANDED))
)

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


(defrule goal-expander-c0-order
	?g <- (goal (id ?goal-id) (mode FORMULATED) (class C0-ORDER)
				(params wp ?wp base-color ?base-color cap-color ?cap-color cc ?cc target-mps ?mps) 
			)
	=>

	; subgoal 1 holding Base and cap buffered
	(printout t "Goal " BASE-CAP-READY " formulated" crlf)
	(bind ?goal-id-1 (sym-cat BASE-CAP-READY- (gensym*)))
	(assert (goal (class BASE-CAP-READY)
					(id ?goal-id-1)
					(sub-type SIMPLE)
					(parent ?goal-id)
					(verbosity NOISY) (is-executable FALSE)
					(meta-template goal-meta)
					(params wp ?wp base-color ?base-color cap-color ?cap-color cc ?cc target-mps ?mps)
			))
	(assert (goal-meta (goal-id ?goal-id-1)))

	; subgoal 2 mount cap upon base
	(bind ?goal-id-2 (sym-cat MOUNT-CAP-THEN-GET-WP-GOAL- (gensym*)))
		(assert (goal (class MOUNT-CAP-THEN-GET-WP-GOAL)
					(id ?goal-id-2)
					(sub-type SIMPLE)
					(parent ?goal-id)
					(verbosity NOISY) (is-executable TRUE)
					(meta-template goal-meta)
					(params target-mps ?mps cap-color ?cap-color wp ?wp)
		))
	(assert (goal-meta (goal-id ?goal-id-2)))

	; subgoal 3 deliver
	(bind ?goal-id-3 (sym-cat INSTRUCT-DS-DELIVER- (gensym*)))
		(assert (goal (class INSTRUCT-DS-DELIVER)
					(id ?goal-id-3)
					(sub-type SIMPLE)
					(parent ?goal-id)
					(params wp ?wp target-mps C-DS)
					(verbosity NOISY) (is-executable TRUE)
					(meta-template goal-meta)
		))
	(assert (goal-meta (goal-id ?goal-id-3) (assigned-to nil)))


	(modify ?g (mode EXPANDED) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS))
)

; 1 prepare base and buffer cap
(defrule goal-expander-base-cap-ready
	?g <- (goal (id ?goal-id) (mode FORMULATED) (class BASE-CAP-READY)
				(params wp ?wp  base-color ?base-color cap-color ?cap-color cc ?cc target-mps ?mps)
			)
	=>
	; subgoal 1-1 buffer cap then discard base 
	(bind ?goal-id-1-1 (sym-cat BUFFER-CAP-DISCARD-GOAL- (gensym*)))
	(assert (goal (class BUFFER-CAP-DISCARD-GOAL)
					(id ?goal-id-1-1)
					(sub-type SIMPLE)	
					(parent ?goal-id)
					(verbosity NOISY) (is-executable TRUE)
					(meta-template goal-meta)

					; todo cc and cap color
          			(params target-cs ?mps cc ?cc)
        	))
	(assert (goal-meta (goal-id ?goal-id-1-1)))

	; leafgoal 1-2 Prepare and get base
	(bind ?goal-id-1-2 (sym-cat PRE-GET-BASE-GOAL- (gensym*)))
	(assert (goal (class PRE-GET-BASE-GOAL)
                (id ?goal-id-1-2)
                (sub-type SIMPLE)
                (parent ?goal-id)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
                (params wp ?wp target-mps C-BS target-side OUTPUT base-color ?base-color)
          ))
	(assert (goal-meta (goal-id ?goal-id-1-2) (assigned-to nil)))

	(modify ?g (mode EXPANDED) (sub-type CENTRAL-RUN-SUBGOALS-IN-PARALLEL))

)

; 2 mount-cap-then-get-wp
(defrule goal-expander-mount-cap-then-get-wp
	?g <- (goal (id ?goal-id) (mode FORMULATED) (class MOUNT-CAP-THEN-GET-WP-GOAL)
				(params target-mps ?mps cap-color ?cap-color wp ?wp)
			)
	=>
	; leafgoal 2-1 mount cap upon base
	(bind ?goal-id-2-1 (sym-cat MOUNT-CAP-GOAL- (gensym*)))
	(assert (goal (class MOUNT-CAP-GOAL)
                (id ?goal-id-2-1)
                (sub-type SIMPLE)
                (parent ?goal-id)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
                (params target-mps ?mps cap-color ?cap-color wp ?wp)
			))
	(assert (goal-meta (goal-id ?goal-id-2-1) (assigned-to nil)))

	; leafgoal 2-2 get mounted base
	(bind ?goal-id-2-2 (sym-cat GET-MOUNTED-BASE-GOAL- (gensym*)))
	(assert (goal (class GET-MOUNTED-BASE-GOAL)
                (id ?goal-id-2-2)
                (sub-type SIMPLE)
                (parent ?goal-id)
                (verbosity NOISY) (is-executable TRUE)
                (meta-template goal-meta)
	            (params wp ?wp target-mps ?mps)
			))
	(assert (goal-meta (goal-id ?goal-id-2-2) (assigned-to nil)))

	(modify ?g (mode COMMITTED) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS))
)

; 1-1 BUFFER-CAP-DISCARD-GOAL
(defrule goal-expander-BUFFER-CAP-DISCARD-GOAL
	?g <- (goal (id ?goal-id) (mode FORMULATED) (class BUFFER-CAP-DISCARD-GOAL)
						(params target-cs ?mps cc ?cc)
		)
	=>
	; leafgoal 1-1-1 buffer
 	(bind ?goal-id-1-1-1 (sym-cat BUFFER-CAP-GOAL- (gensym*)))
	(assert (goal (class BUFFER-CAP-GOAL)
					(id ?goal-id-1-1-1)
					(sub-type SIMPLE)
					(parent ?goal-id)
					(verbosity NOISY) (is-executable TRUE)
					(meta-template goal-meta)
					(params target-cs ?mps cc ?cc)
	        ))
	(assert (goal-meta (goal-id ?goal-id-1-1-1) (assigned-to robot1)))

	; leafgoal 1-1-2 discard uesless base
  	(bind ?goal-id-1-1-2 (sym-cat DISCARD-GOAL- (gensym*)))
	(assert (goal (class DISCARD-GOAL)
					(id ?goal-id-1-1-2)
					(sub-type SIMPLE)
					(parent ?goal-id)
					(verbosity NOISY) (is-executable TRUE)
					(meta-template goal-meta)
					(params target-cs ?mps cc ?cc)
          ))
	(assert (goal-meta (goal-id ?goal-id-1-1-2) (assigned-to nil)))

	(modify ?g (mode EXPANDED) (sub-type CENTRAL-RUN-ALL-OF-SUBGOALS))
)




;-----------------------------------------selector--------------------------------

(defrule goal-reasoner-mygoal-3-select
	?g <- (goal (id ?goal-id) (class INSTRUCT-DS-DELIVER) (mode FORMULATED))
	?gm <- (goal-meta (goal-id ?goal-id) (assigned-to nil))
	(goal (class GET-MOUNTED-BASE-GOAL) (mode FINISHED))
	=>
	(modify ?gm (assigned-to robot2))
	(modify ?g (mode SELECTED))
)

; goal 1-2
(defrule goal-reasoner-pre-get-base-goal-select
	?g <- (goal (id ?goal-id) (class PRE-GET-BASE-GOAL) (mode FORMULATED))
  ?gm <- (goal-meta (goal-id ?goal-id) (assigned-to nil))
  	; if avaliable robts ?
  ;(not (goal-meta (assigned-to ?robot)))
	=>
  ; assign robot
  (modify ?gm (assigned-to robot2))
	(modify ?g (mode SELECTED))
)

todo goal 1-1-1
(defrule goal-reasoner-buffer-cap-goal-select
	?g <- (goal (id ?goal-id) (class BUFFER-CAP-GOAL) (mode FORMULATED))
  ; ?gf <- (goal (class BUFFER-CAP-DISCARD-GOAL))
  =>
	(modify ?g (mode SELECTED))

)

(defrule goal-reasoner-discard-goal-select
	?g <- (goal (id ?goal-id) (class DISCARD-GOAL) (mode FORMULATED))
  	?gm <- (goal-meta (goal-id ?goal-id) (assigned-to nil))
  	(goal (class BUFFER-CAP-GOAL) (mode FINISHED))

	=>
 	 ; assign robot
  	(modify ?gm (assigned-to robot1))
	(modify ?g (mode SELECTED))
)

; goal 2-1
(defrule goal-reasoner-mount-cap-goal-select
	?g <- (goal (id ?goal-id) (class MOUNT-CAP-GOAL) (mode FORMULATED))
	?gm <- (goal-meta (goal-id ?goal-id) (assigned-to nil))
	(goal (class DISCARD-GOAL) (mode FINISHED))
	=>
	(modify ?gm (assigned-to robot2))
	(modify ?g (mode SELECTED))
)

(defrule goal-reasoner-get-mounted-cap-goal-select
	?g <- (goal (id ?goal-id) (class GET-MOUNTED-BASE-GOAL) (mode FORMULATED))
	?gm <- (goal-meta (goal-id ?goal-id) (assigned-to nil))
	(goal (class MOUNT-CAP-GOAL) (mode FINISHED))

		=>
 	(modify ?gm (assigned-to robot2))
	(modify ?g (mode SELECTED))
)

