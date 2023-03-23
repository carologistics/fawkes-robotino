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


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  GROUP 1 PARALLEL EXECUTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defrule goal-expander-g1-c1-spawn-wp
"Spawn a WP"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params workpiece ?wp))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )
 	=>
	(bind ?action-class (sym-cat SPAWN-WP- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action spawn-wp ?wp ?robot)
	)
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-g1-c1-prepare-bs
"Prepare BS"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params bs ?bs bs-side ?bs-side base-clr ?bs-clr))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )
 	=>
	(bind ?action-class (sym-cat RETRIEVE-BASE- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
 		(plan-assert-action prepare-bs ?bs ?bs-side ?bs-clr)
	)
	(modify ?g (mode EXPANDED))
)

(defrule goal-expander-g1-c1-prepare-rs
"Prepare RS"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params rs ?rs ring-color ?rng-clr ring-req ?rng-req))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact mps-state args? m ?rs s ?m-state ))
	(wm-fact (key domain fact rs-filled-with args? m ?rs n ?rng-before ))

 	=>
	(bind ?a (sym-to-int ?rng-before))
  	(bind ?b (sym-to-int ?rng-req))
  	(bind ?rng-after (int-to-sym (- ?a ?b)))

	(bind ?action-class (sym-cat PREPARE-RS- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action prepare-rs ?rs ?rng-clr ?rng-before ?rng-after ?rng-req)
	)
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-g1-c1-cap-retrieve
"Prepare CS and retrieve a cap"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params cap-color ?cap-clr cap-station ?cs))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )
	(wm-fact (key domain fact at args?  r ?robot m ?at-mps side ?at-side )) 
	(wm-fact (key domain fact wp-on-shelf args?  wp ?cc m ?cs spot ?cc-spot)) 
	=>
	(bind ?action-class (sym-cat CAP-RETRIEVE- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action move ?robot ?at-mps ?at-side ?cs INPUT)
		(plan-assert-action wp-get-shelf ?robot ?cc ?cs ?cc-spot)
		(plan-assert-action wp-put ?robot ?cc ?cs INPUT)
		(plan-assert-action prepare-cs ?cs RETRIEVE_CAP)
		(plan-assert-action cs-retrieve-cap ?cs ?cc ?cap-clr)
		(plan-assert-action move ?robot ?cs INPUT ?cs WAIT)	
	)	
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-g1-c1-make-payment-cs
"Make one payment using already ready cap station"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params cs ?cs rs ?rs ring-before ?rng-before ring-after ?rng-after))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )
	(wm-fact (key domain fact at args?  r ?robot m ?at-mps side ?at-side )) 
	(wm-fact (key domain fact wp-at args?  wp ?cc m ?cs side OUTPUT)) 
	(wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-before))
	=>
	(bind ?action-class (sym-cat PAYMENT-CS- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action move ?robot ?at-mps ?at-side ?cs OUTPUT)
		(plan-assert-action wp-get ?robot ?cc ?cs OUTPUT)
		(plan-assert-action move ?robot ?cs OUTPUT ?rs INPUT)	
		(plan-assert-action wp-put-slide-cc ?robot ?cc ?rs ?rng-before ?rng-after)
		(plan-assert-action move ?robot ?rs INPUT ?rs WAIT)	
	)	
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-g1-c1-make-payment-bs
"Make one payment using base station"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params bs ?bs rs ?rs ring-before ?rng-before ring-after ?rng-after))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )
	(wm-fact (key domain fact at args?  r ?robot m ?at-mps side ?at-side )) 
	(wm-fact (key domain fact wp-at args?  wp ?cc m ?cs side OUTPUT)) 
	(wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-before))
	=>
	(bind ?action-class (sym-cat PAYMENT-BS- (gensym*)))
	(bind ?p-cc (sym-cat CC- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action spawn-wp ?p-cc ?robot)
		(plan-assert-action prepare-bs ?bs OUTPUT BASE_CLEAR)
		(plan-assert-action bs-dispense ?bs OUTPUT ?p-cc BASE_CLEAR)
		(plan-assert-action move ?robot ?at-mps ?at-side ?bs OUTPUT)
		(plan-assert-action wp-get ?robot ?p-cc ?bs OUTPUT)
		(plan-assert-action move ?robot ?bs OUTPUT ?rs INPUT)	
		(plan-assert-action wp-put-slide-cc ?robot ?p-cc ?rng-before ?rng-after)
		(plan-assert-action move ?robot ?rs INPUT ?rs WAIT)	
	)	
	(modify ?g (mode EXPANDED))
)



(defrule goal-expander-g1-c1-transport-wp
"Transport WP"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params from ?from from-side ?from-side to ?to to-side ?to-side workpiece ?wp))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact at args?  r ?robot m ?at-mps side ?at-side )) 
	=>
	(bind ?action-class (sym-cat TRANSPORT-WP- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(if (or (neq ?at-mps ?from) (neq ?at-side ?from-side)) then 
		(create$
			(plan-assert-action move ?robot ?at-mps ?at-side ?from ?from-side)
			(plan-assert-action wp-get ?robot ?wp ?from ?from-side)
			(plan-assert-action move ?robot ?from ?from-side ?to ?to-side)
			(plan-assert-action wp-put ?robot ?wp ?to ?to-side)
		)
		else 
		(create$
			(plan-assert-action wp-get ?robot ?wp ?from ?from-side)
			(plan-assert-action move ?robot ?from ?from-side ?to ?to-side)
			(plan-assert-action wp-put ?robot ?wp ?to ?to-side)
		)
		)
	)
	(modify ?g (mode EXPANDED))
)




(defrule goal-expander-g1-c1-bs-dispense
"Dispatch a base"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params bs ?bs bs-clr ?bs-clr workpiece ?wp))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )

	=>
	(bind ?action-class (sym-cat BASE-DISPENSE- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action bs-dispense ?bs OUTPUT ?wp ?bs-clr)
	)	
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-g1-c1-mount-ring1
"Mount one ring"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params rs ?rs ring-clr ?rng-clr ring-req ?rng-req workpiece ?wp))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
	(wm-fact (key domain fact rs-filled-with args? m ?rs n ?rng-before ))

	=>

	(bind ?action-class (sym-cat MOUNT-RING1- (gensym*)))
	
	(bind ?a (sym-to-int ?rng-before))
  	(bind ?b (sym-to-int ?rng-req))
  	(bind ?rng-after (int-to-sym (- ?a ?b)))

	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action rs-mount-ring1 ?rs ?wp ?rng-clr ?rng-before ?rng-after ?rng-req)
	)	
	(modify ?g (mode EXPANDED))
)


(defrule goal-expander-g1-c1-cap-mount
"Prepare CS and mount the cap"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params cs ?cs cap-clr ?cap-clr workpiece ?wp))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )
	=>
	(bind ?action-class (sym-cat CAP-MOUNT- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action prepare-cs ?cs MOUNT_CAP)
 		(plan-assert-action cs-mount-cap ?cs ?wp ?cap-clr)	
	)	
	(modify ?g (mode EXPANDED))
)



(defrule goal-expander-g1-c1-deliver
"Prepare DS and deliver the order"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params order ?ord workpiece ?wp delivery-station ?ds ds-gate ?ds-gate base-clr ?base-clr cap-clr ?cap-clr rng-clr ?rng-clr))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )
	=>
	(bind ?action-class (sym-cat DELIVER- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action prepare-ds ?ds ?ord)
		(plan-assert-action fulfill-order-c1 ?ord ?wp ?ds ?ds-gate ?base-clr ?cap-clr ?rng-clr)
	)	
	(modify ?g (mode EXPANDED))
)



(defrule goal-expander-g1-c0-deliver
"Prepare DS and deliver the order"
	?g <- (goal (id ?goal-id) (class ?cls) (mode SELECTED) 
 	            (params order ?ord workpiece ?wp delivery-station ?ds ds-gate ?ds-gate base-clr ?base-clr cap-clr ?cap-clr))
 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil) )
	=>
	(bind ?action-class (sym-cat DELIVER- (gensym*)))
	(plan-assert-sequential ?action-class ?goal-id ?robot
		(plan-assert-action prepare-ds ?ds ?ord)
		(plan-assert-action fulfill-order-c0 ?ord ?wp ?ds ?ds-gate ?base-clr ?cap-clr)
	)	
	(modify ?g (mode EXPANDED))
)
	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  GROUP 1 SERIAL EXECUTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; (defrule goal-expander-g1-c1-spawn-wp
; "Spawn the workpiece"
; 	?g <- (goal (id ?goal-id) (class ORDER1) (mode SELECTED) (parent ?parent)
;  	            (params workpiece ?wp robot ?robot))
;  	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;  	=>
; 	(plan-assert-sequential SPAWN-WP ?goal-id ?robot
; 		(plan-assert-action spawn-wp ?wp ?robot)
; 	)
; 	(modify ?g (mode EXPANDED))
; )

; (defrule goal-expander-g1-c1-base
; "Take the base from base station"
; 	?g <- (goal (id ?goal-id) (class ORDER1) (mode SELECTED) 
;  	            (params bs ?bs bs-side ?bs-side base-color ?bs-clr workpiece ?wp ))
;  	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;  	=>
; 	(plan-assert-sequential RETRIEVE-BASE ?goal-id ?robot
; 		(plan-assert-action move ?robot START INPUT ?bs ?bs-side)
; 		(plan-assert-action prepare-bs ?bs ?bs-side ?bs-clr)
; 		(plan-assert-action bs-dispense ?bs ?bs-side ?wp ?bs-clr)
; 	)
; 	(modify ?g (mode EXPANDED))
; )


; (defrule goal-expander-g1-c1-transport-wp
; "Take the workpiece from one mps to another"
; 	?g <- (goal (id ?goal-id) (class ORDER1) (mode SELECTED) 
;  	            (params from ?from from-side ?from-side to ?to to-side ?to-side workpiece ?wp))
;  	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
; 	(wm-fact (key domain fact at args?  r ?robot m ?at-mps side ?at-side )) 
;  	=>
	
; 	(bind ?cls (sym-cat TRANSPORT-WP- (gensym*)))

; 	(plan-assert-sequential ?cls ?goal-id ?robot
; 		(if (or (neq ?at-mps ?from) (neq ?at-side ?from-side)) then 
; 		(create$
; 			(plan-assert-action move ?robot ?at-mps ?at-side ?from ?from-side)
; 			(plan-assert-action wp-get ?robot ?wp ?from ?from-side)
; 			(plan-assert-action move ?robot ?from ?from-side ?to ?to-side)
; 			(plan-assert-action wp-put ?robot ?wp ?to ?to-side)
; 		)
; 		else 
; 		(create$
; 			(plan-assert-action wp-get ?robot ?wp ?from ?from-side)
; 			(plan-assert-action move ?robot ?from ?from-side ?to ?to-side)
; 			(plan-assert-action wp-put ?robot ?wp ?to ?to-side)
; 		)
; 		)
; 	)
; 	(modify ?g (mode EXPANDED))
; )


; (defrule goal-expander-g1-c1-rs
; "Prepare and dispatch a ring"
; 	?g <- (goal (id ?goal-id) (class ORDER1) (mode SELECTED) 
;  	            (params target-rs ?rs ring-color ?rng-clr ring-before ?rng-before ring-after ?rng-after ring-req ?rng-req workpiece ?wp))
;  	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
; 	; (wm-fact (key domain fact rs-filled-with args? m ?mps n ?rs-before))
; 	; (wm-fact (key domain fact rs-sub args? minuend ?rs-before subtrahend ?rs-req difference ?rs-after))
; 	(wm-fact (key domain fact rs-ring-spec args? m ?rs r ?rng-clr rn ?rn))
;  	=>
; 	(printout t "Req-ring" ?rn)

; 	(plan-assert-sequential MOUNT-RING ?goal-id ?robot
; 		(plan-assert-action prepare-rs ?rs ?rng-clr ?rng-before ?rng-after ?rng-req)
; 		(plan-assert-action rs-mount-ring1 ?rs ?wp ?rng-clr ?rng-before ?rng-after ?rng-req)
; 	)
; 	(modify ?g (mode EXPANDED))
; )


; (defrule goal-expander-g1-c1-cap-retrive
; "Retrieve the cap"
; 	?g <- (goal (id ?goal-id) (class ORDER1) (mode SELECTED) 
;                 (params cap-carrier ?wp cap-color ?cap-clr cap-station ?cs))
;  	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
; 	(wm-fact (key domain fact at args?  r ?robot m ?at-mps side ?at-side )) 
; 	(wm-fact (key domain fact wp-on-shelf args?  wp ?cc m ?cs spot ?cc-spot)) 

;  	=>

; 	(plan-assert-sequential CAP-RETRIEVE ?goal-id ?robot
; 		(plan-assert-action prepare-cs ?cs RETRIEVE_CAP)
; 		(plan-assert-action move ?robot ?at-mps ?at-side ?cs INPUT)
; 		(plan-assert-action wp-get-shelf ?robot ?cc ?cs ?cc-spot)
; 		(plan-assert-action wp-put ?robot ?cc ?cs INPUT)
; 		(plan-assert-action cs-retrieve-cap ?cs ?cc ?cap-clr)
; 		(plan-assert-action move ?robot ?cs INPUT ?cs OUTPUT)	
; 		(plan-assert-action wp-get ?robot ?cc ?cs OUTPUT)
; 		(plan-assert-action wp-discard ?robot ?cc)
; 	)
; 	(modify ?g (mode EXPANDED))
; )



; (defrule goal-expander-g1-c1-cap-mount
; "Mount the cap on the order workpiece"
; 	?g <- (goal (id ?goal-id) (class ORDER1) (mode SELECTED) 
;                 (params order-carrier ?wp cap-color ?cap-clr cap-station ?cs))
;  	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;  	=>

; 	(plan-assert-sequential CAP-MOUNT ?goal-id ?robot
; 	    (plan-assert-action prepare-cs ?cs MOUNT_CAP)
; 		(plan-assert-action cs-mount-cap ?cs ?wp ?cap-clr)
; 	)
; 	(modify ?g (mode EXPANDED))
; )



; (defrule goal-expander-g1-c1-deliver
; "Prepare the DS and deliver order"
; 	?g <- (goal (id ?goal-id) (class ORDER1) (mode SELECTED) 
;                 (params order ?ord workpiece ?wp delivery-station ?ds ds-gate ?ds-gate base-clr ?base-clr cap-clr ?cap-clr rng-clr ?rng-clr))
;  	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
;  	=>

; 	(plan-assert-sequential CAP-MOUNT ?goal-id ?robot
; 		(plan-assert-action prepare-ds ?ds ?ord)
; 		(plan-assert-action fulfill-order-c1 ?ord ?wp ?ds ?ds-gate ?base-clr ?cap-clr ?rng-clr)
; 	)
; 	(modify ?g (mode EXPANDED))
; )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; (defrule goal-expander-g1-c1-execute
; "Execute the goal "
; 	?g <- (goal (id ?goal-id) (class TESTGOAL) (mode SELECTED) (parent ?parent)
; 	            (params bs ?bs base-color ?bs-color workpiece ?wp))
; 	(goal-meta (goal-id ?goal-id) (assigned-to ?robot&~nil))
; 	(wm-fact (key domain fact at args? r ?robot m ?curr-loc side ?curr-side))
; 	(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
; 	=>
; 	(bind ?shelf-spot nil)
; 	(do-for-fact ((?wp-on-shelf wm-fact)) (and (wm-key-prefix ?wp-on-shelf:key (create$ domain fact wp-on-shelf))
; 	                                      (eq (wm-key-arg ?wp-on-shelf:key wp) ?cc)
; 	                                      (eq (wm-key-arg ?wp-on-shelf:key m) ?cs))
; 		(bind ?shelf-spot (wm-key-arg ?wp-on-shelf:key spot))
; 	)
; 	(plan-assert-sequential BUFFER-CAP-PLAN ?goal-id ?robot
; 		(plan-assert-safe-move ?robot ?curr-loc ?curr-side ?bs INPUT
; 			(plan-assert-action prepare-bs ?bs INPUT ?bs-color)
; 			(plan-assert-action bs-dispense ?bs OUTPUT ?wp ?bs-color)
; 		)
; 	)
; 	(modify ?g (mode EXPANDED))
; )



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