;****************************************************************************
;  rcll_domain_production.pddl: RoboCup Logistics League Production Model
;
;  Created: Fri Feb 24 23:20:38 2017
;  Copyright  2017  Tim Niemueller [www.niemueller.de]
;             2018  Till Hofmann
;****************************************************************************

;  This program is free software; you can redistribute it and/or modify
;  it under the terms of the GNU General Public License as published by
;  the Free Software Foundation; either version 2 of the License, or
;  (at your option) any later version.
;
;  This program is distributed in the hope that it will be useful,
;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;  GNU Library General Public License for more details.
;
;  Read the full text in the LICENSE.GPL file in the doc directory.

(define (domain rcll-production)
(:requirements :strips :typing)

(:types
	robot - object
	team-color - object
	location - object
	obstacle - object
	waitpoint - location
	mps - location
	mps - obstacle
	zone - location
	mps-typename - object
	mps-statename - object
	mps-side - object
	base-color - object
	cap-color - object
	ring-color - object
	ds-gate - object
	ss-operation - object
	cs-operation - object
	cs-statename - object
	order - object
	order-complexity-value - object
	workpiece - object
	cap-carrier - workpiece
	shelf-spot - object
	number - object
	ring-num - number
	ss-shelf - number
	ss-slot - number
	token - object
	master-token - token
)

(:constants
	START - location
	UNKNOWN NONE - obstacle
	UNKNOWN_ROBOT - robot
	BS CS DS RS SS - mps-typename
	IDLE BROKEN PREPARED PROCESSING PROCESSED WAIT-IDLE READY-AT-OUTPUT DOWN - mps-statename
	INPUT OUTPUT WAIT - mps-side
	BASE_NONE BASE_RED BASE_BLACK BASE_SILVER - base-color
	CAP_NONE CAP_BLACK CAP_GREY - cap-color
	GATE-1 GATE-2 GATE-3 - ds-gate
	RING_NONE RING_BLUE RING_GREEN RING_ORANGE RING_YELLOW - ring-color
	RETRIEVE_CAP MOUNT_CAP - cs-operation
	RETRIEVE STORE - ss-operation
	C0 C1 C2 C3 - order-complexity-value
	LEFT MIDDLE RIGHT - shelf-spot
	NA ZERO ONE TWO THREE - ring-num
	ZERO ONE TWO THREE FOUR FIVE - ss-shelf
	ZERO ONE TWO THREE FOUR FIVE SIX SEVEN - ss-slot
  C-Z18 C-Z28 C-Z38 C-Z48 C-Z58 C-Z68 C-Z78 - zone
  C-Z17 C-Z27 C-Z37 C-Z47 C-Z57 C-Z67 C-Z77 - zone
  C-Z16 C-Z26 C-Z36 C-Z46 C-Z56 C-Z66 C-Z76 - zone
  C-Z15 C-Z25 C-Z35 C-Z45 C-Z55 C-Z65 C-Z75 - zone
  C-Z14 C-Z24 C-Z34 C-Z44 C-Z54 C-Z64 C-Z74 - zone
  C-Z13 C-Z23 C-Z33 C-Z43 C-Z53 C-Z63 C-Z73 - zone
  C-Z12 C-Z22 C-Z32 C-Z42 C-Z52 C-Z62 C-Z72 - zone
  C-Z11 C-Z21 C-Z31 C-Z41 - zone
  M-Z78 M-Z68 M-Z58 M-Z48 M-Z38 M-Z28 M-Z18 - zone
  M-Z77 M-Z67 M-Z57 M-Z47 M-Z37 M-Z27 M-Z17 - zone
  M-Z76 M-Z66 M-Z56 M-Z46 M-Z36 M-Z26 M-Z16 - zone
  M-Z75 M-Z65 M-Z55 M-Z45 M-Z35 M-Z25 M-Z15 - zone
  M-Z74 M-Z64 M-Z54 M-Z44 M-Z34 M-Z24 M-Z14 - zone
  M-Z73 M-Z63 M-Z53 M-Z43 M-Z33 M-Z23 M-Z13 - zone
  M-Z72 M-Z62 M-Z52 M-Z42 M-Z32 M-Z22 M-Z12 - zone
                    M-Z41 M-Z31 M-Z21 M-Z11 - zone

	G-1-1 G-2-1 G-3-1 G-4-1 G-5-1 - waitpoint
	G-1-2 G-2-2 G-3-2 G-4-2 G-5-2 - waitpoint
	G-1-3 G-2-3 G-3-3 G-4-3 G-5-3 - waitpoint
	G-1-4 G-2-4 G-3-4 G-4-4 G-5-4 - waitpoint
	G-1-5 G-2-5 G-3-5 G-4-5 G-5-5 - waitpoint
)

(:predicates
	(at ?r - robot ?m - location ?side - mps-side)
	(holding ?r - robot ?wp - workpiece)
	(can-hold ?r - robot)
	(entered-field ?r - robot)
	(robot-waiting ?r - robot)
	(maps ?m - mps ?r -robot)
	(zone-content ?z - zone ?m - obstacle)
	(mps-type ?m - mps ?t - mps-typename)
	(mps-state ?m - mps ?s - mps-statename)
	(mps-team ?m - mps ?col - team-color)
	(mps-side-free ?m - mps ?side - mps-side)
	(mps-side-approachable ?m - location ?side - mps-side)
	(bs-prepared-color ?m - mps ?col - base-color)
	(bs-prepared-side ?m - mps ?side - mps-side)
	(bs-color ?m - mps ?col - base-color)
	(cs-can-perform ?m - mps ?op - cs-operation)
	(cs-prepared-for ?m - mps ?op - cs-operation)
	(cs-buffered ?m - mps ?col - cap-color)
	(cs-color ?m - mps ?col - cap-color)
	(ss-prepared-for ?m - mps ?op - ss-operation ?wp - workpiece ?shelf - ss-shelf ?slot - ss-slot)
	(rs-prepared-color ?m - mps ?col - ring-color)
	(rs-ring-spec ?m - mps ?r - ring-color ?rn - ring-num)
	(rs-filled-with ?m - mps ?n - ring-num)
	;rs-sub and rs-inc are static predicates stating the legal ring-num operations
	(rs-sub ?minuend - ring-num ?subtrahend - ring-num ?difference - ring-num)
	(rs-inc ?summand - ring-num ?sum - ring-num)
	(ds-prepared-order ?m - mps ?ord - order)
	(order-complexity ?ord - order ?com - order-complexity-value)
	(order-base-color ?ord - order ?col - base-color)
	(order-ring1-color ?ord - order ?col - ring-color)
	(order-ring2-color ?ord - order ?col - ring-color)
	(order-ring3-color ?ord - order ?col - ring-color)
	(order-cap-color ?ord - order ?col - cap-color)
	(order-fulfilled ?ord - order)
	(order-delivery-begin ?ord - order)
	(order-delivery-end ?ord - order)
	(order-gate ?ord - order ?gate - ds-gate)
	(wp-unused ?wp - workpiece)
	(wp-usable ?wp - workpiece)
	(wp-at ?wp - workpiece ?m - mps ?side - mps-side)
	(wp-base-color ?wp - workpiece ?col - base-color)
	(wp-ring1-color ?wp - workpiece ?col - ring-color)
	(wp-ring2-color ?wp - workpiece ?col - ring-color)
	(wp-ring3-color ?wp - workpiece ?col - ring-color)
	(wp-cap-color ?wp - workpiece ?col - cap-color)
	(wp-on-shelf ?wp - workpiece ?m - mps ?spot - shelf-spot)
	(wp-spawned-for ?wp - workpiece ?r - robot)
	(wp-get-pending ?wp - workpiece ?m - mps ?side - mps-side)
	(spot-free ?m - mps ?spot - shelf-spot)
	(ss-stored-wp ?m  - mps ?wp - workpiece ?shelf - ss-shelf ?slot - ss-slot)
	(ss-shelf-slot-free ?m  - mps ?shelf - ss-shelf ?slot - ss-slot)
	(ss-new-wp-at ?m  - mps ?wp - workpiece ?shelf - ss-shelf
	              ?slot - ss-slot ?base-col - base-color
	              ?ring1-col - ring-color ?ring2-col - ring-color
	              ?ring3-col - ring-color ?cap-col - cap-color)
)

(:action goal-instruct-cs-mount-cap
	:parameters (?mps - mps ?cap-color - cap-color)
	:precondition (and
			(not (mps-state ?mps BROKEN))
			(mps-type ?mps CS)
			(cs-can-perform ?mps MOUNT_CAP)
			(or (cs-buffered ?mps CAP_BLACK)
					(cs-buffered ?mps CAP_GREY)
			)
			(mps-side-free ?mps OUTPUT)
			(not (mps-side-free ?mps INPUT))
		)
	:effect (mps-state ?mps READY-AT-OUTPUT)
)

(:action goal-mount-cap
	:parameters (?wp - workpiece ?target-mps - mps ?target-side - mps-side ?wp-loc - mps ?wp-side - mps-side ?robot - robot)
	:precondition (and (mps-type ?target-mps CS)
										 (not (mps-state ?target-mps BROKEN))
										 ;(not (wp-at ?any-wp  ?target-mps INPUT))
							 			 (mps-side-free ?target-mps INPUT)
										 ;(mps-team ?target-mps ?team-color)
										 (or (cs-buffered ?target-mps CAP_BLACK)
												 (cs-buffered ?target-mps CAP_GREY)
										 )
										 (cs-can-perform ?target-mps MOUNT_CAP)
										 ;(mps-team ?wp-loc ?team-color)
										 (or (and
														 (can-hold ?robot)
														 (or (and (mps-type ?wp-loc BS)
														          (wp-unused ?wp)
																			(wp-base-color ?wp BASE_NONE)
														 		 )
																 (wp-at ?wp ?wp-loc ?wp-side)
														 )
												 )
												 (holding ?robot ?wp)
										 )
						 )
	:effect (mps-state ?target-mps READY-AT-OUTPUT)
)

; Capcarrier CEs
;(or   (and
;				 (not (wm-fact (key domain fact holding args? r ?robot wp ?wp-h)))
;				 (wm-fact (key domain fact wp-on-shelf args? wp ?cc m ?mps spot ?spot))
;				 (not (plan-action (action-name wp-get-shelf) (param-values $? ?wp $?)))
;				 (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
;		 )
;		 (and
;				 (wm-fact (key domain fact holding args? r ?robot wp ?cc))
;				 (wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
;				 (domain-object (name ?cc) (type cap-carrier))
;
(:action goal-buffer-cap
	:parameters (?target-mps - mps ?cap-color - cap-color)
	:precondition (and (mps-type ?target-mps CS)
										 (not (mps-state ?target-mps BROKEN))
										 (cs-can-perform ?target-mps RETRIEVE_CAP)
										 (not (or (cs-buffered ?target-mps CAP_BLACK)
												 (cs-buffered ?target-mps CAP_GREY)
										 	 )
										 )
										 (mps-side-free ?target-mps INPUT)
							  )
	:effect (mps-state ?target-mps READY-AT-OUTPUT)
)

;(wm-fact (key domain fact wp-cap-color args? wp ?cc col ?cap-color))
(:action goal-instruct-cs-buffer-cap
	:parameters (?target-mps - mps ?cap-color - cap-color)
	:precondition (and (mps-type ?target-mps CS)
										 (not (mps-state ?target-mps BROKEN))
										 (cs-can-perform ?target-mps RETRIEVE_CAP)
										 (not (or (cs-buffered ?target-mps CAP_BLACK)
												 (cs-buffered ?target-mps CAP_GREY)
										 	 )
										 )
										 (not (mps-side-free ?target-mps INPUT))
										 (mps-side-free ?target-mps OUTPUT)
								)
	:effect (mps-state ?target-mps READY-AT-OUTPUT)
)

(:action goal-discard
	:parameters (?wp - workpiece ?wp-loc - mps ?wp-side - mps-side ?robot - robot)
	:precondition (and (or (mps-type ?wp-loc CS)
												 (mps-type ?wp-loc BS)
												 (mps-type ?wp-loc DS)
												 (mps-type ?wp-loc RS)
												 (mps-type ?wp-loc SS))
										 ;(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))
										 (or (and (can-hold ?robot)
									 	          (wp-at ?wp ?wp-loc ?wp-side))
									 	    (holding ?robot ?wp))
							  )
	:effect (mps-type ?wp-loc CS)
)

(:action goal-instruct-bs-dispense-base
	:parameters (?wp - workpiece ?target-mps - mps ?target-side - mps-side ?base-color - base-color)
	:precondition (and (mps-type ?target-mps BS)
										 (mps-state ?target-mps IDLE)
  								 	 ;(wm-fact (key domain fact mps-team args? m ?mps col ?team-color))
										 (mps-side-free ?target-mps INPUT)
										 (mps-side-free ?target-mps OUTPUT)
										 ;(mps-side-free ?target-mps WAIT)
									 	 (wp-unused ?wp)
										 (wp-get-pending ?wp ?target-mps ?target-side)
								)
	:effect (mps-state ?target-mps READY-AT-OUTPUT)
)

(:action goal-deliver
	:parameters (?wp - workpiece ?target-mps - mps)
	:precondition (and (mps-type ?target-mps DS)
										 (mps-state ?target-mps IDLE)
										 (wp-at ?wp ?target-mps INPUT)
							  )
	:effect (mps-state ?target-mps READY-AT-OUTPUT)
)

(:action goal-deliver-rc21
	:parameters (?wp - workpiece ?target-mps - mps)
	:precondition (and (mps-type ?target-mps DS)
										 (mps-state ?target-mps IDLE)
										 (wp-at ?wp ?target-mps INPUT)

										 ;(wm-fact (key domain fact mps-type args? m ?wp-loc t ?))
										 ;(wm-fact (key domain fact mps-team args? m ?wp-loc col ?team-color))

										 ;(or (can-hold ?robot)
										 ;	 (holding ?robot ?wp)
										 ;)
							  )
	:effect (mps-state ?target-mps READY-AT-OUTPUT)
)

(:action goal-mount-ring
	:parameters (?wp - workpiece ?target-mps - mps ?target-side - mps-side
				?wp-loc - mps ?wp-side - mps-side ?ring-color - ring-color)
	:precondition (and (mps-type ?target-mps RS)
										 (not (mps-state ?target-mps BROKEN))
										 ;(wm-fact (key domain fact mps-team args?       m ?target-mps col ?team-color))
										 ;(wm-fact (key domain fact rs-filled-with args? m ?target-mps n ?bases-filled))
										 ;(wm-fact (key domain fact ?wp-ring-color&:(eq ?wp-ring-color(sym-cat wp-ring (sub-string 5 5 ?ring) -color)) args? wp ?wp col RING_NONE ))
										 ;(wm-fact (key domain fact ?order-ring-color&:(eq ?order-ring-color(sym-cat order-ring (sub-string 5 5 ?ring) -color))args? ord ?order col ?ring-color ))
										 ;(wm-fact (key domain fact order-complexity args? ord ?order com ?complexity&C1|C2|C3))
										 ;WP LOC IS A WORKPIECE HERE?!?
										 ;(not (wm-fact (key domain fact wp-at args? wp ?wp-loc m ?target-mps side INPUT)))
										 ;(not (wp-at ?wp-loc ?target-mps INPUT))
										 ;(Other-rs is missing)
										 ;(or (wm-fact (key domain fact mps-side-free args? m ?target-mps side OUTPUT))
									 	 ;   (and (wm-fact (key domain fact mps-type args? m ?other-rs&~?target-mps t RS))
									 	 ;        (wm-fact (key domain fact mps-team args? m ?other-rs col ?team-color))
									 	 ;        (wm-fact (key domain fact mps-side-free args? m ?other-rs side ?any-side))))
										 ;MPS-SOURCE CEs FACTS MISSING .
								)
	:effect (mps-state ?target-mps READY-AT-OUTPUT)
)

;Similar difficulties to mount-ring
;(:action goal-instruct-rs-mount-ring
;	:parameters ()
;	:precondition ()
;	:effect ()
;)

)
