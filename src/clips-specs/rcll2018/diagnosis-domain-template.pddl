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
	(:requirements :strips :typing :action-costs :adl)

	(:types
		robot - object
		team-color - object
		location - object
    	waitpoint - location
		mps - location
		mps-typename - object
		mps-statename - object
		mps-side - object
		base-color - object
		cap-color - object
		ring-color - object
		ds-gate - object
		cs-operation - object
		cs-statename - object
		order - object
    	order-complexity-value - object
		workpiece - object
		cap-carrier - workpiece
		shelf-spot - object
		ring-num - object
		zone - object
		token - object
		master-token - token
		state - object
		component - object
	)

	(:constants
		START - location
		BS CS DS RS SS - mps-typename
		IDLE BROKEN PREPARED PROCESSING PROCESSED WAIT-IDLE READY-AT-OUTPUT DOWN - mps-statename
		;INPUT OUTPUT - mps-side
		BASE_NONE BASE_RED BASE_BLACK BASE_SILVER - base-color
		CAP_NONE CAP_BLACK CAP_GREY - cap-color
		GATE-1 GATE-2 GATE-3 - ds-gate
		RING_NONE RING_BLUE RING_GREEN RING_ORANGE RING_YELLOW - ring-color
		RETRIEVE_CAP MOUNT_CAP - cs-operation
		C0 C1 C2 C3 - order-complexity-value
		LEFT MIDDLE RIGHT - shelf-spot
		NA ZERO ONE TWO THREE - ring-num
		<<#constants>>
	)

	(:predicates
		(self ?r - robot)
		(at ?r - robot ?m - location ?side - mps-side)
		(holding ?r - robot ?wp - workpiece)
		(can-hold ?r - robot)
		(entered-field ?r - robot)
		(location-free ?l - location ?side - mps-side)
		(robot-waiting ?r - robot)
		(mps-type ?m - mps ?t - mps-typename)
		(mps-state ?m - mps ?s - mps-statename)
		(mps-team ?m - mps ?col - team-color)
		(bs-prepared-color ?m - mps ?col - base-color)
		(bs-prepared-side ?m - mps ?side - mps-side)
		(cs-can-perform ?m - mps ?op - cs-operation)
		(cs-prepared-for ?m - mps ?op - cs-operation)
		(cs-buffered ?m - mps ?col - cap-color)
		(cs-color ?m - mps ?col - cap-color)
		(mps-side-free ?m - mps ?side - mps-side)
		(rs-prepared-color ?m - mps ?col - ring-color)
		(rs-ring-spec ?m - mps ?r - ring-color ?rn - ring-num)
		(rs-filled-with ?m - mps ?n - ring-num)
		;rs-sub and rs-inc are static predicates stating the legal ring-num operations
		(rs-sub ?minuend - ring-num ?subtrahend - ring-num ?difference - ring-num)
		(rs-inc ?summand - ring-num ?sum - ring-num)
		(ds-prepared-gate ?m - mps ?g - ds-gate)
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
    (spot-free ?m - mps ?spot - shelf-spot)
		(comp-state ?c - component ?s - state)

    (exog-possible)
		(last-begin)
		(next-finish)

    (locked ?name - object)
    (location-locked ?m - mps ?s - mps-side)

		(next-reset-mps ?m - mps)
		(last-reset-mps ?m - mps)

		(next-prepare-bs ?m - mps ?side - mps-side ?bc - base-color)
		(last-prepare-bs ?m - mps ?side - mps-side ?bc - base-color)

		(next-prepare-ds ?m - mps ?gate - ds-gate)
		(last-prepare-ds ?m - mps ?gate - ds-gate)

		(next-prepare-cs ?m - mps ?op - cs-operation)
		(last-prepare-cs ?m - mps ?op - cs-operation)

		(next-bs-dispense ?r - robot ?m - mps ?side - mps-isde ?wp - workpiece ?basecol - base-color)
		(last-bs-dispense ?r - robot ?m - mps ?side - mps-isde ?wp - workpiece ?basecol - base-color)
	
		(next-cs-mount-cap ?m - mps ?wp - workpiece ?capcol - cap-color)
		(last-cs-mount-cap ?m - mps ?wp - workpiece ?capcol - cap-color)

		(next-cs-retrieve-cap ?m - mps ?cc - cap-carrier ?capcol - cap-color)
		(last-cs-retrieve-cap ?m - mps ?cc - cap-carrier ?capcol - cap-color)
	
		(next-prepare-rs ?m - mps ?rc - ring-color)
		(last-prepare-rs ?m - mps ?rc - ring-color)

		(next-rs-mount-ring1 ?m - mps ?wp - workpiece ?col - ring-color)
		(last-rs-mount-ring1 ?m - mps ?wp - workpiece ?col - ring-color)
	
		(next-rs-mount-ring2 ?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color)
		(last-rs-mount-ring2 ?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color)
	
		(next-rs-mount-ring3 ?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color ?col2 - ring-color)
		(last-rs-mount-ring3 ?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color ?col2 - ring-color)
	
		(next-go-wait ?r - robot ?to - waitpoint)
		(last-go-wait ?r - robot ?to - waitpoint)
	
		(next-wait ?r - robot)
		(last-wait ?r - robot)

		(next-move ?r - robot ?to - mps ?to-side - mps-side)
		(last-move ?r - robot ?to - mps ?to-side - mps-side)

		(next-enter-field ?r - robot)
		(last-enter-field ?r - robot)

		(next-wp-discard ?r - robot)
		(last-wp-discard ?r - robot)

		(next-wp-get-shelf ?r - robot ?spot - shelf-spot)
		(last-wp-get-shelf ?r - robot ?spot - shelf-spot)

		(next-wp-get ?r - robot)
		(last-wp-get ?r - robot)

		(next-wp-put ?r - robot)
		(last-wp-put ?r - robot)

		(next-wp-put-slide ?r - robot)
		(last-wp-put-slide ?r - robot)

		(next-fulfill-order-c0)
		(last-fulfill-order-c0)

		(next-fulfill-order-c1)
		(last-fulfill-order-c1)

		(next-fulfill-order-c2)
		(last-fulfill-order-c2)

		(next-fulfill-order-c3)
		(last-fulfill-order-c3)

		(next-lock ?name - object)
		(last-lock ?name - object)

		(next-one-time-lock ?name - object)
		(last-one-time-lock ?name - object)

		(next-unlock ?name - object)
		(last-unlock ?name - object)

		(next-eventually-unlock ?name - object)
		(last-eventually-unlock ?name - object)
		
		(next-location-lock ?location - mps ?side - side)
		(last-location-lock ?location - mps ?side - side)

		(next-location-unlock ?location - mps ?side - side)
		(last-location-unlock ?location - mps ?side - side)
	)

;Kind of a hack. actually it should model the removal of present workpieces
	(:action reset-mps
		:parameters (?m - mps)
		:precondition (and (next-reset-mps ?m)
		                   (comp-state communication ESTABLISHED)
		              )
		:effect (and (mps-state ?m BROKEN)
								 (not (next-reset-mps ?m)) (last-reset-mps ?m)
								 (not (exog-possible)))
	)

	(:action prepare-bs
		:parameters (?m - mps ?side - mps-side ?bc - base-color)
		:precondition (and (next-prepare-bs ?m ?side ?bc)
											 (comp-state communication ESTABLISHED)
											 (mps-type ?m BS)
									)
		:effect (and (not (next-prepare-bs ?m ?side ?bc)) 
								 (last-prepare-bs ?m ?side ?bc)
								 (when (and (mps-state ?m IDLE) (locked ?m))
								 			 (and (not (mps-state ?m IDLE)) (mps-state ?m READY-AT-OUTPUT)
								 			 			(bs-prepared-color ?m ?bc) (bs-prepared-side ?m ?side))
								 )
								 (not (exog-possible))
						)
	)

	(:action prepare-ds
		:parameters (?m - mps ?gate - ds-gate)
		:precondition (and (next-prepare-ds ?m ?gate)
											 (comp-state communication ESTABLISHED)
											 (mps-type ?m DS)
									)
		:effect (and (not (next-prepare-ds ?m ?gate))
								 (last-prepare-ds ?m ?gate)
								 (when (and (mps-state ?m IDLE) (locked ?m))
								 			 (and (not (mps-state ?m IDLE)) 
													  (mps-state ?m PREPARED)
                 						(ds-prepared-gate ?m ?gate)
											 )
								 )
                 (not (exog-possible))
						)
	)

	(:action prepare-cs
		:parameters (?m - mps ?op - cs-operation)
		:precondition (and (next-prepare-cs ?m ?op)
                        (comp-state communication ESTABLISHED)
											  (mps-type ?m CS) (mps-state ?m IDLE)
                        (cs-can-perform ?m ?op) (locked ?m))
		:effect (and (not (next-prepare-cs ?m ?op))
								 (last-prepare-cs ?m ?op)
								 (when (and (mps-state ?m IDLE)
                        		(cs-can-perform ?m ?op)
														(locked ?m)
											 )
											 (and (not (mps-state ?m IDLE))
											 			(mps-state ?m READY-AT-OUTPUT)
								 						(not (cs-can-perform ?m ?op))
														(cs-prepared-for ?m ?op)
											 )
								 )
                 (not (exog-possible))
				    )
	)

	(:action bs-dispense
		:parameters (?r - robot ?m - mps ?side - mps-side ?wp - workpiece ?basecol - base-color)
		:precondition (and (next-bs-dispense ?r ?m ?side ?wp ?basecol)
											 (mps-type ?m BS)
											 (mps-state ?m READY-AT-OUTPUT)
                       (locked ?m) 
											 (bs-prepared-color ?m ?basecol)
                       (bs-prepared-side ?m ?side)
											 (wp-base-color ?wp BASE_NONE)
											 (wp-unused ?wp)
											 (wp-spawned-for ?wp ?r)
									)
											 ;(not (wp-usable ?wp))
		:effect (and (not (next-bs-dispense ?r ?m ?side ?wp ?basecol)) (last-bs-dispense ?r ?m ?side ?wp ?basecol)
								 (wp-at ?wp ?m ?side)
								 (not (wp-base-color ?wp BASE_NONE))
								 (wp-base-color ?wp ?basecol)
								 (not (wp-unused ?wp)) 
								 (wp-usable ?wp)
								 (not (wp-spawned-for ?wp ?r))
                 (not (exog-possible))
						)
	)

	(:action cs-mount-cap
		:parameters (?m - mps ?wp - workpiece ?capcol - cap-color)
		:precondition (and (next-cs-mount-cap ?m ?wp ?capcol)
											 (mps-type ?m CS)
											 (mps-state ?m READY-AT-OUTPUT)
											 (locked ?m)
											 (cs-buffered ?m ?capcol)
											 (cs-prepared-for ?m MOUNT_CAP)
											 (wp-usable ?wp)
											 (wp-at ?wp ?m INPUT)
											 (wp-cap-color ?wp CAP_NONE)
									)
		:effect (and (not (next-cs-mount-cap ?m ?wp ?capcol)) (last-cs-mount-cap ?m ?wp ?capcol)
								 (not (wp-at ?wp ?m INPUT))
								 (wp-at ?wp ?m OUTPUT)
								 (not (wp-cap-color ?wp CAP_NONE)) (wp-cap-color ?wp ?capcol)
								 (not (cs-can-perform ?m MOUNT_CAP)) (cs-can-perform ?m RETRIEVE_CAP)
								 (not (cs-prepared-for ?m MOUNT_CAP))
								 (not (cs-buffered ?m ?capcol))
                 (not (exog-possible))
            )
	)

	(:action cs-retrieve-cap
		:parameters (?m - mps ?cc - cap-carrier ?capcol - cap-color)
		:precondition (and (next-cs-retrieve-cap ?m ?cc ?capcol)
											 (mps-type ?m CS) 
											 (mps-state ?m READY-AT-OUTPUT)
                       (locked ?m) 
											 (cs-prepared-for ?m RETRIEVE_CAP)
											 (wp-at ?cc ?m INPUT)
										   (wp-cap-color ?cc ?capcol)
									)
		:effect (and (not (next-cs-retrieve-cap ?m ?cc ?capcol)) (last-cs-retrieve-cap ?m ?cc ?capcol)
								 (not (wp-at ?cc ?m INPUT)) (wp-at ?cc ?m OUTPUT)
								 (not (wp-cap-color ?cc ?capcol)) (wp-cap-color ?cc CAP_NONE)
								 (cs-buffered ?m ?capcol)
								 (cs-can-perform ?m MOUNT_CAP)
								 (not (cs-prepared-for ?m RETRIEVE_CAP))
						     (not (exog-possible))
						)
	)

	(:action prepare-rs
		:parameters (?m - mps ?rc - ring-color ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (next-prepare-rs ?m ?rc)
											 (mps-type ?m RS)
                       (rs-ring-spec ?m ?rc ?r-req)
						           (rs-filled-with ?m ?rs-before)
                       (rs-sub ?rs-before ?r-req ?rs-after)
									)
		:effect (and (not (next-prepare-rs ?m ?rc)) (last-prepare-rs ?m ?rc)
								 (when (and (mps-state ?m IDLE) (locked ?m)
								 			 )
											 (and (not (mps-state ?m IDLE)) (mps-state ?m READY-AT-OUTPUT)
											 			(rs-prepared-color ?m ?rc)
											 )
									
								 )
                 (not (exog-possible))
						)
	)

	(:action rs-mount-ring1
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (next-rs-mount-ring1 ?m ?wp ?col) 
										(mps-type ?m RS) (mps-state ?m READY-AT-OUTPUT) (locked ?m)
										(wp-at ?wp ?m INPUT) (wp-usable ?wp)
										(wp-ring1-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and (not (next-rs-mount-ring1 ?m ?wp ?col)) (last-rs-mount-ring1 ?m ?wp ?col)
								 (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
								 (not (wp-ring1-color ?wp RING_NONE)) (wp-ring1-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after))
	)

	(:action rs-mount-ring2
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color
					?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (next-rs-mount-ring2 ?m ?wp ?col ?col1) 
										(mps-type ?m RS) (mps-state ?m READY-AT-OUTPUT)
                    (locked ?m)
										(wp-at ?wp ?m INPUT) (wp-usable ?wp)
										(wp-ring1-color ?wp ?col1)
										(wp-ring2-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and (not (exog-possible))
		             (not (next-rs-mount-ring2 ?m ?wp ?col ?col1)) (last-rs-mount-ring2 ?m ?wp ?col ?col1)
								 (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
								 (not (wp-ring2-color ?wp RING_NONE)) (wp-ring2-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after)
								 )
	)

	(:action rs-mount-ring3
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color ?col2 - ring-color
							?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (next-rs-mount-ring3 ?m ?wp ?col ?col1 ?col2)
										(mps-type ?m RS) (mps-state ?m READY-AT-OUTPUT) (locked ?m)
										(wp-at ?wp ?m INPUT) (wp-usable ?wp)
										(wp-ring1-color ?wp ?col1)
										(wp-ring2-color ?wp ?col2)
										(wp-ring3-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and (not (exog-possible))
		             (not (next-rs-mount-ring3 ?m ?wp ?col ?col1 ?col2)) (last-rs-mount-ring3 ?m ?wp ?col ?col1 ?col2)
								 (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
								 (not (wp-ring3-color ?wp RING_NONE)) (wp-ring3-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after))
	)

	; The following is the generic move version.
	; It takes the robot from any location (at any side) to any MPS (any side).
	; However, this also creates a tremendous number of options during search and
	; hence is detrimental for planning performance.
	;
	(:action go-wait
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - waitpoint)
		:precondition (and (next-go-wait ?r ?to)
											 (or (at ?r ?to WAIT) (location-free ?to WAIT))
                       (at ?r ?from ?from-side))
		:effect (and (not (exog-possible))
								 (not (next-go-wait ?r ?to)) (last-go-wait ?r ?to)
								 (not (at ?r ?from ?from-side)) (at ?r ?to WAIT)
          			 (not (location-free ?to WAIT)) (location-free ?from ?from-side)
						)
	)

  (:action wait
    :parameters (?r - robot ?point - waitpoint)
    :precondition (next-wait ?r)
    :effect (and (not (exog-possible)) (not (next-wait ?r)) (last-wait ?r))
  )

	(:action move
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
		:precondition (and (next-move ?r ?to ?to-side)
											 (comp-state MOVE-BASE INIT)
											 (at ?r ?from ?from-side)
											 (or (at ?r ?to ?to-side) (location-free ?to ?to-side))
									)
		:effect (and (not (exog-possible))
								 (not (next-move ?r ?to ?to-side)) (last-move ?r ?to ?to-side)
								 (not (at ?r ?from ?from-side)) (at ?r ?to ?to-side)
								 (not (location-free ?to ?to-side)) (location-free ?from ?from-side)
						)
	)

	(:action enter-field
		:parameters (?r - robot)
		:precondition (and (next-enter-field ?r)
											 (or (location-free START INPUT) (at ?r START INPUT))
											 (robot-waiting ?r)
									)
		:effect (and (not (exog-possible))
								 (not (next-enter-field ?r)) (last-enter-field ?r)
								 (entered-field ?r)
								 (at ?r START INPUT)
								 (not (location-free START INPUT))
								 (not (robot-waiting ?r)))
	)

	(:action wp-discard
		:parameters (?r - robot ?cc - cap-carrier)
		:precondition (and (next-wp-discard ?r)
		                   (or (holding ?r ?cc)
		                       (can-hold ?r)
		                    )
		              )
		:effect (and (not (exog-possible))
								 (not (next-wp-discard ?r)) (last-wp-discard ?r)
								 (when (holding ?r ?cc)
								 		(and (not (holding ?r ?cc)) 
												(wp-unused ?cc) 
												(not (wp-usable ?cc))
                 				(can-hold ?r))
								 )
						)
	)

	(:action wp-get-shelf
		:parameters (?r - robot ?cc - cap-carrier ?m - mps ?spot - shelf-spot)
		:precondition (and (comp-state gripper CALIBRATED)
		                   (next-wp-get-shelf ?r ?spot)
		                   (at ?r ?m INPUT) 
		                   (or (wp-on-shelf ?cc ?m ?spot)
		                       (spot-free ?m ?spot))
		                   (can-hold ?r)
		              )
		:effect (and (not (exog-possible))
								 (not (next-wp-get-shelf ?r ?spot)) (last-wp-get-shelf ?r ?spot)
								 (when (and (wp-on-shelf ?cc ?m ?spot)
								 			 )
											 (and (holding ?r ?cc) 
											 			(not (can-hold ?r))
								 						(not (wp-on-shelf ?cc ?m ?spot))
														(wp-usable ?cc)
                 						(spot-free ?m ?spot)
											 )
								 )
						  	 (when (not (wp-on-shelf ?cc ?m ?spot))
								       (can-hold ?r)
								 )
						)
	)

	(:action wp-get
		:parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side)
		:precondition (and (next-wp-get ?r)
		                   (comp-state gripper CALIBRATED)
											 (at ?r ?m ?side)
											 (wp-usable ?wp)
											 (or (wp-at ?wp ?m ?side)
											     (mps-side-free ?m ?side)
											 )
										   (can-hold ?r)
										
									)
		:effect (and (not (exog-possible))
								 (not (next-wp-get ?r)) (last-wp-get ?r)
								 (when (and (wp-at ?wp ?m ?side)
                    				(locked ?m) 
								 			 )
											 (and (not (wp-at ?wp ?m ?side)) 
											 			(holding ?r ?wp)
											 			(mps-side-free ?m ?side)
														(not (can-hold ?r))
								 						(not (mps-state ?m READY-AT-OUTPUT)) 
														(mps-state ?m IDLE)
											 )
								 )
								 (when (not (wp-at ?wp ?m ?side))
								       (can-hold ?r)
								 )
						)
	)

	(:action wp-put
		:parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side)
		:precondition (and (next-wp-put ?r)
		                   (comp-state gripper CALIBRATED)
											 (at ?r ?m ?side)
											 (locked ?m)
											 (or (mps-side-free ?m ?side)
											 )
											 (or (holding ?r ?wp)
											     (can-hold ?r)
											 )
									)
		:effect (and (not (exog-possible))
								 (not (next-wp-put ?r)) (last-wp-put ?r)
								 (when (and (holding ?r ?wp)
								 						(wp-usable ?wp)
								 						(mps-side-free ?m ?side)
								 			 )
											 (and (wp-at ?wp ?m INPUT)
											      (not (mps-side-free ?m ?side))
											 			(not (holding ?r ?wp)) 
														(can-hold ?r)
											 )
								 )
								 (when (and (holding ?r ?wp)
								            (not (mps-side-free ?m ?side)))
								       (and (not (holding ?r ?wp))
								            (can-hold ?r)
								       )
						     )
						)
	)

	(:action wp-put-slide-cc
		:parameters (?r - robot ?wp - cap-carrier ?m - mps ?rs-before - ring-num ?rs-after - ring-num)
		:precondition (and (comp-state gripper CALIBRATED)
		                   (next-wp-put-slide ?r)
							         (mps-type ?m RS) 
							         (locked ?m)
							         (at ?r ?m INPUT)
							         (rs-filled-with ?m ?rs-before)
							         (rs-inc ?rs-before ?rs-after)
							         (or (holding ?r ?wp)
							             (can-hold ?r)
							         )
							    )
		:effect (and (not (exog-possible))
								 (not (next-wp-put-slide ?r)) (last-wp-put-slide ?r) 
					 			 (when (and (wp-usable ?wp)
								       			(holding ?r ?wp)
					 	           )
						    			 (and (not (wp-usable ?wp))
								     				(not (holding ?r ?wp))
								     				(can-hold ?r)
								     				(not (rs-filled-with ?m ?rs-before))
								     				(rs-filled-with ?m ?rs-after)
						    			 )
					 			 )	
						)
	)

	(:action fulfill-order-c0
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color)
		:precondition (and (next-fulfill-order-c0)
							(wp-at ?wp ?m INPUT)
                      		(wp-usable ?wp)
                       		(mps-type ?m DS) (locked ?m)
							(ds-prepared-gate ?m ?g)
                       		(order-gate ?ord ?g)
							(order-complexity ?ord C0)
							(order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
							(order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol)
							(wp-ring1-color ?wp RING_NONE) (wp-ring2-color ?wp RING_NONE) (wp-ring3-color ?wp RING_NONE))
		:effect (and (not (exog-possible))
(not (next-fulfill-order-c0)) (last-fulfill-order-c0)
						(order-fulfilled ?ord)
						(not (wp-at ?wp ?m INPUT))
                 		(not (ds-prepared-gate ?m ?g))
                 		(not (wp-usable ?wp))
						(not (wp-base-color ?wp ?basecol))
						(not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c1
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color)

		:precondition (and (next-fulfill-order-c1)
							(wp-at ?wp ?m INPUT) (wp-usable ?wp)
							(mps-type ?m DS) (locked ?m)
							(ds-prepared-gate ?m ?g)
                       		(order-gate ?ord ?g)
							(order-complexity ?ord C1)
							(order-base-color ?ord ?basecol)
							(wp-base-color ?wp ?basecol)
							(order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
							(order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol))
		:effect (and (not (exog-possible))
(not (next-fulfill-order-c1)) (last-fulfill-order-c1)
		 				(order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT))
                 		(not (ds-prepared-gate ?m ?g))
                 		(not (wp-usable ?wp))
						(not (wp-base-color ?wp ?basecol))
						(not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c2
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color ?ring2col - ring-color)

		:precondition (and (next-fulfill-order-c2)
							(wp-at ?wp ?m INPUT) (wp-usable ?wp)
							(mps-type ?m DS) (locked ?m)
							(ds-prepared-gate ?m ?g)
                       		(order-gate ?ord ?g)
							(order-complexity ?ord C2)
							(order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
							(order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
							(order-ring2-color ?ord ?ring2col) (wp-ring2-color ?wp ?ring2col)
							(wp-ring3-color ?wp RING_NONE)
							(order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol))
		:effect (and (not (exog-possible))
(not (next-fulfill-order-c2)) (last-fulfill-order-c2)
						(order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT))
                 		(not (ds-prepared-gate ?m ?g))
						(not (wp-base-color ?wp ?basecol))
						(not (wp-cap-color ?wp ?capcol)))

	)

	(:action fulfill-order-c3
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color ?ring2col - ring-color ?ring3col - ring-color)

		:precondition (and (last-fulfill-order-c3)
							(wp-at ?wp ?m INPUT) (wp-usable ?wp)
						 	(mps-type ?m DS) (locked ?m)
							(ds-prepared-gate ?m ?g)
                       		(order-gate ?ord ?g)
							(order-complexity ?ord C3)
							(order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
							(order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
							(order-ring2-color ?ord ?ring2col) (wp-ring2-color ?wp ?ring2col)
							(order-ring3-color ?ord ?ring3col) (wp-ring3-color ?wp ?ring3col)
							(order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol)
							)
		:effect (and (not (exog-possible))
								 (not (next-fulfill-order-c3)) (last-fulfill-order-c3)
								 (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT))
                 (not (ds-prepared-gate ?m ?g))
                 (not (wp-usable ?wp))
								 (not (wp-base-color ?wp ?basecol)) 
								 (not (wp-cap-color ?wp ?capcol))
						)
	)


  (:action refill-shelf
    :parameters (?m - mps ?spot - shelf-spot ?cc1 - cap-carrier
                 ?color - cap-color)
    :precondition (and (spot-free ?m ?spot) (locked ?m)
                        (wp-unused ?cc1)
                  )
    :effect (and (not (exog-possible))
(not (spot-free ?m ?spot) )
                 (wp-on-shelf ?cc1 ?m ?spot)
                 (not (wp-unused ?cc1))
                 (wp-cap-color ?cc1  ?color)
            )
  )

  (:action lock
    :parameters (?name - object)
    :precondition (and (next-lock ?name) (not (locked ?name)))
    :effect (and (not (exog-possible))
(not (next-lock ?name)) (last-lock ?name)
					(locked ?name)
			)
  )

  (:action one-time-lock
    :parameters (?name - object)
    :precondition (and (next-one-time-lock ?name) (not (locked ?name)))
    :effect (and (not (exog-possible))
(not (next-one-time-lock ?name)) (last-one-time-lock ?name)
					(locked ?name)
			)
  )

  (:action unlock
    :parameters (?name - object)
    :precondition (and (next-unlock ?name) (locked ?name))
    :effect (and (not (exog-possible))
(not (next-unlock ?name)) (last-unlock ?name)
				(not (locked ?name))
			)
  )

  (:action eventually-unlock
    :parameters (?name - object)
    :precondition (and (next-eventually-unlock ?name) (locked ?name))
    :effect (and (not (exog-possible))
(not (next-eventually-unlock ?name)) (last-eventually-unlock ?name)
				(not (locked ?name))
			)
  )

  (:action location-lock
    :parameters (?location - mps ?side - side)
    :precondition (and (next-location-lock ?location ?side) 
						(not (location-locked ?location ?side))
				  )
    :effect (and (not (exog-possible))
(not (next-location-lock ?location ?side)) (next-location-lock ?location ?side)
					(location-locked ?location ?side)
			)
  )

  (:action location-unlock
    :parameters (?location - mps ?side - side)
    :precondition (and (next-location-unlock ?location ?side) 
						(location-locked ?location ?side)
				  )
    :effect (and (not (exog-possible))
(not (next-location-unlock ?location ?side)) (next-location-unlock ?location ?side)
					(not (location-locked ?location ?side))
			)
  )

  ; ----------------------------- action alternatives ----------------------------
  
	(:action move-stuck
    :parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
    :precondition (and (next-move ?r ?to ?to-side) 
                       (comp-state move-base LOCKED))
    :effect (and (not (exog-possible))
								 (not (next-move ?r ?to ?to-side)) (last-move ?r ?to ?to-side))
  )

  (:action move-lost
    :parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side ?real-to - mps ?real-to-side - mps-side)
    :precondition (and (next-move ?r ?to ?to-side) 
                       (at ?r ?from ?from-side) 
                       (location-free ?real-to ?real-to-side) 
                       (comp-state navgraph INIT))
    :effect (and (not (exog-possible))
								 (not (next-move ?r ?to ?to-side))
                 (last-move ?r ?to ?to-side)
                 (not (at ?r ?from ?from-side))
                 (at ?r ?real-to ?real-to-side) 
                 (not (location-free ?real-to ?real-to-side))
                 (location-free ?from from-side))
    )

  (:action wp-get-decalib
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side)
    :precondition (and (next-wp-get ?r) 
											 (comp-state gripper UNCALIBRATED) 
											 (at ?r ?m ?side)
											 (or (wp-at ?wp ?m ?side)
											 		 (mps-side-free ?m ?side))
									)
    :effect (and (not (exog-possible))
								 (not (next-wp-get ?r)) (last-wp-get ?r)
             		 (when (wp-at ?wp ?m ?side) 
											 (not (wp-at ?wp ?m ?side))
								 )
			)
  )

  (:action wp-put-decalib
    :parameters (?r - robot ?wp - workpiece ?m - mps )
    :precondition (and (next-wp-put ?r) 
                       (comp-state gripper UNCALIBRATED) 
                       (at ?r ?m INPUT)
                       (or (holding ?r ?wp)
                           (can-hold ?r)
                       )
                  )
    :effect (and (not (exog-possible))
            		 (when (holding ?r ?wp) 
                			 (and (not (holding ?r ?wp))
											 			(can-hold ?r)
											 			(not (wp-usable ?wp))
											 )
								 )
                 (not (next-wp-put ?r))
                 (last-wp-put ?r) 
                 (can-hold ?r)
            )
  )

   (:action wp-put-axis-broken
    :parameters (?r - robot ?wp - workpiece ?m - mps )
    :precondition (and (next-wp-put ?r) 
                       (comp-state gripper AXIS-BROKEN) 
                       (at ?r ?m INPUT)
                       (or (holding ?r ?wp)
                           (can-hold ?r)
                       )
                  )
    :effect (and (not (exog-possible))
								 (not (next-wp-put ?r )) 
                 (last-wp-put ?r)
                 (when (holding ?r ?wp)
                       (and (not (holding ?r ?wp))
                            (can-hold ?r)
                            (not (wp-usable ?wp))
                       )
                 )
            )
    )
 

  (:action wp-get-shelf-decalib
		:parameters (?r - robot ?cc - cap-carrier ?m - mps ?spot - shelf-spot)
		:precondition (and (next-wp-get-shelf ?r ?spot)
		                   (comp-state gripper UNCALIBRATED) 
		                   (at ?r ?m INPUT)
                       (or (wp-on-shelf ?cc ?m ?spot)
                           (spot-free ?m ?spot)
                       )
		              )
		:effect (and (not (exog-possible))
								 (not (next-wp-get-shelf ?r ?spot)) (last-wp-get-shelf ?r ?spot)
								 (when (and (wp-on-shelf ?cc ?m ?spot)
								 			 )
											 (and (not (wp-on-shelf ?cc ?m ?spot))
                 						(spot-free ?m ?spot)
											 )
								 )
						)
		
	)

  (:action wp-get-shelf-axis-broken
		:parameters (?r - robot ?cc - cap-carrier ?m - mps ?spot - shelf-spot)
		:precondition (and (next-wp-get-shelf ?r ?spot)
		                   (comp-state gripper AXIS-BROKEN) 
		                   (at ?r ?m INPUT)
                       (or (wp-on-shelf ?cc ?m ?spot)
                           (spot-free ?m ?spot)
                       )
		              )
		:effect (and (not (exog-possible))
								 (not (next-wp-get-shelf ?r ?spot)) (last-wp-get-shelf ?r ?spot)
						)
		
	)
; ----------------------------- exog actions ------------------------------

;  (:action drop
;    :parameters (?r - robot ?wp - workpiece)
;    :precondition (and (exog-possible) (holding ?r ?wp))
;    :effect (and (not (holding ?r ?wp))
;								 (can-hold ?r)
;            (increase (total-cost) 1))
;  )

  (:action break_gripper_axis
 	 :parameters ()
   :precondition (and (exog-possible) 
	 										(or (comp-state gripper CALIBRATED) 
											(comp-state gripper UNCALIBRATED) ))
   :effect (and (when (comp-state gripper CALIBRATED)
  										(and (not (comp-state gripper CALIBRATED)) 
													 (comp-state gripper AXIS-BROKEN) )
 								) 
 								(when (comp-state gripper UNCALIBRATED)
  										(and (not (comp-state gripper UNCALIBRATED)) 
											(comp-state gripper AXIS-BROKEN) )
 								)  
 								(increase (total-cost) 1)
 					 )
	)

(:action break_gripper_fingers
 :parameters ()
 :precondition (and (exog-possible) (or (comp-state gripper CALIBRATED) (comp-state gripper UNCALIBRATED)))
 :effect (and 
 (when (comp-state gripper CALIBRATED)
  (and (not (comp-state gripper CALIBRATED)) (comp-state gripper FINGERS-BROKEN) )
 ) 
 (when (comp-state gripper UNCALIBRATED)
  (and (not (comp-state gripper UNCALIBRATED)) (comp-state gripper FINGERS-BROKEN) )
 )  
 (increase (total-cost) 1)
 )
)

(:action break_move_base
 :parameters ()
 :precondition (and (exog-possible) (or (comp-state move-base INIT) (comp-state move-base LOCKED) ))
 :effect (and 
 (when (comp-state move-base INIT)
  (and (not (comp-state move-base INIT)) (comp-state move-base FAILED) )
 ) 
 (when (comp-state move-base LOCKED)
  (and (not (comp-state move-base LOCKED)) (comp-state move-base FAILED) )
 )  
 (increase (total-cost) 1)
 )
)

(:action calibrate
 :parameters ()
 :precondition (and (exog-possible) (or (comp-state gripper UNCALIBRATED) ))
 :effect (and 
 (when (comp-state gripper UNCALIBRATED)
  (and (not (comp-state gripper UNCALIBRATED)) (comp-state gripper CALIBRATED) )
 )  
 (increase (total-cost) 1)
 )
)

(:action localize
 :parameters ()
 :precondition (and (exog-possible) (or (comp-state navgraph INIT) ))
 :effect (and 
 (when (comp-state navgraph INIT)
  (and (not (comp-state navgraph INIT)) (comp-state navgraph LOCALIZED) )
 )  
 (increase (total-cost) 1)
 )
)

(:action lock-move-base
 :parameters ()
 :precondition (and (exog-possible) (or (comp-state move-base INIT)))
 :effect (and (when (comp-state move-base INIT)
  									(and (not (comp-state move-base INIT)) 
												 (comp-state move-base LOCKED)
										)
 							)  
 							(increase (total-cost) 1)
 				 )
)

(:action uncalibrate
 :parameters ()
 :precondition (or (exog-possible) (comp-state gripper CALIBRATED) )
 :effect (and 
 (when (comp-state gripper CALIBRATED)
  (and (not (comp-state gripper CALIBRATED)) (comp-state gripper UNCALIBRATED) )
 )  
 (increase (total-cost) 1)
 )
)

(:action unlocalize
 :parameters ()
 :precondition (and (exog-possible) (or (comp-state navgraph LOCALIZED) ))
 :effect (and 
 (when (comp-state navgraph LOCALIZED)
  (and (not (comp-state navgraph LOCALIZED)) (comp-state navgraph INIT) )
 )  
 (increase (total-cost) 1)
 )
)

(:action unlock-movebase
 :parameters ()
 :precondition (and (exog-possible) (or (comp-state move-base LOCKED) ))
 :effect (and 
 (when (comp-state move-base LOCKED)
  (and (not (comp-state move-base LOCKED)) (comp-state move-base INIT) )
 )  
 (increase (total-cost) 1)
 )
)


  <<#order-actions>>

)
