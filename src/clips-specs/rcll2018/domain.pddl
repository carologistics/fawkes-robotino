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
		component - location
    mps - component
	)

	(:constants
		START - location
		BS CS DS RS SS - mps-typename
		;M-BS C-BS M-CS1 C-CS1 M-CS2 C-CS2 M-RS1 C-RS1 M-RS2 C-RS2 M-DS C-DS M-SS C-SS - object
		;INPUT OUTPUT - mps-side
		BASE_NONE BASE_RED BASE_BLACK BASE_SILVER - base-color
		CAP_NONE CAP_BLACK CAP_GREY - cap-color
		GATE-1 GATE-2 GATE-3 - ds-gate
		RING_NONE RING_BLUE RING_GREEN RING_ORANGE RING_YELLOW - ring-color
		RETRIEVE_CAP MOUNT_CAP - cs-operation
		C0 C1 C2 C3 - order-complexity-value
		LEFT MIDDLE RIGHT - shelf-spot
		NA ZERO ONE TWO THREE - ring-num
		BROKEN - state
		IDLE - state
		DOWN-TO-IDLE - state
		DOWN-TO-READY-AT-OUTPUT - state
		READY-AT-OUTPUT - state
		PREPARED - state
		CONNECTION-ESTABLISHED - state
		CONNECTION-LOST - state
		CALIBRATED - state
		AXIS-BROKEN - state
		FINGERS-BROKEN - state
		UNCALIBRATED - state
		ACTIVATED - state		
		LOST - state
		INIT - state
		FAILED - state
		LOCKED - state
		LOCALIZED - state
		DEACTIVATED - state
		C-BS - mps
		C-CS1 - mps
		C-CS2 - mps
		C-RS1 - mps
    C-RS2 - mps
    C-DS - mps
		communication - component
		gripper - component
		laser - component
		move-base - component
		navgraph - component
		realsense - component
		tag-camera - component
	)

	(:predicates
		(self ?r - robot)
		(at ?r - robot ?m - location ?side - mps-side)
		(holding ?r - robot ?wp - workpiece)
		(can-hold ?r - robot)
		(entered-field ?r - robot)
		(robot-waiting ?r - robot)
		(mps-type ?m - mps ?t - mps-typename)
		(mps-team ?m - mps ?col - team-color)
		(bs-prepared-color ?m - mps ?col - base-color)
		(bs-prepared-side ?m - mps ?side - mps-side)
		(cs-can-perform ?m - mps ?op - cs-operation)
		(cs-prepared-for ?m - mps ?op - cs-operation)
		(cs-buffered ?m - mps ?col - cap-color)
		(cs-color ?m - mps ?col - cap-color)
		(mps-side-free ?m - mps ?side - mps-side)
		(mps-state ?m - mps ?s - mps-state)
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
    	(spot-free ?m - mps ?spot - shelf-spot)
		(comp-state ?c - component ?s - state)
    	(dummy-wp ?wp - workpiece)

    	(exog-possible)
		(last-begin)
		(next-finish)

   		(locked ?name - object)
    	(location-locked ?m - mps ?s - mps-side)

		(next-reset-mps ?m - mps)
		(last-reset-mps ?m - mps)

		(next-prepare-bs ?m - mps ?side - mps-side ?bc - base-color)
		(last-prepare-bs ?m - mps ?side - mps-side ?bc - base-color)

		(next-prepare-ds ?m - mps ?ord - order)
		(last-prepare-ds ?m - mps ?ord - order)

		(next-prepare-cs ?m - mps ?op - cs-operation)
		(last-prepare-cs ?m - mps ?op - cs-operation)

		(next-bs-dispense ?r - robot ?m - mps ?side - mps-side ?wp - workpiece ?basecol - base-color)
		(last-bs-dispense ?r - robot ?m - mps ?side - mps-side ?wp - workpiece ?basecol - base-color)
	
		(next-cs-mount-cap ?m - mps)
		(last-cs-mount-cap ?m - mps)

		(next-cs-retrieve-cap ?m - mps)
		(last-cs-retrieve-cap ?m - mps)
	
		(next-prepare-rs ?m - mps ?rc - ring-color)
		(last-prepare-rs ?m - mps ?rc - ring-color)

		(next-rs-mount-ring1 ?m - mps)
		(last-rs-mount-ring1 ?m - mps)
	
		(next-rs-mount-ring2 ?m - mps)
		(last-rs-mount-ring2 ?m - mps)
	
		(next-rs-mount-ring3 ?m - mps)
		(last-rs-mount-ring3 ?m - mps)
	
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

		(next-wp-put-slide-cc ?r - robot)
		(last-wp-put-slide-cc ?r - robot)

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
		:parameters (?m - mps ?state - mps-state)
		:precondition (and (comp-state ?m ?state)
		                   (comp-state communication CONNECTION-ESTABLISHED)
		              )
		:effect (and (comp-state ?m BROKEN) (not (comp-state ?m ?state))
								 (not (exog-possible)))
	)

	(:action prepare-bs
		:parameters (?m - mps ?side - mps-side ?bc - base-color)
		:precondition (and (comp-state communication CONNECTION-ESTABLISHED)
											 (mps-type ?m BS)
                       (comp-state ?m IDLE)
					)
		:effect (and (when (and (comp-state ?m IDLE))
								 			 (and (not (comp-state ?m IDLE)) (comp-state ?m READY-AT-OUTPUT)
								 			 			(bs-prepared-color ?m ?bc) 
                            (bs-prepared-side ?m ?side))
								 )
                 (when (comp-state ?m READY-AT-OUTPUT)
                       (and (bs-prepared-color ?m ?bc) 
                            (bs-prepared-side ?m ?side))
                 )
								 (not (exog-possible))
						)
	)

	(:action prepare-ds
		:parameters (?m - mps ?ord - ds-order)
		:precondition (and (comp-state communication CONNECTION-ESTABLISHED)
											 (mps-type ?m DS)
                       (comp-state ?m IDLE)
									)
		:effect (and (when (not (mps-side-free ?m INPUT))
                       (and (not (comp-state ?m IDLE))
                            (comp-state ?m PROCESSED)
                            (ds-prepared-order ?m ?ord)
                       )
                 )
                 (when (mps-side-free ?m INPUT)
                       (and (not (comp-state ?m IDLE))
                            (comp-state ?m PREPARED)
                            (ds-prepared-order ?m ?ord)
                       )
                 )
                 (not (exog-possible))
						)
	)

	(:action prepare-cs
		:parameters (?m - mps ?op - cs-operation)
		:precondition (and (comp-state communication CONNECTION-ESTABLISHED)
					             (mps-type ?m CS)
                       (comp-state ?m IDLE)
                  )
		:effect (and (when (and (mps-side-free ?m INPUT)
                        		(cs-can-perform ?m ?op)
											 )
                       (and (not (cs-can-perform ?m ?op))
                             (cs-prepared-for ?m ?op)
                             (not (comp-state ?m IDLE))
                             (comp-state ?m PREPARED)
                       )
                 )
                 (when (and (not (mps-side-free ?m INPUT))
                             (cs-can-perform ?m ?op))
                        (and (not (cs-can-perform ?m ?op))
                              (cs-prepared-for ?m ?op)
                              (not (comp-state ?m IDLE))
                              (comp-state ?m READY-AT-OUPUT)
                        )
                )
                 (not (exog-possible))
				    )
	)

	(:action bs-dispense
		:parameters (?r - robot ?m - mps ?side - mps-side ?wp - workpiece ?basecol - base-color)
		:precondition (and (mps-type ?m BS)
											 (comp-state ?m READY-AT-OUTPUT)
											 (bs-prepared-color ?m ?basecol)
                       (bs-prepared-side ?m ?side)
											 (wp-base-color ?wp BASE_NONE)
											 (wp-unused ?wp)
											 (wp-spawned-for ?wp ?r)
									)
											 ;(not (wp-usable ?wp))
		:effect (and (wp-at ?wp ?m ?side)
								 (not (mps-side-free ?m ?side))
								 (not (wp-base-color ?wp BASE_NONE)) (wp-base-color ?wp ?basecol)
								 (not (wp-unused ?wp)) (wp-usable ?wp)
								 (not (wp-spawned-for ?wp ?r))
                 (not (exog-possible))
						)
	)

	(:action cs-mount-cap
		:parameters (?m - mps ?wp - workpiece ?capcol - cap-color)
		:precondition (and (mps-type ?m CS)
											 (comp-state ?m READY-AT-OUTPUT)
											 (cs-buffered ?m ?capcol)
											 (cs-prepared-for ?m MOUNT_CAP)
											 (wp-usable ?wp)
											 (wp-at ?wp ?m INPUT)
											 (wp-cap-color ?wp CAP_NONE)
									)
		:effect (and (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT)
								 (wp-at ?wp ?m OUTPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-cap-color ?wp CAP_NONE)) (wp-cap-color ?wp ?capcol)
								 (not (cs-can-perform ?m MOUNT_CAP)) (cs-can-perform ?m RETRIEVE_CAP)
								 (not (cs-prepared-for ?m MOUNT_CAP))
								 (not (cs-buffered ?m ?capcol))
                 (not (exog-possible))
            )
	)

	(:action cs-retrieve-cap
		:parameters (?m - mps ?cc - cap-carrier ?capcol - cap-color)
		:precondition (and (mps-type ?m CS) 
											 (comp-state ?m READY-AT-OUTPUT)
											 (cs-prepared-for ?m RETRIEVE_CAP)
											 (wp-at ?cc ?m INPUT)
										   (wp-cap-color ?cc ?capcol)
									)
		:effect (and (not (wp-at ?cc ?m INPUT)) (wp-at ?cc ?m OUTPUT)
								 (mps-side-free ?m INPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-cap-color ?cc ?capcol)) (wp-cap-color ?cc CAP_NONE)
								 (cs-buffered ?m ?capcol)
								 (cs-can-perform ?m MOUNT_CAP)
								 (not (cs-prepared-for ?m RETRIEVE_CAP))
						     (not (exog-possible))
						)
	)

	(:action prepare-rs
		:parameters (?m - mps ?rc - ring-color ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (comp-state communication CONNECTION-ESTABLISHED)
											 (mps-type ?m RS)
                       (comp-state ?m IDLE)
                       (rs-ring-spec ?m ?rc ?r-req)
                       (rs-sub ?rs-before ?r-req ?rs-after)
									)
		:effect (and (when (and (mps-side-free ?m INPUT)
                            (rs-filled-with ?m ?rs-before))
                       (and (not (comp-state ?m IDLE))
                             (comp-state ?m READY-AT-OUTPUT)
                             (rs-prepared-color ?m ?rc)
                       )           
                 )
                 (when (and (not (mps-side-free ?m INPUT))
                            (rs-filled-with ?m ?rs-before))
                       (and (not (comp-state ?m IDLE))
                             (comp-state ?m PREPARED)
                             (rs-prepared-color ?m ?rc)
                       )           
                 )
                 (not (exog-possible))
						)
	)

	(:action rs-mount-ring1
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) 
                    (comp-state ?m READY-AT-OUTPUT)
										(wp-at ?wp ?m INPUT) (wp-usable ?wp)
										(wp-ring1-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
								 (mps-side-free ?m INPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-ring1-color ?wp RING_NONE)) (wp-ring1-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after))
	)

	(:action rs-mount-ring2
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color
					?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS)
                    (comp-state ?m READY-AT-OUTPUT)
										(wp-at ?wp ?m INPUT) (wp-usable ?wp)
										(wp-ring1-color ?wp ?col1)
										(wp-ring2-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and (not (exog-possible))
		             (not (rs-prepared-color ?m ?col))
								 (mps-side-free ?m INPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
								 (not (wp-ring2-color ?wp RING_NONE)) (wp-ring2-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after)
						)
	)

	(:action rs-mount-ring3
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color ?col2 - ring-color
							?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) (comp-state ?m READY-AT-OUTPUT)
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
		             (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
							   (mps-side-free ?m INPUT) (not (mps-side-free ?m OUTPUT))
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
		:precondition (and (comp-state move-base INIT)
								       (comp-state navgraph LOCALIZED)
								       (comp-state laser ACTIVATED)
                       (at ?r ?from ?from-side))
		:effect (and (not (exog-possible))
								 (not (at ?r ?from ?from-side)) (at ?r ?to WAIT)
						)
	)

  (:action wait
    :parameters (?r - robot ?point - waitpoint)
    :precondition (and (at ?r ?point WAIT))
    :effect (and (not (exog-possible)) )
  )

	(:action move
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
		:precondition (and (comp-state move-base INIT)
											 (comp-state navgraph LOCALIZED)
											 (comp-state laser ACTIVATED)
											 (at ?r ?from ?from-side)
									)
		:effect (and (not (exog-possible))
								 (not (at ?r ?from ?from-side)) (at ?r ?to ?to-side)
						)
	)

	(:action enter-field
		:parameters (?r - robot)
		:precondition (and (at ?r START INPUT)
											 (robot-waiting ?r)
											 (comp-state move-base INIT)
											 (comp-state navgraph LOCALIZED)
											 (comp-state laser ACTIVATED)
									)
		:effect (and (not (exog-possible))
								 (entered-field ?r)
								 (at ?r START INPUT)
								 (not (robot-waiting ?r)))
	)

	(:action wp-discard
		:parameters (?r - robot ?cc - cap-carrier)
		:precondition (and (or (holding ?r ?cc)
		                      (and (can-hold ?r) (dummy-wp ?cc))
		                   )
											 (not (comp-state gripper FINGERS-BROKEN))
		              )
		:effect (and (not (exog-possible))
								 (when (holding ?r ?cc)
								 		   (and (not (holding ?r ?cc)) 
											    (wp-unused ?cc) 
												  (not (wp-usable ?cc))
                 				  (can-hold ?r))
								 )
						)
	)

	(:action wp-get-shelf
		:parameters (?r - robot ?cc - cap-carrier ?m - mps ?side - mps-side ?spot - shelf-spot ?hold - workpiece)
		:precondition (and (comp-state gripper CALIBRATED)
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
		                   (at ?r ?m ?side) 
                       ; Either the cc is the grounded for the cc at the shelf spot
                       ; or the shelf spot is clear and the cc is grounded as the dummy wp
		                   (or (wp-on-shelf ?cc ?m ?spot)
		                       (and (spot-free ?m ?spot) (dummy-wp ?cc)))
                       ; Either the cc is the grounded for the cc currently held by the robot
                       ; or the gripper is clear and the cc is grounded as the dummy wp
											 (or (and (can-hold ?r) (dummy-wp ?hold))
											 		 (holding ?r ?hold)
											 )
		              )
		:effect (and (not (exog-possible))
								 ; Robot is at a CS and there is something at the shelf spot
                 (when (and (wp-on-shelf ?cc ?m ?spot)
								 						(mps-type ?m CS)
                            (at ?r ?m INPUT)
								 			 )
											 (and (holding ?r ?cc) 
											 			(not (can-hold ?r))
								 						(not (wp-on-shelf ?cc ?m ?spot))
														(wp-usable ?cc)
                 						(spot-free ?m ?spot)
											 )
								 )
                 ; There is nothing at the shelf spot -> Gripper is clear
						  	 (when (or (not (wp-on-shelf ?cc ?m ?spot))
                           (not (at ?r ?m INPUT)))
								       (can-hold ?r)
								 )
                 ; If something was in the gripper, it gets dropped
								 (when (holding ?r ?hold)
								 				(and (not (holding ?r ?hold))
												 		 (not (wp-usable ?hold))
												)
								 )
						)
	)	

	(:action wp-get
		:parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?hold - workpiece)
		:precondition (and (comp-state gripper CALIBRATED)
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
											 (comp-state ?m READY-AT-OUTPUT)
											 (at ?r ?m ?side)
											 (wp-usable ?wp)
                       ; Either the wp is grounded by the workpiece at the mps side
                       ; or the mps side is free and the wp is grounded by the dummy wp
											 (or (wp-at ?wp ?m ?side)
											     (and (mps-side-free ?m ?side) (dummy-wp ?wp))
											 )
                       ; Either the hold wp is grounded by the wp currently held by the robot
                       ; or the robot does not hold anything and its grounded by the dummy wp
											 (or (and (can-hold ?r) (dummy-wp ?hold))
											 		 (holding ?r ?hold)
											 )								
									)
		:effect (and (not (exog-possible))
								 ; There is something at the mps side which is grabbed
								 (when (and (wp-at ?wp ?m ?side)
								 			 )
											 (and (not (wp-at ?wp ?m ?side)) 
											 			(holding ?r ?wp)
											 			(mps-side-free ?m ?side)
														(not (can-hold ?r))
								 						(not (comp-state ?m READY-AT-OUTPUT)) 
														(comp-state ?m IDLE)
											 )
								 )
                 ; If the mps side is clear, the robots gripper is free as a result
								 (when (not (wp-at ?wp ?m ?side))
								       (can-hold ?r)
								 )
                 ; Drop a workpiece if it was in the gripper while gripping
								 (when (holding ?r ?hold)
								 			 (and (can-hold ?r) (not (wp-usable ?hold)))
								 )
						)
	)


	(:action wp-put
		:parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?there - workpiece)
		:precondition (and (comp-state gripper CALIBRATED)
						           (comp-state realsense ACTIVATED)
						           (comp-state tag-camera ACTIVATED)
						           (comp-state laser ACTIVATED)
											 (at ?r ?m ?side)
											 (or (and (mps-side-free ?m ?side) (dummy-wp ?there))
                           (wp-at ?there ?m ?side)
											 )
											 (or (holding ?r ?wp)
											     (and (can-hold ?r) (dummy-wp ?wp))
											 )
									)
		:effect (and (not (exog-possible))
								 (when (and (holding ?r ?wp)
								 			 )
											 (and (wp-at ?wp ?m ?side)
											      (not (mps-side-free ?m ?side))
											 			(not (holding ?r ?wp)) 
														(can-hold ?r)
											 )
								 )
								 (when (and (not (mps-side-free ?m ?side))
                            (wp-at ?there ?m ?side))
								       (and (not (wp-at ?there ?m ?side))
                            (not (wp-usable ?there))
								       )
						     )
						)
	)


	(:action wp-put-slide-cc
		:parameters (?r - robot ?wp - cap-carrier ?m - mps ?side - mps-side ?rs-before - ring-num ?rs-after - ring-num)
		:precondition (and (comp-state gripper CALIBRATED)
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
		                   ; The bot searches for the slide, therefore the action
                       ; would fail if the mps is not a RS or the robot is not at the input side
                       (mps-type ?m RS) 
							         (at ?r ?m INPUT)

							         (rs-filled-with ?m ?rs-before)
							         (rs-inc ?rs-before ?rs-after)
							         (or (holding ?r ?wp)
							             (and (can-hold ?r) (dummy-wp ?wp))
							         )
							    )
		:effect (and (not (exog-possible))
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
		:precondition (and (wp-at ?wp ?m INPUT)
                       (wp-base-color ?wp ?basecol)
                       (wp-cap-color ?wp ?capcol)
							         (wp-ring1-color ?wp RING_NONE) 
                       (wp-ring2-color ?wp RING_NONE) 
                       (wp-ring3-color ?wp RING_NONE)
                       (wp-usable ?wp)
                       (mps-type ?m DS)
							         (ds-prepared-order ?m ?ord)
							         (order-complexity ?ord C0)
                  )
		:effect (and (not (exog-possible))
						     (when (and (order-base-color ?ord ?basecol) 
							              (order-cap-color ?ord ?capcol)
                       ) 
                       (and (order-fulfilled ?ord)
                       )
                 )

						     (not (wp-at ?wp ?m INPUT))
						     (mps-side-free ?m INPUT)
                 (not (ds-prepared-order ?m ?ord))
                 (not (wp-usable ?wp))
						     (not (wp-base-color ?wp ?basecol))
						     (not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c1
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) 
                       (wp-usable ?wp)
							         (mps-type ?m DS)
							         (ds-prepared-order ?m ?ord)
                       
							         (order-complexity ?ord C1)
							         
							         (wp-base-color ?wp ?basecol)
                       (wp-ring1-color ?wp ?ring1col)
                       (wp-ring2-color ?wp RING_NONE)
                       (wp-ring3-color ?wp RING_NONE)
                       (wp-cap-color ?wp ?capcol))
		:effect (and (not (exog-possible))
                 (when (and (order-base-color ?ord ?basecol)
                            (order-ring1-color ?ord ?ring1col)
                            (order-cap-color ?ord ?capcol)
                       )
                       (order-fulfilled ?ord)
                 )

                 (not (wp-at ?wp ?m INPUT))
						 		 (mps-side-free ?m INPUT)
                 (not (ds-prepared-order ?m ?ord))
                 (not (wp-usable ?wp))
						     (not (wp-base-color ?wp ?basecol))
						     (not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c2
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color ?ring2col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) 
                       (wp-usable ?wp)
							         (mps-type ?m DS)
							         (ds-prepared-order ?m ?ord)
                       
							         (order-complexity ?ord C2)
							         
                       (wp-base-color ?wp ?basecol)
                       (wp-ring1-color ?wp ?ring1col)
                       (wp-ring2-color ?wp ?ring2col)
							         (wp-ring3-color ?wp RING_NONE)
                       (wp-cap-color ?wp ?capcol))
		:effect (and (not (exog-possible))
                 (when (and (order-base-color ?ord ?basecol) 
                            (order-ring1-color ?ord ?ring1col) 
                            (order-ring2-color ?ord ?ring2col)
                            (order-cap-color ?ord ?capcol))
                       (order-fulfilled ?ord)
                 )
                 (not (wp-at ?wp ?m INPUT))
                 (not (wp-usable ?wp))
								 (mps-side-free ?m INPUT)
                 (not (ds-prepared-order ?m ?ord))
						     (not (wp-base-color ?wp ?basecol))
						     (not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c3
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color ?ring2col - ring-color ?ring3col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) (wp-usable ?wp)
						 	 (mps-type ?m DS) (locked ?m)
							 (ds-prepared-order ?m ?ord)
							 (order-complexity ?ord C3)

							 (wp-base-color ?wp ?basecol)
							 (wp-ring1-color ?wp ?ring1col)
							 (wp-ring2-color ?wp ?ring2col)
							 (wp-ring3-color ?wp ?ring3col)
							 (wp-cap-color ?wp ?capcol)
							)
		:effect (and (not (exog-possible))
								 (when (and (order-base-color ?ord ?basecol)
                            (order-ring1-color ?ord ?ring1col)
                            (order-ring2-color ?ord ?ring2col)
                            (order-ring3-color ?ord ?ring3col)
                            (order-cap-color ?ord ?capcol))
								       (order-fulfilled ?ord)                             
                 )

                 (not (wp-at ?wp ?m INPUT))
								 (mps-side-free ?m INPUT)
                 (not (ds-prepared-order ?m ?ord))
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
    :precondition (and (comp-state communication CONNECTION-ESTABLISHED) 
                      (not (locked ?name))
                  )
    :effect (and (not (exog-possible))
					      (locked ?name)
			      )
  )

  (:action one-time-lock
    :parameters (?name - object)
    :precondition (and (comp-state communication CONNECTION-ESTABLISHED) 
                       (not (locked ?name)))
    :effect (and (not (exog-possible))
                 (locked ?name)
			      )
  )

  (:action unlock
    :parameters (?name - object)
    :precondition (and (comp-state communication CONNECTION-ESTABLISHED) 
                        (locked ?name))
    :effect (and (not (exog-possible))
                 (not (locked ?name))
			      )
  )

  (:action eventually-unlock
    :parameters (?name - object)
    :precondition (and (comp-state communication CONNECTION-ESTABLISHED) 
                       (locked ?name))
    :effect (and (not (exog-possible))
                 (not (locked ?name))
			      )
  )

  (:action location-lock
    :parameters (?location - mps ?side - mps-side)
    :precondition (and (comp-state communication CONNECTION-ESTABLISHED) 
                       (not (location-locked ?location ?side))
				          )
    :effect (and (not (exog-possible))
                 (location-locked ?location ?side)
			)
  )

  (:action location-unlock
    :parameters (?location - mps ?side - mps-side)
    :precondition (and (comp-state communication CONNECTION-ESTABLISHED) 
                       (location-locked ?location ?side)
				          )
    :effect (and (not (exog-possible))
                 (not (location-locked ?location ?side))
			)
  )

  ; ----------------------------- action alternatives ----------------------------
 (:action move-navgraph-init
    :parameters (?r - robot ?from - location ?from-side - mps-side 
                ?to - mps ?to-side - mps-side ?real-to - mps ?real-to-side - mps-side)
    :precondition (and (at ?r ?from ?from-side) 
											 (not (or (comp-state move-base LOCKED) (comp-state move-base FAILED)))
                       (comp-state navgraph INIT))
    :effect (and (not (exog-possible))
								 (not (at ?r ?from ?from-side))
                 (at ?r ?real-to ?real-to-side))
 )

  (:action wp-get-gripper-uncalibrated-only-loose
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?hold - workpiece)
    :precondition (and (comp-state gripper UNCALIBRATED)
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
											 (at ?r ?m ?side)
											 (or (wp-at ?wp ?m ?side)
											 		 (and (mps-side-free ?m ?side) (dummy-wp ?wp))
											 )
											 (or (holding ?r ?hold)
											 		 (and (can-hold ?r) (dummy-wp ?hold)))
									)
    :effect (and (not (exog-possible))
								 (when (holding ?r ?hold)
								 			 (and (not (holding ?r ?hold))
											 			(not (wp-usable ?hold))
														(can-hold ?r)
											 )
								 )
			)
  )

   (:action wp-get-gripper-uncalibrated-only-loose-knock-off
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?hold - workpiece)
    :precondition (and (comp-state gripper UNCALIBRATED)
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
											 (at ?r ?m ?side)
											 (or (wp-at ?wp ?m ?side)
											 		 (and (mps-side-free ?m ?side) (dummy-wp ?wp))
											 )
											 (or (holding ?r ?hold)
											 		 (and (can-hold ?r) (dummy-wp ?hold)))
									)
    :effect (and (not (exog-possible))
								 (when (wp-at ?wp ?m ?side) 
											 (and (not (wp-at ?wp ?m ?side))
											 			 (mps-side-free ?m ?side)
                            (not (comp-state ?m READY-AT-OUTPUT))
                            (comp-state ?m IDLE))
								 )
								 (when (holding ?r ?hold)
								 			 (and (not (holding ?r ?hold))
											 			(not (wp-usable ?hold))
														(can-hold ?r)
											 )
								 )
			)
  )

  (:action wp-get-gripper-axis-broken
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?hold - workpiece)
    :precondition (and (comp-state gripper AXIS-BROKEN) 
											 	(comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
											 (at ?r ?m ?side)
											 (or (wp-at ?wp ?m ?side)
											 		 (and (mps-side-free ?m ?side) (dummy-wp ?wp))
											 )
											 (or (holding ?r ?hold)
											 		 (and (can-hold ?r) (dummy-wp ?hold)))
									)
    :effect (and (not (exog-possible))
								 (when (holding ?r ?hold)
								 			 (and (not (holding ?r ?hold))
											 			(not (wp-usable ?hold))
														 (can-hold ?r)
											 )
								 )
			)
  )

	(:action wp-get-gripper-finger-broken
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?hold - workpiece)
    :precondition (and (comp-state gripper FINGERS-BROKEN) 
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
											 (at ?r ?m ?side)
											 (or (wp-at ?wp ?m ?side)
											 		 (and (mps-side-free ?m ?side) (dummy-wp ?wp))
											 )
											 (or (holding ?r ?hold)
											 		 (and (can-hold ?r) (dummy-wp ?hold)))
									)
    :effect (and (not (exog-possible))
								 (when (wp-at ?wp ?m ?side) 
											 (and (not (wp-at ?wp ?m ?side))
											 			(mps-side-free ?m ?side)
                            (not (comp-state ?m READY-AT-OUTPUT))
                            (comp-state ?m IDLE))
								 )
			)
  )


  (:action wp-put-gripper-decalibrated-only-loose
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?there - workpiece)
    :precondition (and (comp-state gripper UNCALIBRATED)
											 	(comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
                       (at ?r ?m ?side)
                       (or (holding ?r ?wp)
                           (and (can-hold ?r) (dummy-wp ?wp))
                       )
											 (or (wp-at ?there ?m ?side)
											 		 (and (mps-side-free ?m ?side) (dummy-wp ?wp)))
                  )
    :effect (and (not (exog-possible))
            		 (when (holding ?r ?wp) 
                			 (and (not (holding ?r ?wp))
											 			(can-hold ?r)
											 			(not (wp-usable ?wp))
											 )
								 )
            )
  )
  
  (:action wp-put-gripper-decalibrated-knock-off
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?there - workpiece)
    :precondition (and (comp-state gripper UNCALIBRATED)
											 	(comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
                       (at ?r ?m ?side)
                       (or (holding ?r ?wp)
                           (and (can-hold ?r) (dummy-wp ?wp))
                       )
											 (or (wp-at ?there ?m ?side)
											 		 (and (mps-side-free ?m ?side) (dummy-wp ?wp)))
                  )
    :effect (and (not (exog-possible))
            		 (when (holding ?r ?wp) 
                			 (and (not (holding ?r ?wp))
											 			(can-hold ?r)
											 			(not (wp-usable ?wp))
											 )
								 )
								 (when (wp-at ?there ?m ?side)
								 			 (and (not (wp-at ?there ?m ?side))
														(mps-side-free ?m ?side))
								 )
                 )
  )
  

   (:action wp-put-gripper-axis-broken
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?there - workpiece)
    :precondition (and (comp-state gripper AXIS-BROKEN) 
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
                       (at ?r ?m ?side)
                       (or (holding ?r ?wp)
                           (and (can-hold ?r) (dummy-wp ?wp))
                       )
											 (or (wp-at ?there ?m ?side)
											 		 (and (mps-side-free ?m ?side) (dummy-wp ?wp)))
                  )
    :effect (and (not (exog-possible))
								 (when (holding ?r ?wp)
                       (and (not (holding ?r ?wp))
                            (can-hold ?r)
                            (not (wp-usable ?wp))
                       )
                 )
            )
    )


   (:action wp-put-gripper-fingers-broken
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side ?there - workpiece)
    :precondition (and (comp-state gripper AXIS-BROKEN) 
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
                       (at ?r ?m ?side)
                       (or (holding ?r ?wp)
                           (and (can-hold ?r) (dummy-wp ?wp))
                       )
											 (or (wp-at ?there ?m ?side)
											 		 (and (mps-side-free ?m ?side) (dummy-wp ?wp)))
                  )
    :effect (and (not (exog-possible))
								 (when (wp-at ?there ?m ?side)
								 			 (and (not (wp-at ?there ?m ?side))
														(mps-side-free ?m ?side))
								 )
            )
    )
 
 	(:action wp-put-slide-cc-gripper-broken
		:parameters (?r - robot ?wp - cap-carrier ?m - mps ?side - mps-side)
		:precondition (and (or (comp-state gripper AXIS-BROKEN) (comp-state gripper UNCALIBRATED))
											 	(comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
		                   (at ?r ?m ?side)
							         (or (holding ?r ?wp)
							             (and (can-hold ?r) (dummy-wp ?wp))
							         )
							    )
		:effect (and (not (exog-possible))
								 (when (and (wp-usable ?wp)
								       			(holding ?r ?wp)
					 	           )
						    			 (and (not (wp-usable ?wp))
								     				(not (holding ?r ?wp))
								     				(can-hold ?r)
						    			 )
					 			 )	
						)
	)

	 	(:action wp-put-slide-cc-fingers-broken
		:parameters (?r - robot ?wp - cap-carrier ?m - mps ?side - mps-side)
		:precondition (and (comp-state gripper FINGERS-BROKEN)
											 	(comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
		                   (at ?r ?m ?side)
							         (or (holding ?r ?wp)
							             (and (can-hold ?r) (dummy-wp ?wp))
							         )
							    )
		:effect (and (not (exog-possible))
								 )
		)

	(:action wp-put-slide-cc-gripper-broken
		:parameters (?r - robot ?wp - cap-carrier ?m - mps ?side - mps-side)
		:precondition (and (or (comp-state gripper AXIS-BROKEN) (comp-state gripper UNCALIBRATED))
											 (comp-state realsense ACTIVATED)
											 (comp-state tag-camera ACTIVATED)
											 (comp-state laser ACTIVATED)
		                   (at ?r ?m ?side)
							         (or (holding ?r ?wp)
							             (and (can-hold ?r) (dummy-wp ?wp))
							         )
							    )
		:effect (and (not (exog-possible))
								 (when (and (wp-usable ?wp)
								       			(holding ?r ?wp)
					 	           )
						    			 (and (not (wp-usable ?wp))
								     				(not (holding ?r ?wp))
								     				(can-hold ?r)
						    			 )
					 			 )	
						)
	)

  (:action wp-get-shelf-gripper-decalibrated
		:parameters (?r - robot ?cc - cap-carrier ?m - mps ?spot - shelf-spot ?hold - workpiece)
		:precondition (and (comp-state gripper UNCALIBRATED) 
		                   (at ?r ?m INPUT)
                       (or (wp-on-shelf ?cc ?m ?spot)
                           (and (spot-free ?m ?spot) (dummy-wp ?cc))
                       )
											 (or (holding ?r ?hold)
											 		 (and (can-hold ?r) (dummy-wp ?hold))
											 )
		              )
		:effect (and (not (exog-possible))
								 (when (and (wp-on-shelf ?cc ?m ?spot)
								 			 )
											 (and (not (wp-on-shelf ?cc ?m ?spot))
                 						(spot-free ?m ?spot)
											 )
								 )
								 (when (holding ?r ?hold)
								 			 (and (not (holding ?r ?hold))
											      (not (wp-usable ?hold)))
								 )
						)
		
	)

  (:action wp-get-shelf-axis-broken
		:parameters (?r - robot ?cc - cap-carrier ?m - mps ?side - mps-side ?spot - shelf-spot ?hold - workpiece)
		:precondition (and (comp-state gripper AXIS-BROKEN) 
		                   (at ?r ?m ?side)
                       (or (wp-on-shelf ?cc ?m ?spot)
                           (and (spot-free ?m ?spot) (dummy-wp ?cc))
                       )
											 (or (holding ?r ?hold)
											 		 (and (can-hold ?r) (dummy-wp ?wp)))
		              )
		:effect (and (not (exog-possible))
								 (when (holding ?r ?hold)
								 			 (and (not (holding ?r ?hold))
												    (not (wp-usable ?r)))
								 )
						)
		
	)

	  (:action wp-get-shelf-gripper-fingers-broken
		:parameters (?r - robot ?cc - cap-carrier ?m - mps ?side - mps-side ?spot - shelf-spot ?hold - workpiece)
		:precondition (and (comp-state gripper FINGERS-BROKEN) 
		                   (at ?r ?m ?side)
                       (or (wp-on-shelf ?cc ?m ?spot)
                           (and (spot-free ?m ?spot) (dummy-wp ?cc))
                       )
											 (or (holding ?r ?hold)
											 		 (and (can-hold ?r) (dummy-wp ?hold))
											 )
		              )
		:effect (and (not (exog-possible))
								 (when (and (wp-on-shelf ?cc ?m ?spot) (at ?r ?m INPUT)
								 			 )
											 (and (not (wp-on-shelf ?cc ?m ?spot))
                 						(spot-free ?m ?spot)
											 )
								 )
						)
	)
; ----------------------------- exog actions ------------------------------

(:action check-workpiece
	:parameters (?r - robot ?m - mps ?side - mps-side)
	:precondition (and (at ?r ?m ?side)
										 (comp-state realsense ACTIVATED))
	:effect ()
)

(:action drive-to-check-workpiece
	:parameters (?r - robot ?m - mps ?side - mps-side)
	:precondition (and (comp-state realsense ACTIVATED)
										 (comp-state move-base INIT)
										 (comp-state navgraph LOCALIZED))
	:effect ()
)

(:action gripper-calibrated
	:parameters (?r - robot)
	:precondition ()
	:effect ()
)

(:action move-base-is-locked
	:parameters (?r - robot)
	:precondition ()
	:effect ()
)

; ----------------------------- sensing actions ---------------------------

  (:action drop
    :parameters (?r - robot ?wp - workpiece)
    :precondition (and (exog-possible) (holding ?r ?wp))
    :effect (and (not (holding ?r ?wp))
								 (can-hold ?r)
            (increase (total-cost) 1))
  )
	(:action back_up_c_bs
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-BS DOWN-TO-IDLE) (comp-state C-BS DOWN-TO-READY-AT-OUTPUT) ))
 	:effect (and 
 (when (comp-state C-BS DOWN-TO-IDLE)
  (and (not (comp-state C-BS DOWN-TO-IDLE)) (comp-state C-BS IDLE) )
 ) 
 (when (comp-state C-BS DOWN-TO-READY-AT-OUTPUT)
  (and (not (comp-state C-BS DOWN-TO-READY-AT-OUTPUT)) (comp-state C-BS READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action back_up_c_cs1
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-CS1 DOWN-TO-IDLE) (comp-state C-CS1 DOWN-TO-READY-AT-OUTPUT) ))
 	:effect (and 
 (when (comp-state C-CS1 DOWN-TO-IDLE)
  (and (not (comp-state C-CS1 DOWN-TO-IDLE)) (comp-state C-CS1 IDLE) )
 ) 
 (when (comp-state C-CS1 DOWN-TO-READY-AT-OUTPUT)
  (and (not (comp-state C-CS1 DOWN-TO-READY-AT-OUTPUT)) (comp-state C-CS1 READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action back_up_c_cs2
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-CS2 DOWN-TO-IDLE) (comp-state C-CS2 DOWN-TO-READY-AT-OUTPUT) ))
 	:effect (and 
 (when (comp-state C-CS2 DOWN-TO-IDLE)
  (and (not (comp-state C-CS2 DOWN-TO-IDLE)) (comp-state C-CS2 IDLE) )
 ) 
 (when (comp-state C-CS2 DOWN-TO-READY-AT-OUTPUT)
  (and (not (comp-state C-CS2 DOWN-TO-READY-AT-OUTPUT)) (comp-state C-CS2 READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action back_up_c_ds
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-DS DOWN-TO-IDLE) ))
 	:effect (and 
 (when (comp-state C-DS DOWN-TO-IDLE)
  (and (not (comp-state C-DS DOWN-TO-IDLE)) (comp-state C-DS IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action back_up_c_rs1
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS1 DOWN-TO-IDLE) (comp-state C-RS1 DOWN-TO-READY-AT-OUTPUT) ))
 	:effect (and 
 (when (comp-state C-RS1 DOWN-TO-IDLE)
  (and (not (comp-state C-RS1 DOWN-TO-IDLE)) (comp-state C-RS1 IDLE) )
 ) 
 (when (comp-state C-RS1 DOWN-TO-READY-AT-OUTPUT)
  (and (not (comp-state C-RS1 DOWN-TO-READY-AT-OUTPUT)) (comp-state C-RS1 READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action back_up_c_rs2
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS2 DOWN-TO-IDLE) (comp-state C-RS2 DOWN-TO-READY-AT-OUTPUT) ))
 	:effect (and 
 (when (comp-state C-RS2 DOWN-TO-IDLE)
  (and (not (comp-state C-RS2 DOWN-TO-IDLE)) (comp-state C-RS2 IDLE) )
 ) 
 (when (comp-state C-RS2 DOWN-TO-READY-AT-OUTPUT)
  (and (not (comp-state C-RS2 DOWN-TO-READY-AT-OUTPUT)) (comp-state C-RS2 READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action break_c_bs
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-BS PREPARED)))
 	:effect (and 
 (when (comp-state C-BS PREPARED)
  (and (not (comp-state C-BS PREPARED)) (comp-state C-BS BROKEN) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action break_c_cs1
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-CS1 PREPARED) (mps-side-free C-CS1 INPUT) ))
 	:effect (and 
 (when (comp-state C-CS1 PREPARED)
  (and (not (comp-state C-CS1 PREPARED)) (comp-state C-CS1 BROKEN) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action break_c_cs2
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-CS2 PREPARED) (mps-side-free C-CS2 INPUT) ))
 	:effect (and 
 (when (comp-state C-CS2 PREPARED)
  (and (not (comp-state C-CS2 PREPARED)) (comp-state C-CS2 BROKEN) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action break_c_ds
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-DS PREPARED) ))
 	:effect (and 
 (when (comp-state C-DS PREPARED)
  (and (not (comp-state C-DS PREPARED)) (comp-state C-DS BROKEN) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action break_c_rs1
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS1 PREPARED) (mps-side-free C-RS1 INPUT) ))
 	:effect (and 
 (when (comp-state C-RS1 PREPARED)
  (and (not (comp-state C-RS1 PREPARED)) (comp-state C-RS1 BROKEN) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action break_c_rs2
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS2 PREPARED) (mps-side-free C-RS2 INPUT) ))
 	:effect (and 
 (when (comp-state C-RS2 PREPARED)
  (and (not (comp-state C-RS2 PREPARED)) (comp-state C-RS2 BROKEN) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action break_gripper_axis
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state gripper CALIBRATED) (comp-state gripper UNCALIBRATED) ))
 	:effect (and 
 (when (comp-state gripper CALIBRATED)
  (and (not (comp-state gripper CALIBRATED)) (comp-state gripper AXIS-BROKEN) )
 ) 
 (when (comp-state gripper UNCALIBRATED)
  (and (not (comp-state gripper UNCALIBRATED)) (comp-state gripper AXIS-BROKEN) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action break_gripper_fingers
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state gripper CALIBRATED) (comp-state gripper UNCALIBRATED) ))
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
(:action c_bs_finished_processing
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-BS PREPARED) ))
 	:effect (and 
 (when (comp-state C-BS PREPARED)
  (and (not (comp-state C-BS PREPARED)) (comp-state C-BS READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action c_ds_finished_processing
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-DS PREPARED) ))
 	:effect (and 
 (when (comp-state C-DS PREPARED)
  (and (not (comp-state C-DS PREPARED)) (comp-state C-DS IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action connection_lost
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state communication CONNECTION-ESTABLISHED) ))
 	:effect (and 
 (when (comp-state communication CONNECTION-ESTABLISHED)
  (and (not (comp-state communication CONNECTION-ESTABLISHED)) (comp-state communication CONNECTION-LOST) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action finished-processing-c-rs1
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS1 PREPARED) ))
 	:effect (and 
 (when (comp-state C-RS1 PREPARED)
  (and (not (comp-state C-RS1 PREPARED)) (comp-state C-RS1 READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action finished-processing-c-rs2
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS2 PREPARED) ))
 	:effect (and 
 (when (comp-state C-RS2 PREPARED)
  (and (not (comp-state C-RS2 PREPARED)) (comp-state C-RS2 READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action laser_lost
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state laser ACTIVATED) ))
 	:effect (and 
 (when (comp-state laser ACTIVATED)
  (and (not (comp-state laser ACTIVATED)) (comp-state laser LOST) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action lock-move-base
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state move-base INIT) ))
 	:effect (and 
 (when (comp-state move-base INIT)
  (and (not (comp-state move-base INIT)) (comp-state move-base LOCKED) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action realsense_lost
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state realsense ACTIVATED) (comp-state realsense DEACTIVATED) ))
 	:effect (and 
 (when (comp-state realsense ACTIVATED)
  (and (not (comp-state realsense ACTIVATED)) (comp-state realsense LOST) )
 ) 
 (when (comp-state realsense DEACTIVATED)
  (and (not (comp-state realsense DEACTIVATED)) (comp-state realsense LOST) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action refbox_c_bs_down
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-BS IDLE) ))
 	:effect (and 
 (when (comp-state C-BS IDLE)
  (and (not (comp-state C-BS IDLE)) (comp-state C-BS DOWN-TO-IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action refbox_c_cs1_down
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-CS1 IDLE) ))
 	:effect (and 
 (when (comp-state C-CS1 IDLE)
  (and (not (comp-state C-CS1 IDLE)) (comp-state C-CS1 DOWN-TO-IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action refbox_c_cs2_down
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-CS2 IDLE) (comp-state C-CS2 READY-AT-OUTPUT) ))
 	:effect (and 
 (when (comp-state C-CS2 IDLE)
  (and (not (comp-state C-CS2 IDLE)) (comp-state C-CS2 DOWN-TO-IDLE) )
 ) 
 (when (comp-state C-CS2 READY-AT-OUTPUT)
  (and (not (comp-state C-CS2 READY-AT-OUTPUT)) (comp-state C-CS2 DOWN-TO-READY-AT-OUTPUT) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action refbox_c_ds_down
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-DS IDLE) ))
 	:effect (and 
 (when (comp-state C-DS IDLE)
  (and (not (comp-state C-DS IDLE)) (comp-state C-DS DOWN-TO-IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action refbox_c_rs1_down
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS1 IDLE) ))
 	:effect (and 
 (when (comp-state C-RS1 IDLE)
  (and (not (comp-state C-RS1 IDLE)) (comp-state C-RS1 DOWN-TO-IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action refbox_c_rs2_down
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS2 IDLE) ))
 	:effect (and 
 (when (comp-state C-RS2 IDLE)
  (and (not (comp-state C-RS2 IDLE)) (comp-state C-RS2 DOWN-TO-IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action tag_camera_lost
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state tag-camera ACTIVATED) (comp-state tag-camera DEACTIVATED) ))
 	:effect (and 
 (when (comp-state tag-camera ACTIVATED)
  (and (not (comp-state tag-camera ACTIVATED)) (comp-state tag-camera LOST) )
 ) 
 (when (comp-state tag-camera DEACTIVATED)
  (and (not (comp-state tag-camera DEACTIVATED)) (comp-state tag-camera LOST) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action unbreak_c_bs
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-BS BROKEN) ))
 	:effect (and 
 (when (comp-state C-BS BROKEN)
  (and (not (comp-state C-BS BROKEN)) (comp-state C-BS IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action unbreak_c_cs1
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-CS1 BROKEN) ))
 	:effect (and 
 (when (comp-state C-CS1 BROKEN)
  (and (not (comp-state C-CS1 BROKEN)) (comp-state C-CS1 IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action unbreak_c_cs2
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-CS2 BROKEN) ))
 	:effect (and 
 (when (comp-state C-CS2 BROKEN)
  (and (not (comp-state C-CS2 BROKEN)) (comp-state C-CS2 IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action unbreak_c_ds
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-DS BROKEN) ))
 	:effect (and 
 (when (comp-state C-DS BROKEN)
  (and (not (comp-state C-DS BROKEN)) (comp-state C-DS IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action unbreak_c_rs1
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS1 BROKEN) ))
 	:effect (and 
 (when (comp-state C-RS1 BROKEN)
  (and (not (comp-state C-RS1 BROKEN)) (comp-state C-RS1 IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action unbreak_c_rs2
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state C-RS2 BROKEN) ))
 	:effect (and 
 (when (comp-state C-RS2 BROKEN)
  (and (not (comp-state C-RS2 BROKEN)) (comp-state C-RS2 IDLE) )
 ) 
 		(increase (total-cost) 1)
 )
 )
(:action uncalibrate
 	:parameters ()
 	:precondition (and (exog-possible) (or (comp-state gripper CALIBRATED) ))
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

)
