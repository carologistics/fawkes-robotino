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
	)

	(:constants
		START - location
		BS CS DS RS SS - mps-typename
		IDLE BROKEN PREPARED PROCESSING PROCESSED WAIT-IDLE READY-AT-OUTPUT DOWN - mps-statename
		INPUT OUTPUT - mps-side
		BASE_NONE BASE_RED BASE_BLACK BASE_SILVER - base-color
		CAP_NONE CAP_BLACK CAP_GREY - cap-color
		GATE-1 GATE-2 GATE-3 - ds-gate
		RING_NONE RING_BLUE RING_GREEN RING_ORANGE RING_YELLOW - ring-color
		RETRIEVE_CAP MOUNT_CAP - cs-operation
		C0 C1 C2 C3 - order-complexity-value
		LEFT MIDDLE RIGHT - shelf-spot
		NA ZERO ONE TWO THREE - ring-num
	)

	(:predicates
		(at ?r - robot ?m - location ?side - mps-side)
		(holding ?r - robot ?wp - workpiece)
		(can-hold ?r - robot)
		(entered-field ?r - robot)
		(location-free ?l - location ?side - mps-side)
		(robot-waiting ?r - robot)
		(mps-type ?m - mps ?t - mps-typename)
		(mps-state ?m - mps ?s - mps-statename)
		(bs-prepared-color ?m - mps ?col - base-color)
		(bs-prepared-side ?m - mps ?side - mps-side)
		(cs-can-perform ?m - mps ?op - cs-operation)
		(cs-prepared-for ?m - mps ?op - cs-operation)
		(cs-buffered ?m - mps ?col - cap-color)
		(cs-free ?m - mps)
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
    (spot-free ?m - mps ?spot - shelf-spot)
	)

	(:action prepare-bs
		:parameters (?m - mps ?side - mps-side ?bc - base-color)
		:precondition (and (mps-type ?m BS) (mps-state ?m IDLE))
		:effect (and (not (mps-state ?m IDLE)) (mps-state ?m READY-AT-OUTPUT)
								 (bs-prepared-color ?m ?bc) (bs-prepared-side ?m ?side))
	)

	(:action prepare-ds
		:parameters (?m - mps ?gate - ds-gate)
		:precondition (and (mps-type ?m DS) (mps-state ?m IDLE))
		:effect (and (not (mps-state ?m IDLE)) (mps-state ?m PREPARED)
                 (ds-prepared-gate ?m ?gate))
	)

	(:action prepare-cs
		:parameters (?m - mps ?op - cs-operation)
		:precondition (and (mps-type ?m CS) (mps-state ?m IDLE) (cs-can-perform ?m ?op))
		:effect (and (not (mps-state ?m IDLE)) (mps-state ?m PREPARED)
								 (not (cs-can-perform ?m ?op)) (cs-prepared-for ?m ?op))
	)

	(:action bs-dispense
		:parameters (?m - mps ?side - mps-side ?wp - workpiece ?basecol - base-color)
		:precondition (and (mps-type ?m BS) (mps-state ?m READY-AT-OUTPUT)
											 (bs-prepared-color ?m ?basecol) (bs-prepared-side ?m ?side)
											 (wp-base-color ?wp BASE_NONE) (wp-unused ?wp))
		:effect (and (wp-at ?wp ?m ?side)
								 (not (wp-base-color ?wp BASE_NONE)) (wp-base-color ?wp ?basecol)
								 (not (wp-unused ?wp)) (wp-usable ?wp))
	)
		
	(:action cs-mount-cap
		:parameters (?m - mps ?wp - workpiece ?capcol - cap-color)
		:precondition (and (mps-type ?m CS) (mps-state ?m PROCESSING)
										(cs-buffered ?m ?capcol) (cs-prepared-for ?m MOUNT_CAP)
										(wp-usable ?wp) (wp-at ?wp ?m INPUT)
										(wp-cap-color ?wp CAP_NONE))
		:effect (and (not (mps-state ?m PROCESSING)) (mps-state ?m READY-AT-OUTPUT)
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
								 (not (wp-cap-color ?wp CAP_NONE)) (wp-cap-color ?wp ?capcol)
								 (cs-can-perform ?m RETRIEVE_CAP))
	)

	(:action cs-retrieve-cap
		:parameters (?m - mps ?cc - cap-carrier ?capcol - cap-color)
		:precondition (and (mps-type ?m CS)
										(cs-prepared-for ?m RETRIEVE_CAP)
										(wp-at ?cc ?m INPUT)  (wp-cap-color ?cc ?capcol))
		:effect (and (not (mps-state ?m PROCESSING)) (mps-state ?m READY-AT-OUTPUT)
								 (not (wp-at ?cc ?m INPUT)) (wp-at ?cc ?m OUTPUT)
								 (not (wp-cap-color ?cc ?capcol)) (wp-cap-color ?cc CAP_NONE)
								 (cs-buffered ?m ?capcol) (cs-can-perform ?m MOUNT_CAP))
	)
	
	(:action prepare-rs
		:parameters (?m - mps ?rc - ring-color ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) (mps-state ?m IDLE) (rs-ring-spec ?m ?rc ?r-req)
						(rs-filled-with ?m ?rs-before) (rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and (not (mps-state ?m IDLE)) (mps-state ?m PREPARED)
								 (rs-prepared-color ?m ?rc))
	)

	(:action rs-mount-ring1
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) (mps-state ?m PROCESSING)
										(wp-at ?wp ?m INPUT) (wp-usable ?wp)
										(wp-ring1-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and (not (mps-state ?m PROCESSING)) (mps-state ?m READY-AT-OUTPUT)
								 (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
								 (not (wp-ring1-color ?wp RING_NONE)) (wp-ring1-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after))
	)

	(:action rs-mount-ring2
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color
					?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) (mps-state ?m PROCESSING)
										(wp-at ?wp ?m INPUT) (wp-usable ?wp)
										(wp-ring1-color ?wp ?col1)
										(wp-ring2-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-afrer))
		:effect (and (not (mps-state ?m PROCESSING)) (mps-state ?m READY-AT-OUTPUT)
								 (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (wp-at ?wp ?m OUTPUT)
								 (not (wp-ring2-color ?wp RING_NONE)) (wp-ring2-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after))
	)

	(:action rs-mount-ring3
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color ?col2 - ring-color
							?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) (mps-state ?m PROCESSING)
										(wp-at ?wp ?m INPUT) (wp-usable ?wp)
										(wp-ring1-color ?wp ?col1)
										(wp-ring2-color ?wp ?col2)
										(wp-ring3-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-afrer))
		:effect (and (not (mps-state ?m PROCESSING)) (mps-state ?m READY-AT-OUTPUT)
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
	; (:action move
	; 	:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
	; 	:precondition (and (entered-field ?r)
	; 									(at ?r ?from ?from-side)
	; 									(location-free ?to ?to-side))
	; 	:effect (and (not (at ?r ?from ?from-side))
	; 							 (location-free ?from ?from-side)
	; 							 (not (location-free ?to ?to-side))
	; 							 (at ?r ?to ?to-side))
	; )

	; Move actions specific for the expected follow-up action.
	; This models the move in two versions specific to the expected next action,
	; either the retrieval or the delivery of a workpiece. While a more generic
	; such as the one would be desirable, in typical test cases these specific
	; actions cut the planning time by about 95%.
	(:action move-wp-put
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps)
		:precondition (and (entered-field ?r)
										(at ?r ?from ?from-side)
										(location-free ?to INPUT)
										(mps-state ?to IDLE))
		:effect (and (not (at ?r ?from ?from-side))
								 (location-free ?from ?from-side)
								 (not (location-free ?to INPUT))
								 (at ?r ?to INPUT))
	)

	(:action move-wp-get
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
		:precondition (and (entered-field ?r)
										(at ?r ?from ?from-side)
										(location-free ?to ?to-side)
										(mps-state ?to READY-AT-OUTPUT)
										(can-hold ?r))
		:effect (and (not (at ?r ?from ?from-side))
								 (location-free ?from ?from-side)
								 (not (location-free ?to ?to-side))
								 (at ?r ?to ?to-side))
	)

	(:action enter-field
		:parameters (?r - robot ?team-color - team-color)
		:precondition (and (location-free START INPUT)
										(robot-waiting ?r))
		:effect (and (entered-field ?r)
								 (at ?r START INPUT)
								 (not (location-free START INPUT))
								 (not (robot-waiting ?r)) (can-hold ?r))
	)

	(:action wp-discard
		:parameters (?r - robot ?cc - cap-carrier)
		:precondition (and (holding ?r ?cc))
		:effect (and (not (holding ?r ?cc)) (wp-unused ?cc) (not (wp-usable ?cc)) 
                 (can-hold ?r))
	)

	(:action wp-get-shelf
		:parameters (?r - robot ?cc - cap-carrier ?m - mps ?spot - shelf-spot)
		:precondition (and (at ?r ?m INPUT) (wp-on-shelf ?cc ?m ?spot) (can-hold ?r))
		:effect (and (holding ?r ?cc) (not (can-hold ?r))
								 (not (wp-on-shelf ?cc ?m ?spot)) (wp-usable ?cc)
                 (spot-free ?m ?spot))
	)

  (:action refill-shelf1
    :parameters (?m - mps ?spot - shelf-spot ?cc1 - cap-carrier
                 ?color - cap-color)
    :precondition ( and (spot-free ?m ?spot)
                        (wp-unused ?cc1)
                  )
    :effect (and ( not (spot-free ?m ?spot) )
                 (wp-on-shelf ?cc1 ?m ?spot)
                 (not (wp-unused ?cc1))
                 (wp-cap-color ?cc1  ?color)
            )
  )
;  (:action refill-shelf3
;    :parameters (?m - mps ?cc1 - cap-carrier
;                 ?cc2 - cap-carrier ?cc3 - cap-carrier
;                 ?color - cap-color)
;    :precondition ( and (spot-free ?m LEFT) (spot-free ?m MIDDLE)
;                        (spot-free ?m RIGHT)
;                        (wp-unused ?cc1) (wp-unused ?cc2) (wp-unused ?cc3)
;;                        (wp-cap-color ?cc1  CAP_NONE)
;;                        (wp-cap-color ?cc2  CAP_NONE)
;;                        (wp-cap-color ?cc3  CAP_NONE)
;                        (not (= ?cc1 ?cc2)) (not (= ?cc1 ?cc3))
;                        (not (= ?cc2 ?cc3))
;                  )
;    :effect (and ( not (spot-free ?m LEFT) ) ( not (spot-free ?m MIDDLE) )
;                 ( not (spot-free ?m RIGHT) )
;                 (wp-on-shelf ?cc1 ?m LEFT) (wp-on-shelf ?cc2 ?m MIDDLE)
;                 (wp-on-shelf ?cc3 ?m RIGHT)
;                 (not (wp-unused ?cc1))
;                 (not (wp-unused ?cc2))
;                 (not (wp-unused ?cc3))
;                 (wp-cap-color ?cc1  ?color)
;                 (wp-cap-color ?cc2  ?color)
;                 (wp-cap-color ?cc3  ?color)
;            )
;  )

	(:action wp-get
		:parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side)
		:precondition (and (at ?r ?m ?side) (can-hold ?r) (wp-at ?wp ?m ?side)
										(mps-state ?m READY-AT-OUTPUT) (wp-usable ?wp))
		:effect (and (not (wp-at ?wp ?m ?side)) (holding ?r ?wp) (not (can-hold ?r))
								 (not (mps-state ?m READY-AT-OUTPUT)) (mps-state ?m IDLE))
	)

	(:action wp-put
		:parameters (?r - robot ?wp - workpiece ?m - mps)
		:precondition (and (at ?r ?m INPUT) (mps-state ?m PREPARED)
										(wp-usable ?wp) (holding ?r ?wp))
		:effect (and (wp-at ?wp ?m INPUT) (not (holding ?r ?wp)) (can-hold ?r)
								 (not (mps-state ?m PREPARED)) (mps-state ?m PROCESSING))
	)

	(:action wp-put-slide-cc
		:parameters (?r - robot ?wp - cap-carrier ?m - mps ?rs-before - ring-num ?rs-after - ring-num)
		:precondition (and (mps-type ?m RS)
							(at ?r ?m INPUT)
							(wp-usable ?wp)
							(holding ?r ?wp)
							(rs-filled-with ?m ?rs-before)
							(rs-inc ?rs-before ?rs-after))
		:effect (and (not (wp-usable ?wp))
					(not (holding ?r ?wp))
					(can-hold ?r)
					(not (rs-filled-with ?m ?rs-before))
					(rs-filled-with ?m ?rs-after))
	)

	(:action fulfill-order-c0
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color)
		:precondition (and (wp-at ?wp ?m INPUT)
                      (wp-usable ?wp)
                       (mps-type ?m DS)
											 (ds-prepared-gate ?m ?g)
                       (order-gate ?ord ?g)
											 (order-complexity ?ord C0)
											 (order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
											 (order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol)
											 (wp-ring1-color ?wp RING_NONE) (wp-ring2-color ?wp RING_NONE) (wp-ring3-color ?wp RING_NONE))
		:effect (and (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT))
                 (not (ds-prepared-gate ?m ?g))
								 (not (wp-base-color ?wp ?basecol)) (not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c1
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) (wp-usable ?wp)
											 (mps-type ?m DS)
											 (ds-prepared-gate ?m ?g)
                       (order-gate ?ord ?g)
											 (order-complexity ?ord C1)
											 (order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
											 (order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
											 (order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol))
		:effect (and (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT))
                 (not (ds-prepared-gate ?m ?g))
								 (not (wp-base-color ?wp ?basecol)) (not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c2
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color ?ring2col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) (wp-usable ?wp)
											 (mps-type ?m DS)
											 (ds-prepared-gate ?m ?g)
                       (order-gate ?ord ?g)
											 (order-complexity ?ord C2)
											 (order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
											 (order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
											 (order-ring2-color ?ord ?ring2col) (wp-ring2-color ?wp ?ring2col)
											 (wp-ring3-color ?wp RING_NONE)
											 (order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol))
		:effect (and (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT))
                 (not (ds-prepared-gate ?m ?g))
								 (not (wp-base-color ?wp ?basecol)) (not (wp-cap-color ?wp ?capcol)))
								 
	)

	(:action fulfill-order-c3
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color ?ring2col - ring-color ?ring3col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) (wp-usable ?wp)
											 (mps-type ?m DS)
											 (ds-prepared-gate ?m ?g)
                       (order-gate ?ord ?g)
											 (order-complexity ?ord C3)
											 (order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
											 (order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
											 (order-ring2-color ?ord ?ring2col) (wp-ring2-color ?wp ?ring2col)
											 (order-ring3-color ?ord ?ring3col) (wp-ring3-color ?wp ?ring3col)
											 (order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol)
											 )
		:effect (and (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT))
                 (not (ds-prepared-gate ?m ?g))
								 (not (wp-base-color ?wp ?basecol)) (not (wp-cap-color ?wp ?capcol)))
	)
)
