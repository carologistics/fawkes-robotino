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
    waitpoint - location
		mps - location
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
		ring-num - object
		zone - object
		token - object
		master-token - token
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
		RETRIEVE - ss-operation
		C0 C1 C2 C3 - order-complexity-value
		LEFT MIDDLE RIGHT - shelf-spot
		NA ZERO ONE TWO THREE - ring-num
	)

	(:predicates
		(self ?r - robot)
		(at ?r - robot ?m - location ?side - mps-side)
		(holding ?r - robot ?wp - workpiece)
		(can-hold ?r - robot)
		(entered-field ?r - robot)
		(robot-waiting ?r - robot)
		(mps-type ?m - mps ?t - mps-typename)
		(mps-state ?m - mps ?s - mps-statename)
		(mps-team ?m - mps ?col - team-color)
		(mps-side-free ?m - mps ?side - mps-side)
		(bs-prepared-color ?m - mps ?col - base-color)
		(bs-prepared-side ?m - mps ?side - mps-side)
		(cs-can-perform ?m - mps ?op - cs-operation)
		(cs-prepared-for ?m - mps ?op - cs-operation)
		(cs-buffered ?m - mps ?col - cap-color)
		(cs-color ?m - mps ?col - cap-color)
		(cs-free ?m - mps)
		(ss-prepared-for ?m - mps ?op - ss-operation ?wp - workpiece)
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
    (ss-initialized ?m - mps)
    (ss-stored-wp ?m  - mps ?wp - workpiece)
    (locked ?name - object)
    (location-locked ?m - mps ?s - mps-side)
	)

;Kind of a hack. actually it should model the removal of present workpieces
	(:action reset-mps
		:parameters (?m - mps)
		:precondition (or (mps-state ?m BROKEN) (not (mps-state ?m BROKEN)))
		:effect (mps-state ?m BROKEN)
	)

	(:action prepare-bs
		:parameters (?m - mps ?side - mps-side ?bc - base-color)
		:precondition (and (mps-type ?m BS) (mps-state ?m IDLE))
		:effect (and (not (mps-state ?m IDLE)) (mps-state ?m READY-AT-OUTPUT)
								 (bs-prepared-color ?m ?bc) (bs-prepared-side ?m ?side))
	)

	(:action prepare-ds
		:parameters (?m - mps ?ord - order)
		:precondition (and (mps-type ?m DS) (mps-state ?m IDLE))
		:effect (and (not (mps-state ?m IDLE)) (mps-state ?m PREPARED)
                 (ds-prepared-order ?m ?ord))
	)

	(:action prepare-cs
		:parameters (?m - mps ?op - cs-operation)
		:precondition (and  (mps-type ?m CS) (mps-state ?m IDLE)
                        (cs-can-perform ?m ?op))
		:effect (and (not (mps-state ?m IDLE)) (mps-state ?m READY-AT-OUTPUT)
								 (not (cs-can-perform ?m ?op)) (cs-prepared-for ?m ?op))
	)

	(:action bs-dispense
		:parameters (?r - robot ?m - mps ?side - mps-side ?wp - workpiece ?basecol - base-color)
		:precondition (and (mps-type ?m BS) (or (mps-state ?m PROCESSING) (mps-state ?m READY-AT-OUTPUT))
                       (bs-prepared-color ?m ?basecol)
                       (bs-prepared-side ?m ?side)
											 (wp-base-color ?wp BASE_NONE) (wp-unused ?wp)
											 (self ?r)
											 (mps-side-free ?m ?side))
		:effect (and (wp-at ?wp ?m ?side) (not (mps-side-free ?m ?side))
								 (not (wp-base-color ?wp BASE_NONE)) (wp-base-color ?wp ?basecol)
								 (not (wp-unused ?wp)) (wp-usable ?wp)
								 (not (wp-spawned-for ?wp ?r)))
	)

	(:action cs-mount-cap
		:parameters (?m - mps ?wp - workpiece ?capcol - cap-color)
		:precondition (and (mps-type ?m CS) (or (mps-state ?m PROCESSING) (mps-state ?m READY-AT-OUTPUT))
										(cs-buffered ?m ?capcol) (cs-prepared-for ?m MOUNT_CAP)
										(wp-usable ?wp) (wp-at ?wp ?m INPUT)
										(not (mps-side-free ?m INPUT))
										(mps-side-free ?m OUTPUT)
										(wp-cap-color ?wp CAP_NONE))
		:effect (and
								 (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT)
								 (wp-at ?wp ?m OUTPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-cap-color ?wp CAP_NONE)) (wp-cap-color ?wp ?capcol)
								 (cs-can-perform ?m RETRIEVE_CAP)
								 (not (cs-can-perform ?m MOUNT_CAP))
								 (not (cs-prepared-for ?m MOUNT_CAP))
								 (not (cs-buffered ?m ?capcol)))
	)

	(:action request-cs-mount-cap
		:parameters (?r - robot ?m - mps ?wp - workpiece ?capcol - cap-color)
		:precondition (self ?r)
		:effect (self ?r)
	)

	(:action cs-retrieve-cap
		:parameters (?m - mps ?cc - cap-carrier ?capcol - cap-color)
		:precondition (and (mps-type ?m CS) (or (mps-state ?m PROCESSING) (mps-state ?m READY-AT-OUTPUT))
                       (cs-prepared-for ?m RETRIEVE_CAP)
										(wp-at ?cc ?m INPUT) (not (mps-side-free ?m INPUT)) (mps-side-free ?m OUTPUT)
										(wp-cap-color ?cc ?capcol))
		:effect (and
								 (not (wp-at ?cc ?m INPUT)) (mps-side-free ?m INPUT)
								 (wp-at ?cc ?m OUTPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-cap-color ?cc ?capcol)) (wp-cap-color ?cc CAP_NONE)
								 (cs-buffered ?m ?capcol)(cs-can-perform ?m MOUNT_CAP)
								 (not (cs-prepared-for ?m RETRIEVE_CAP)))
	)

	(:action request-cs-retrieve-cap
		:parameters (?r - robot ?m - mps ?cc - cap-carrier ?capcol - cap-color)
		:precondition (self ?r)
		:effect (self ?r)
	)

	(:action prepare-rs
		:parameters (?m - mps ?rc - ring-color ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and  (mps-type ?m RS) (mps-state ?m IDLE)
                        (rs-ring-spec ?m ?rc ?r-req)
						            (rs-filled-with ?m ?rs-before)
                        (rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and (not (mps-state ?m IDLE)) (mps-state ?m READY-AT-OUTPUT)
								 (rs-prepared-color ?m ?rc))
	)

	(:action rs-mount-ring1
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) (or (mps-state ?m PROCESSING) (mps-state ?m READY-AT-OUTPUT))
										(wp-at ?wp ?m INPUT) (not (mps-side-free ?m INPUT)) 
                    (mps-side-free ?m OUTPUT)
										(wp-usable ?wp)
										(wp-ring1-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and
								 (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT) (wp-at ?wp ?m OUTPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-ring1-color ?wp RING_NONE)) (wp-ring1-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after))
	)

	(:action rs-mount-ring2
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color
					?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) (or (mps-state ?m PROCESSING) (mps-state ?m READY-AT-OUTPUT))
										(wp-at ?wp ?m INPUT) (not (mps-side-free ?m INPUT)) (wp-usable ?wp)
										(mps-side-free ?m OUTPUT)
										(wp-ring1-color ?wp ?col1)
										(wp-ring2-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and
								 (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT) (wp-at ?wp ?m OUTPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-ring2-color ?wp RING_NONE)) (wp-ring2-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after))
	)

	(:action rs-mount-ring3
		:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color ?col2 - ring-color
							?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
		:precondition (and (mps-type ?m RS) (or (mps-state ?m PROCESSING) (mps-state ?m READY-AT-OUTPUT))
										(wp-at ?wp ?m INPUT) (not (mps-side-free ?m INPUT)) (wp-usable ?wp)
										(mps-side-free ?m OUTPUT)
										(wp-ring1-color ?wp ?col1)
										(wp-ring2-color ?wp ?col2)
										(wp-ring3-color ?wp RING_NONE)
										(wp-cap-color ?wp CAP_NONE)
										(rs-prepared-color ?m ?col)
										(rs-ring-spec ?m ?col ?r-req)
										(rs-filled-with ?m ?rs-before)
										(rs-sub ?rs-before ?r-req ?rs-after))
		:effect (and
								 (not (rs-prepared-color ?m ?col))
								 (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT) (wp-at ?wp ?m OUTPUT) (not (mps-side-free ?m OUTPUT))
								 (not (wp-ring3-color ?wp RING_NONE)) (wp-ring3-color ?wp ?col)
								 (not (rs-filled-with ?m ?rs-before)) (rs-filled-with ?m ?rs-after))
	)

	(:action request-rs-mount-ring
		:parameters (?r - robot ?m - mps ?wp - workpiece ?col - ring-color
			 				?ring-pos - ring-num ?col1 - ring-color ?col2 - ring-color ?col3 - ring-color
							?r-req - ring-num)
		:precondition (self ?r)
		:effect (self ?r)	
	)

	; The following is the generic move version.
	; It takes the robot from any location (at any side) to any MPS (any side).
	; However, this also creates a tremendous number of options during search and
	; hence is detrimental for planning performance.
	;
	(:action go-wait
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - waitpoint)
		:precondition (at ?r ?from ?from-side)
		:effect (and
					(not (at ?r ?from ?from-side))
					(at ?r ?to WAIT))
	)

  (:action wait
    :parameters (?r - robot ?point - waitpoint)
    :precondition (at ?r ?point WAIT)
    :effect (at ?r ?point WAIT)
  )

	(:action move
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
		:precondition (at ?r ?from ?from-side)
		:effect (and (not (at ?r ?from ?from-side))
								 (at ?r ?to ?to-side))
	)

	; Move actions specific for the expected follow-up action.
	; This models the move in two versions specific to the expected next action,
	; either the retrieval or the delivery of a workpiece. While a more generic
	; such as the one would be desirable, in typical test cases these specific
	; actions cut the planning time by about 95%.
	(:action move-wp-put
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps)
		:precondition (and (at ?r ?from ?from-side)
										(mps-state ?to IDLE))
		:effect (and (not (at ?r ?from ?from-side))
								 (at ?r ?to INPUT))
	)

	(:action move-wp-get
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
		:precondition (and (at ?r ?from ?from-side)
										(mps-state ?to READY-AT-OUTPUT)
										(can-hold ?r))
		:effect (and (not (at ?r ?from ?from-side))
								 (at ?r ?to ?to-side))
	)

	(:action enter-field
		:parameters (?r - robot ?team-color - team-color)
		:precondition (robot-waiting ?r)
		:effect (and (entered-field ?r)
								 (at ?r START INPUT)
								 (not (robot-waiting ?r)))
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

  (:action refill-shelf
    :parameters (?m - mps ?spot - shelf-spot ?cc - cap-carrier ?color - cap-color)
    :precondition (and (spot-free ?m ?spot))
    :effect (and (not (spot-free ?m ?spot))
                 (wp-on-shelf ?cc ?m ?spot)
                 (not (wp-unused ?cc))
                 (wp-cap-color ?cc ?color)
            )
  )

	(:action wp-get
		:parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side)
		:precondition (and (at ?r ?m ?side) (can-hold ?r) (wp-at ?wp ?m ?side)
                    (wp-usable ?wp) (not (mps-side-free ?m ?side)))
		:effect (and (not (wp-at ?wp ?m ?side)) (holding ?r ?wp) (not (can-hold ?r))
								 (not (mps-state ?m READY-AT-OUTPUT)) (mps-state ?m IDLE)
								 (mps-side-free ?m ?side))
	)

	(:action wp-put
		:parameters (?r - robot ?wp - workpiece ?m - mps)
		:precondition (and (at ?r ?m INPUT)
										(wp-usable ?wp) (holding ?r ?wp)
										(mps-side-free ?m INPUT))
		:effect (and (wp-at ?wp ?m INPUT) (not (holding ?r ?wp)) (can-hold ?r) (not (mps-side-free ?m INPUT)))
	)

	(:action wp-put-slide-cc
		:parameters (?r - robot ?wp - cap-carrier ?m - mps)
		:precondition (and (mps-type ?m RS) (not (mps-state BROKEN))
							(at ?r ?m INPUT)
							(wp-usable ?wp)
							(holding ?r ?wp))
		:effect (and (not (wp-usable ?wp))
					(not (holding ?r ?wp))
					(can-hold ?r))
	)

	(:action fulfill-order-c0
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color)
		:precondition (and (wp-at ?wp ?m INPUT) (not (mps-side-free ?m INPUT))
                      (wp-usable ?wp)
                       (mps-type ?m DS)
											 (ds-prepared-order ?m ?ord)
											 (order-complexity ?ord C0)
											 (order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
											 (order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol)
											 (wp-ring1-color ?wp RING_NONE) (wp-ring2-color ?wp RING_NONE) (wp-ring3-color ?wp RING_NONE))
		:effect (and (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT)
                 (not (ds-prepared-order ?m ?ord))
                 (not (wp-usable ?wp))
								 (not (wp-base-color ?wp ?basecol)) (not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c1
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) (not (mps-side-free ?m INPUT)) (wp-usable ?wp)
											 (mps-type ?m DS)
											 (ds-prepared-order ?m ?ord)
											 (order-complexity ?ord C1)
											 (order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
											 (order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
											 (order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol))
		:effect (and (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT)
                 (not (ds-prepared-order ?m ?ord))
                 (not (wp-usable ?wp))
								 (not (wp-base-color ?wp ?basecol)) (not (wp-cap-color ?wp ?capcol)))
	)

	(:action fulfill-order-c2
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color ?ring2col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) (not (mps-side-free ?m INPUT)) (wp-usable ?wp)
											 (mps-type ?m DS)
											 (ds-prepared-order ?m ?ord)
											 (order-complexity ?ord C2)
											 (order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
											 (order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
											 (order-ring2-color ?ord ?ring2col) (wp-ring2-color ?wp ?ring2col)
											 (wp-ring3-color ?wp RING_NONE)
											 (order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol))
		:effect (and (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT)
                 (not (ds-prepared-order ?m ?ord)) (not (wp-usable ?wp))
								 (not (wp-base-color ?wp ?basecol)) (not (wp-cap-color ?wp ?capcol)))

	)

	(:action fulfill-order-c3
		:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
		             ?basecol - base-color ?capcol - cap-color
		             ?ring1col - ring-color ?ring2col - ring-color ?ring3col - ring-color)

		:precondition (and (wp-at ?wp ?m INPUT) (not (mps-side-free ?m INPUT)) (wp-usable ?wp)
											 (mps-type ?m DS)
											 (ds-prepared-order ?m ?ord)
											 (order-complexity ?ord C3)
											 (order-base-color ?ord ?basecol) (wp-base-color ?wp ?basecol)
											 (order-ring1-color ?ord ?ring1col) (wp-ring1-color ?wp ?ring1col)
											 (order-ring2-color ?ord ?ring2col) (wp-ring2-color ?wp ?ring2col)
											 (order-ring3-color ?ord ?ring3col) (wp-ring3-color ?wp ?ring3col)
											 (order-cap-color ?ord ?capcol) (wp-cap-color ?wp ?capcol)
											 )
		:effect (and (order-fulfilled ?ord) (not (wp-at ?wp ?m INPUT)) (mps-side-free ?m INPUT)
                 (not (ds-prepared-order ?m ?ord))
                 (not (wp-usable ?wp))
								 (not (wp-base-color ?wp ?basecol)) (not (wp-cap-color ?wp ?capcol)))
	)

(:action request-ds-fulfill-order
		:parameters (?r - robot ?m - mps ?wp - workpiece ?ord - order)
		:precondition (self ?r)
		:effect (self ?r)
	)

  (:action move-node
			:parameters (?r - robot ?z - zone)
			:precondition (self ?r)
			:effect (self ?r)
	)

	(:action explore-zone
			:parameters (?r - robot ?z - zone)
			:precondition (self ?r)
			:effect (self ?r)
	)

  (:action lock
    :parameters (?name - object)
    :precondition (not (locked ?name))
    :effect (locked ?name)
  )
  (:action one-time-lock
    :parameters (?name - object)
    :precondition (not (locked ?name))
    :effect (locked ?name)
  )
  (:action unlock
    :parameters (?name - object)
    :precondition (locked ?name)
    :effect (not (locked ?name))
  )
  (:action eventually-unlock
    :parameters (?name - object)
    :precondition (locked ?name)
    :effect (not (locked ?name))
  )
  (:action location-lock
    :parameters (?location - mps ?side - side)
    :precondition (not (location-locked ?location ?side))
    :effect (location-locked ?location ?side)
  )
  (:action location-unlock
    :parameters (?location - mps ?side - side)
    :precondition (location-locked ?location ?side)
    :effect (not (location-locked ?location ?side))
  )
  (:action expire-locks
    :parameters (?r - robot)
    :precondition (self ?r)
    :effect (self ?r)
  )
  (:action spawn-wp
    :parameters (?wp - workpiece ?r - robot)
    :precondition (and
      (not (wp-unused ?wp))
      (not (wp-usable ?wp))
      (not (wp-spawned-for ?wp ?r)))
    :effect (and
      (wp-spawned-for ?wp ?r)
      (wp-unused ?wp)
      (wp-cap-color ?wp CAP_NONE)
      (wp-ring1-color wp RING_NONE)
      (wp-ring2-color wp RING_NONE)
      (wp-ring3-color wp RING_NONE)
      (wp-base-color wp BASE_NONE)
    )
  )
  (:action ss-store-wp
    :parameters (?r - robot ?m - mps ?wp - workpiece ?base - base-color ?cap - cap-color)
    :precondition (and
      (mps-type ?m SS)
      (wp-unused ?wp)
      (wp-spawned-for ?wp ?r)
      (wp-cap-color ?wp CAP_NONE)
      (wp-ring1-color ?wp RING_NONE)
      (wp-ring2-color ?wp RING_NONE)
      (wp-ring3-color ?wp RING_NONE)
      (wp-base-color ?wp BASE_NONE)
      (not (ss-initialized ?m)))
    :effect (and
      (not (wp-spawned-for ?wp ?r))
      (not (wp-cap-color ?wp CAP_NONE))
      (wp-cap-color ?wp ?cap)
      (not (wp-base-color wp BASE_NONE))
      (wp-base-color wp ?base)
      (ss-initialized ?m)
      (wp-usable ?wp)
      (not (wp-unused ?wp))
      (ss-stored-wp ?m ?wp)
    )
  )
  (:action prepare-ss
    :parameters (?m - mps ?wp - workpiece ?op - ss-operation)
    :precondition (and (mps-type ?m SS) (mps-state ?m IDLE) (ss-stored-wp ?m ?wp))
    :effect (and (not (mps-state ?m IDLE)) (mps-state ?m READY-AT-OUTPUT)
                 (ss-prepared-for ?m ?op ?wp))
  )
  (:action ss-retrieve-c0
   :parameters (?m - mps ?wp - workpiece)
   :precondition (and (mps-type ?m SS) (mps-state ?m READY-AT-OUTPUT)
                      (ss-prepared-for ?m RETRIEVE ?wp)
                      (mps-side-free ?m OUTPUT)
                      (ss-stored-wp ?m ?wp))
   :effect (and (wp-at ?wp ?m OUTPUT) (not (mps-side-free ?m OUTPUT))
                (wp-at ?wp ?m OUTPUT)
                (not (ss-prepared-for ?m RETRIEVE ?wp))
                (not (wp-spawned-for ?wp ?r))
                (not (ss-stored-wp ?m ?wp)))
  )
)
