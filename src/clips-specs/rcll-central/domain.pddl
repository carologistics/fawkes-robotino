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
	zone - object
	token - object
	master-token - token
)

(:constants
	START - location
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
	(mps-type ?m - mps ?t - mps-typename)
	(mps-state ?m - mps ?s - mps-statename)
	(mps-team ?m - mps ?col - team-color)
	(mps-side-free ?m - mps ?side - mps-side)
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
	(spot-free ?m - mps ?spot - shelf-spot)
	(ss-stored-wp ?m  - mps ?wp - workpiece ?shelf - ss-shelf ?slot - ss-slot)
	(ss-shelf-slot-free ?m  - mps ?shelf - ss-shelf ?slot - ss-slot)
	(ss-new-wp-at ?m  - mps ?wp - workpiece ?shelf - ss-shelf
	              ?slot - ss-slot ?base-col - base-color
	              ?ring1-col - ring-color ?ring2-col - ring-color
	              ?ring3-col - ring-color ?cap-col - cap-color)
)

;Kind of a hack. actually it should model the removal of present workpieces
(:action reset-mps
	:parameters (?m - mps)
	:precondition (or (mps-state ?m BROKEN) (not (mps-state ?m BROKEN)))
	:effect (mps-state ?m BROKEN)
)

(:action prepare-bs
	:parameters (?m - mps ?side - mps-side ?bc - base-color)
	:precondition (and (mps-type ?m BS)
	                   (mps-state ?m IDLE)
	              )
	:effect (and (not (mps-state ?m IDLE))
	             (mps-state ?m READY-AT-OUTPUT)
	             (bs-prepared-color ?m ?bc)
	             (bs-prepared-side ?m ?side)
	        )
)

(:action prepare-ds
	:parameters (?m - mps ?ord - order)
	:precondition (and (mps-type ?m DS)
	                   (mps-state ?m IDLE)
	              )
	:effect (and (not (mps-state ?m IDLE))
	             (mps-state ?m PREPARED)
	             (ds-prepared-order ?m ?ord)
	        )
)

(:action prepare-cs
	:parameters (?m - mps ?op - cs-operation)
	:precondition (and  (mps-type ?m CS)
	                    (mps-state ?m IDLE)
	                    (cs-can-perform ?m ?op)
	              )
	:effect (and (not (mps-state ?m IDLE))
	             (mps-state ?m READY-AT-OUTPUT)
	             (not (cs-can-perform ?m ?op))
	             (cs-prepared-for ?m ?op)
	        )
)

(:action bs-dispense
	:parameters (?m - mps ?side - mps-side ?wp - workpiece
	             ?basecol - base-color)
	:precondition (and (mps-type ?m BS)
	                   (or (mps-state ?m PROCESSING)
	                       (mps-state ?m READY-AT-OUTPUT)
	                   )
	                   (bs-prepared-color ?m ?basecol)
	                   (bs-prepared-side ?m ?side)
	                   (wp-base-color ?wp BASE_NONE)
	                   (wp-unused ?wp)
	                   (mps-side-free ?m ?side)
	              )
	:effect (and (wp-at ?wp ?m ?side)
	             (not (mps-side-free ?m ?side))
	             (not (wp-base-color ?wp BASE_NONE))
	             (wp-base-color ?wp ?basecol)
	             (not (wp-unused ?wp)) (wp-usable ?wp)
	        )
)

(:action cs-mount-cap
	:parameters (?m - mps ?wp - workpiece ?capcol - cap-color)
	:precondition (and (mps-type ?m CS)
	                   (or (mps-state ?m PROCESSING)
	                       (mps-state ?m READY-AT-OUTPUT)
	                   )
	                   (cs-buffered ?m ?capcol)
	                   (cs-prepared-for ?m MOUNT_CAP)
	                   (wp-usable ?wp)
	                   (wp-at ?wp ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (mps-side-free ?m OUTPUT)
	                   (wp-cap-color ?wp CAP_NONE)
	              )
	:effect (and (not (wp-at ?wp ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (wp-at ?wp ?m OUTPUT)
	             (not (mps-side-free ?m OUTPUT))
	             (not (wp-cap-color ?wp CAP_NONE))
	             (wp-cap-color ?wp ?capcol)
	             (cs-can-perform ?m RETRIEVE_CAP)
	             (not (cs-can-perform ?m MOUNT_CAP))
	             (not (cs-prepared-for ?m MOUNT_CAP))
	             (not (cs-buffered ?m ?capcol))
	        )
)

(:action cs-retrieve-cap
	:parameters (?m - mps ?cc - cap-carrier ?capcol - cap-color)
	:precondition (and (mps-type ?m CS)
	                   (or (mps-state ?m PROCESSING)
	                       (mps-state ?m READY-AT-OUTPUT)
	                   )
	                   (cs-prepared-for ?m RETRIEVE_CAP)
	                   (wp-at ?cc ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (mps-side-free ?m OUTPUT)
	                   (wp-cap-color ?cc ?capcol)
	              )
	:effect (and (not (wp-at ?cc ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (wp-at ?cc ?m OUTPUT)
	             (not (mps-side-free ?m OUTPUT))
	             (not (wp-cap-color ?cc ?capcol))
	             (wp-cap-color ?cc CAP_NONE)
	             (cs-buffered ?m ?capcol)
	             (cs-can-perform ?m MOUNT_CAP)
	             (not (cs-prepared-for ?m RETRIEVE_CAP))
	        )
)

(:action prepare-rs
	:parameters (?m - mps ?rc - ring-color ?rs-before - ring-num
	             ?rs-after - ring-num ?r-req - ring-num)
	:precondition (and (mps-type ?m RS)
	                   (mps-state ?m IDLE)
	                   (rs-ring-spec ?m ?rc ?r-req)
	                   (rs-filled-with ?m ?rs-before)
	                   (rs-sub ?rs-before ?r-req ?rs-after)
	              )
	:effect (and (not (mps-state ?m IDLE))
	             (mps-state ?m READY-AT-OUTPUT)
	             (rs-prepared-color ?m ?rc)
	        )
)

(:action rs-mount-ring1
	:parameters (?m - mps ?wp - workpiece ?col - ring-color
	             ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
	:precondition (and (mps-type ?m RS)
	                   (or (mps-state ?m PROCESSING)
	                       (mps-state ?m READY-AT-OUTPUT)
	                   )
	                   (wp-at ?wp ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (mps-side-free ?m OUTPUT)
	                   (wp-usable ?wp)
	                   (wp-ring1-color ?wp RING_NONE)
	                   (wp-cap-color ?wp CAP_NONE)
	                   (rs-prepared-color ?m ?col)
	                   (rs-ring-spec ?m ?col ?r-req)
	                   (rs-filled-with ?m ?rs-before)
	                   (rs-sub ?rs-before ?r-req ?rs-after)
	              )
	:effect (and (not (rs-prepared-color ?m ?col))
	             (not (wp-at ?wp ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (wp-at ?wp ?m OUTPUT)
	             (not (mps-side-free ?m OUTPUT))
	             (not (wp-ring1-color ?wp RING_NONE))
	             (wp-ring1-color ?wp ?col)
	             (not (rs-filled-with ?m ?rs-before))
	             (rs-filled-with ?m ?rs-after)
	        )
)

(:action rs-mount-ring2
	:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color
	             ?rs-before - ring-num ?rs-after - ring-num ?r-req - ring-num)
	:precondition (and (mps-type ?m RS)
	                   (or (mps-state ?m PROCESSING)
	                       (mps-state ?m READY-AT-OUTPUT)
	                   )
	                   (wp-at ?wp ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (wp-usable ?wp)
	                   (mps-side-free ?m OUTPUT)
	                   (wp-ring1-color ?wp ?col1)
	                   (wp-ring2-color ?wp RING_NONE)
	                   (wp-cap-color ?wp CAP_NONE)
	                   (rs-prepared-color ?m ?col)
	                   (rs-ring-spec ?m ?col ?r-req)
	                   (rs-filled-with ?m ?rs-before)
	                   (rs-sub ?rs-before ?r-req ?rs-after)
	              )
	:effect (and (not (rs-prepared-color ?m ?col))
	             (not (wp-at ?wp ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (wp-at ?wp ?m OUTPUT)
	             (not (mps-side-free ?m OUTPUT))
	             (not (wp-ring2-color ?wp RING_NONE))
	             (wp-ring2-color ?wp ?col)
	             (not (rs-filled-with ?m ?rs-before))
	             (rs-filled-with ?m ?rs-after)
	        )
)

(:action rs-mount-ring3
	:parameters (?m - mps ?wp - workpiece ?col - ring-color ?col1 - ring-color
	             ?col2 - ring-color ?rs-before - ring-num ?rs-after - ring-num
				 ?r-req - ring-num)
	:precondition (and (mps-type ?m RS)
	                   (or (mps-state ?m PROCESSING)
	                       (mps-state ?m READY-AT-OUTPUT)
	                   )
	                   (wp-at ?wp ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (wp-usable ?wp)
	                   (mps-side-free ?m OUTPUT)
	                   (wp-ring1-color ?wp ?col1)
	                   (wp-ring2-color ?wp ?col2)
	                   (wp-ring3-color ?wp RING_NONE)
	                   (wp-cap-color ?wp CAP_NONE)
	                   (rs-prepared-color ?m ?col)
	                   (rs-ring-spec ?m ?col ?r-req)
	                   (rs-filled-with ?m ?rs-before)
	                   (rs-sub ?rs-before ?r-req ?rs-after)
	              )
	:effect (and (not (rs-prepared-color ?m ?col))
	             (not (wp-at ?wp ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (wp-at ?wp ?m OUTPUT)
	             (not (mps-side-free ?m OUTPUT))
	             (not (wp-ring3-color ?wp RING_NONE))
	             (wp-ring3-color ?wp ?col)
	             (not (rs-filled-with ?m ?rs-before))
	             (rs-filled-with ?m ?rs-after)
	        )
)

; The following is the generic move version.
; It takes the robot from any location (at any side) to any MPS (any side).
; However, this also creates a tremendous number of options during search and
; hence is detrimental for planning performance.
;
(:action go-wait
	:parameters (?r - robot ?from - location ?from-side - mps-side
	             ?to - waitpoint)
	:precondition (at ?r ?from ?from-side)
	:effect (and (not (at ?r ?from ?from-side))
	             (at ?r ?to WAIT)
	        )
)

(:action wait
	:parameters (?r - robot ?point - waitpoint)
	:precondition (at ?r ?point WAIT)
	:effect (at ?r ?point WAIT)
)

(:action wait-for-wp
	:parameters (?r - robot ?m - mps ?side - mps-side)
	:precondition (at ?r ?m ?side)
	:effect (at ?r ?m ?side)
)

(:action wait-for-free-side
	:parameters (?r - robot ?m - mps ?side - mps-side)
	:precondition (at ?r ?m ?side)
	:effect (at ?r ?m ?side)
)

(:action move
	:parameters (?r - robot ?from - location ?from-side - mps-side
	             ?to - mps ?to-side - mps-side)
	:precondition (at ?r ?from ?from-side)
	:effect (and (not (at ?r ?from ?from-side))
	             (at ?r ?to ?to-side)
	        )
)

(:action enter-field
	:parameters (?r - robot ?team-color - team-color)
	:precondition (not (entered-field ?r))
	:effect (and (entered-field ?r)
	             (at ?r START INPUT)
	        )
)

(:action wp-discard
	:parameters (?r - robot ?cc - workpiece)
	:precondition (holding ?r ?cc)
	:effect (and (not (holding ?r ?cc))
	             (wp-unused ?cc)
	             (not (wp-usable ?cc))
	             (can-hold ?r)
	        )
)

(:action wp-get-shelf
	:parameters (?r - robot ?cc - cap-carrier ?m - mps ?spot - shelf-spot)
	:precondition (and (at ?r ?m INPUT)
	                   (wp-on-shelf ?cc ?m ?spot)
	                   (can-hold ?r)
	              )
	:effect (and (holding ?r ?cc)
	             (not (can-hold ?r))
	             (not (wp-on-shelf ?cc ?m ?spot))
	             (wp-usable ?cc)
	             (spot-free ?m ?spot)
	        )
)

(:action refill-shelf
	:parameters (?m - mps ?spot - shelf-spot ?cc - cap-carrier
	             ?color - cap-color)
	:precondition (spot-free ?m ?spot)
	:effect (and (not (spot-free ?m ?spot))
	             (wp-on-shelf ?cc ?m ?spot)
	             (not (wp-unused ?cc))
	             (wp-cap-color ?cc ?color)
	        )
)

(:action wp-get
	:parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side)
	:precondition (and (at ?r ?m ?side)
	                   (can-hold ?r)
	                   (wp-at ?wp ?m ?side)
	                   (wp-usable ?wp)
	                   (not (mps-side-free ?m ?side))
	              )
	:effect (and (not (wp-at ?wp ?m ?side))
	             (holding ?r ?wp)
	             (not (can-hold ?r))
	             (not (mps-state ?m READY-AT-OUTPUT))
	             (mps-state ?m IDLE)
	             (mps-side-free ?m ?side)
	        )
)

(:action wp-put
	:parameters (?r - robot ?wp - workpiece ?m - mps)
	:precondition (and (at ?r ?m INPUT)
	                   (wp-usable ?wp)
	                   (holding ?r ?wp)
	                   (mps-side-free ?m INPUT)
	              )
	:effect (and (wp-at ?wp ?m INPUT)
	             (not (holding ?r ?wp))
	             (can-hold ?r)
	             (not (mps-side-free ?m INPUT))
	        )
)

(:action wp-put-slide-cc
	:parameters (?r - robot ?wp - cap-carrier ?m - mps ?rs-before - ring-num
	             ?rs-after - ring-num)
	:precondition (and (mps-type ?m RS)
	                   (not (mps-state ?m BROKEN))
	                   (at ?r ?m INPUT)
	                   (wp-usable ?wp)
	                   (holding ?r ?wp)
	                   (rs-filled-with ?m ?rs-before)
	                   (rs-inc ?rs-before ?rs-after)
	              )
	:effect (and (not (wp-usable ?wp))
                 (not (holding ?r ?wp))
	             (can-hold ?r)
	             (not (rs-filled-with ?m ?rs-before))
	             (rs-filled-with ?m ?rs-after)
	        )
)

(:action fulfill-order-c0
	:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
	             ?basecol - base-color ?capcol - cap-color)
	:precondition (and (wp-at ?wp ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (wp-usable ?wp)
	                   (mps-type ?m DS)
	                   (ds-prepared-order ?m ?ord)
	                   (order-complexity ?ord C0)
	                   (order-base-color ?ord ?basecol)
	                   (wp-base-color ?wp ?basecol)
	                   (order-cap-color ?ord ?capcol)
	                   (wp-cap-color ?wp ?capcol)
	                   (wp-ring1-color ?wp RING_NONE)
	                   (wp-ring2-color ?wp RING_NONE)
	                   (wp-ring3-color ?wp RING_NONE)
	              )
	:effect (and (order-fulfilled ?ord)
	             (not (wp-at ?wp ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (not (ds-prepared-order ?m ?ord))
	             (not (wp-usable ?wp))
	             (not (wp-base-color ?wp ?basecol))
	             (not (wp-cap-color ?wp ?capcol))
	        )
)

(:action fulfill-order-c1
	:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
	             ?basecol - base-color ?capcol - cap-color
				 ?ring1col - ring-color)
	:precondition (and (wp-at ?wp ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (wp-usable ?wp)
	                   (mps-type ?m DS)
	                   (ds-prepared-order ?m ?ord)
	                   (order-complexity ?ord C1)
	                   (order-base-color ?ord ?basecol)
	                   (wp-base-color ?wp ?basecol)
	                   (order-ring1-color ?ord ?ring1col)
	                   (wp-ring1-color ?wp ?ring1col)
	                   (order-cap-color ?ord ?capcol)
	                   (wp-cap-color ?wp ?capcol)
	              )
	:effect (and (order-fulfilled ?ord)
	             (not (wp-at ?wp ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (not (ds-prepared-order ?m ?ord))
	             (not (wp-usable ?wp))
	             (not (wp-base-color ?wp ?basecol))
	             (not (wp-cap-color ?wp ?capcol))
	        )
)

(:action fulfill-order-c2
	:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
	             ?basecol - base-color ?capcol - cap-color
	             ?ring1col - ring-color ?ring2col - ring-color)
	:precondition (and (wp-at ?wp ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (wp-usable ?wp)
	                   (mps-type ?m DS)
	                   (ds-prepared-order ?m ?ord)
	                   (order-complexity ?ord C2)
	                   (order-base-color ?ord ?basecol)
	                   (wp-base-color ?wp ?basecol)
	                   (order-ring1-color ?ord ?ring1col)
	                   (wp-ring1-color ?wp ?ring1col)
	                   (order-ring2-color ?ord ?ring2col)
	                   (wp-ring2-color ?wp ?ring2col)
	                   (wp-ring3-color ?wp RING_NONE)
	                   (order-cap-color ?ord ?capcol)
	                   (wp-cap-color ?wp ?capcol)
	              )
	:effect (and (order-fulfilled ?ord)
	             (not (wp-at ?wp ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (not (ds-prepared-order ?m ?ord))
	             (not (wp-usable ?wp))
	             (not (wp-base-color ?wp ?basecol))
	             (not (wp-cap-color ?wp ?capcol))
	        )

)

(:action fulfill-order-c3
	:parameters (?ord - order ?wp - workpiece ?m - mps ?g - ds-gate
	             ?basecol - base-color ?capcol - cap-color
	             ?ring1col - ring-color ?ring2col - ring-color
	             ?ring3col - ring-color)
	:precondition (and (wp-at ?wp ?m INPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (wp-usable ?wp)
	                   (mps-type ?m DS)
	                   (ds-prepared-order ?m ?ord)
	                   (order-complexity ?ord C3)
	                   (order-base-color ?ord ?basecol)
	                   (wp-base-color ?wp ?basecol)
	                   (order-ring1-color ?ord ?ring1col)
	                   (wp-ring1-color ?wp ?ring1col)
	                   (order-ring2-color ?ord ?ring2col)
	                   (wp-ring2-color ?wp ?ring2col)
	                   (order-ring3-color ?ord ?ring3col)
	                   (wp-ring3-color ?wp ?ring3col)
	                   (order-cap-color ?ord ?capcol)
	                   (wp-cap-color ?wp ?capcol)
	              )
	:effect (and (order-fulfilled ?ord)
	             (not (wp-at ?wp ?m INPUT))
	             (mps-side-free ?m INPUT)
	             (not (ds-prepared-order ?m ?ord))
	             (not (wp-usable ?wp))
	             (not (wp-base-color ?wp ?basecol))
	             (not (wp-cap-color ?wp ?capcol))
	        )
)

(:action spawn-wp
	:parameters (?wp - workpiece ?r - robot)
	:precondition (and (not (wp-unused ?wp))
	                   (not (wp-usable ?wp))
	                   (not (wp-spawned-for ?wp ?r))
	              )
	:effect (and (wp-spawned-for ?wp ?r)
	             (wp-unused ?wp)
	             (wp-cap-color ?wp CAP_NONE)
	             (wp-ring1-color ?wp RING_NONE)
	             (wp-ring2-color ?wp RING_NONE)
	             (wp-ring3-color ?wp RING_NONE)
	             (wp-base-color ?wp BASE_NONE)
	        )
)

(:action prepare-ss-to-retrieve
	:parameters (?m - mps ?wp - workpiece ?shelf - ss-shelf ?slot - ss-slot)
	:precondition (and (mps-type ?m SS)
	                   (mps-state ?m IDLE)
	                   (ss-stored-wp ?m ?wp ?shelf ?slot)
	              )
	:effect (and (not (mps-state ?m IDLE))
	             (mps-state ?m PREPARED)
	             (ss-prepared-for ?m RETRIEVE ?wp ?shelf ?slot)
	        )
)

(:action prepare-ss-to-store
	:parameters (?m - mps ?wp - workpiece ?shelf - ss-shelf ?slot - ss-slot)
	:precondition (and (mps-type ?m SS)
	                   (mps-state ?m IDLE)
	                   (ss-shelf-slot-free ?m ?shelf ?slot)
	              )
	:effect (and (not (mps-state ?m IDLE))
	                   (mps-state ?m PREPARED)
	                   (ss-prepared-for ?m STORE ?wp ?shelf ?slot)
	        )
)

(:action prepare-ss-to-assign-wp
	:parameters (?m - mps ?r - robot ?old-wp - workpiece ?wp - workpiece
	             ?shelf - ss-shelf ?slot - ss-slot
	             ?base-col - base-color ?ring1-col - ring-color
	             ?ring2-col - ring-color ?ring3-col - ring-color
	             ?cap-col - cap-color)
	:precondition (and (mps-type ?m SS)
	                   (wp-spawned-for ?wp ?r)
	                   (wp-unused ?wp)
	                   (wp-base-color ?wp BASE_NONE)
	                   (wp-ring1-color ?wp RING_NONE)
	                   (wp-ring2-color ?wp RING_NONE)
	                   (wp-ring3-color ?wp RING_NONE)
	                   (wp-cap-color ?wp CAP_NONE)
	                   (ss-stored-wp ?m ?old-wp ?shelf ?slot)
	                   (ss-new-wp-at ?m ?old-wp ?shelf ?slot ?base-col
	                                 ?ring1-col ?ring2-col ?ring3-col ?cap-col)
	              )
	:effect (and (not (ss-stored-wp ?m ?old-wp ?shelf ?slot))
	             (ss-stored-wp ?m ?wp ?shelf ?slot)
	             (not (wp-spawned-for ?wp ?r))
	             (not (wp-base-color ?wp BASE_NONE))
	             (not (wp-ring1-color ?wp RING_NONE))
	             (not (wp-ring2-color ?wp RING_NONE))
	             (not (wp-ring3-color ?wp RING_NONE))
	             (not (wp-cap-color ?wp CAP_NONE))
	             (wp-base-color ?wp ?base-col)
	             (wp-ring1-color ?wp ?ring1-col)
	             (wp-ring2-color ?wp ?ring2-col)
	             (wp-ring3-color ?wp ?ring3-col)
	             (wp-cap-color ?wp ?cap-col)
	             (wp-usable ?wp)
	             (not (wp-unused ?wp))
	             (wp-unused ?old-wp)
	             (not (wp-usable ?old-wp))
	             (not (ss-new-wp-at ?m ?old-wp ?shelf ?slot ?base-col
	                                ?ring1-col ?ring2-col ?ring3-col
	                                ?cap-col))
	        )
)

(:action ss-retrieve-wp
	:parameters (?m - mps ?old-wp - workpiece ?wp - workpiece
	             ?shelf - ss-shelf ?slot - ss-slot)
	:precondition (and (mps-type ?m SS)
	                   (or (mps-state ?m PROCESSING)
	                       (mps-state ?m READY-AT-OUTPUT)
	                   )
	                   (ss-prepared-for ?m RETRIEVE ?wp ?shelf ?slot)
	                   (mps-side-free ?m INPUT)
	                   (mps-side-free ?m OUTPUT)
	                   (not (ss-shelf-slot-free ?m ?shelf ?slot))
	                   (ss-stored-wp ?m ?wp ?shelf ?slot)
	              )
	:effect (and (wp-at ?wp ?m OUTPUT)
	             (not (mps-side-free ?m OUTPUT))
	             (not (ss-prepared-for ?m RETRIEVE ?wp ?shelf ?slot))
	             (ss-shelf-slot-free ?m ?shelf ?slot)
	             (not (mps-side-free ?m OUTPUT))
	             (not (ss-stored-wp ?m ?wp ?shelf ?slot))
	        )
)

(:action ss-store-wp
	:parameters (?m - mps ?wp - workpiece ?shelf - ss-shelf ?slot - ss-slot)
	:precondition (and (mps-type ?m SS)
	                   (or (mps-state ?m PROCESSING)
	                       (mps-state ?m IDLE)
	                   )
	                   (wp-at ?wp ?m INPUT)
	                   (ss-prepared-for ?m STORE ?wp ?shelf ?slot)
	                   (mps-side-free ?m OUTPUT)
	                   (not (mps-side-free ?m INPUT))
	                   (ss-shelf-slot-free ?m ?shelf ?slot)
	              )
	:effect (and (not (wp-at ?wp ?m INPUT))
	             (not (ss-prepared-for ?m STORE ?wp ?shelf ?slot))
	             (not (ss-shelf-slot-free ?m ?shelf ?slot))
	             (mps-side-free ?m INPUT)
	             (ss-stored-wp ?m ?wp ?shelf ?slot)
	        )
)

(:action move-wp-input-output
	:parameters (?m - mps ?wp - workpiece)
	:precondition (and (wp-at ?wp ?m INPUT)
	                   (mps-side-free ?m OUTPUT)
	                   (not (mps-side-free ?m INPUT))
	              )
	:effect (and (not (wp-at ?wp ?m INPUT))
	             (wp-at ?wp ?m OUTPUT)
	             (not (mps-side-free ?m OUTPUT))
	             (mps-side-free ?m INPUT)
	        )
)

(:action wait-for-reached
	:parameters (?r - robot ?point - waitpoint)
	:precondition (at ?r ?point WAIT)
	:effect (at ?r ?point WAIT)
)

)
