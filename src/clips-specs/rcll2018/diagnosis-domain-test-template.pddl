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
	  action-type - object
		robot - object
		location - object
		mps - location
		mps-side - object
		workpiece - object
		num - object
		cap-carrier - workpiece
		state - object
		component - object
	)

	(:constants
		START - location
		<<#constants>>
    ONE - num
    TWO - num
    THREE - num
    FOUR - num
		;INPUT OUTPUT - mps-side
	)


	(:predicates
		(self ?r - robot)
		(done)
		(next-wp-get ?r - robot)
		(last-begin)
		(next-finish)
		(next-move ?r - robot ?to - mps ?to-side - mps-side)
		(next-wp-put ?r - robot)
		(last-wp-get ?r - robot)
		(last-wp-put ?r - robot)
		(last-move ?r - robot ?to - mps ?to-side - mps-side)
		(at ?r - robot ?m - location ?side - mps-side)
		(holding ?r - robot ?wp - workpiece)
		(can-hold ?r - robot)
		(location-free ?l - location ?side - mps-side)
		(wp-at ?wp - workpiece ?m - mps ?side - mps-side)
		(plus ?n1 - num ?n2 - num)
		(num-ex ?a - action-type ?n - num)
		(first ?a - action-type)
		(comp-state ?c - component ?s - state)
	)

  (:functions (total-cost) - number)

	(:action wp-get
		:parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side)
		:precondition (and (next-wp-get ?r) (comp-state GRIPPER CALIBRATED) (at ?r ?m ?side))
		:effect (and (not (next-wp-get ?r)) (last-wp-get ?r)
		              (when
		                    (and (wp-at ?wp ?m ?side))
		                    (and (not (wp-at ?wp ?m ?side)) (holding ?r ?wp) (not (can-hold ?r))))
								 )
	)
	(:action wp-put
		:parameters (?r - robot ?wp - workpiece ?m - mps)
		:precondition (and (next-wp-put ?r) (comp-state GRIPPER CALIBRATED) (at ?r ?m INPUT))
		:effect (and (not (next-wp-put ?r)) (last-wp-put ?r)
		              (when
		                    (and (holding ?r ?wp))
		                    (and (wp-at ?wp ?m INPUT) (not (holding ?r ?wp)) (can-hold ?r)))
		             )
	)

	(:action move
		:parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
		:precondition (and (next-move ?r ?to ?to-side)
										(comp-state NAVGRAPH LOCALIZED)
										(comp-state MOVE-BASE INIT)
										(at ?r ?to ?to-side))
		:effect (and (not (at ?r ?from ?from-side))
								 (not (location-free ?to ?to-side))	(location-free ?from ?from-side)
								 (not (next-move ?r ?to ?to-side))
								 (last-move ?r ?to ?to-side)
								 )
	)

; ----------------------------- action alternatives ----------------------------
  (:action move-stuck
    :parameters (?r - robot ?from - location ?from-side - mps-side ?to - mps ?to-side - mps-side)
    :precondition (and (next-move ?r ?to ?to-side) (comp-state MOVE-BASE LOCKED))
    :effect (and (not (next-move ?r ?to ?to-side)) (last-move ?r ?to ?to-side))
  )

  (:action wp-get-decalib
    :parameters (?r - robot ?wp - workpiece ?m - mps ?side - mps-side)
    :precondition (and (next-wp-get ?r) (comp-state GRIPPER INIT) (at ?r ?m ?side))
    :effect (and (not (next-wp-get ?r)) (last-wp-get ?r)
             (when (wp-at ?wp ?m ?side) (not (wp-at ?wp ?m ?side)))
             (when (holding ?r ?wp) (not (holding ?r ?wp)))
             (done))
  )

  (:action wp-put-decalib
    :parameters (?r - robot ?wp - workpiece ?m - mps )
    :precondition (and (next-wp-put ?r) (comp-state GRIPPER INIT) (at ?r ?m INPUT))
    :effect (and (when (holding ?r ?wp) (not (holding ?r ?wp)))
                 (not (next-wp-put ?r )) (last-wp-put ?r) (can-hold ?r))
  )



; ----------------------------- exog actions ------------------------------

  (:action drop
    :parameters (?r - robot ?wp - workpiece)
    :precondition (holding ?r ?wp)
    :effect (and (not (holding ?r ?wp))
            (increase (total-cost) 1))
  )

	<<#exog-actions>>

  <<#order-actions>>
)
