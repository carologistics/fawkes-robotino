;****************************************************************************
;  domain.pddl: Produce xenonite with multiple robots
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

(define (domain xenonite)
	(:requirements :strips :typing :durative-actions :timed-initial-literals)

	(:types
		robot - object
		location - object
		container - object
		machine - object
		machine-state - object
		material - object
	)

	(:constants
		BASE MACHINE1-INPUT MACHINE1-OUTPUT MACHINE2-INPUT MACHINE2-OUTPUT REGOLITH-MINE1 REGOLITH-MINE2 STORAGE-INPUT CONTAINER-DEPOT - location
		MACHINE1 MACHINE2 - machine
		;C1 C2 C3 C4 - container
		IDLE FILLED OPERATING READY - machine-state
		REGOLITH XENONITE PROCESSITE BYPRODUCT - material
	)

	(:predicates
		(self ?r - robot)
		(ready ?r - robot)
		(robot-at ?r - robot ?l - location)
		(robot-carries ?r - robot ?c - container)
		(robot-can-carry ?r - robot)
		(container-at ?c - container ?l - location)
		(container-filled ?c - container ?m - material)
		(container-can-be-filled ?c - container)
		(machine-in-state ?m - machine ?s - machine-state)
		(machine-for-material ?m - machine ?mat - material)
		(machine-makes-material ?m - machine ?mat - material)
		(location-is-mine ?location - location)
		(location-part-of-machine ?location - location ?machine - machine)
		(location-is-machine ?location - location)
		(location-is-machine-input ?location - location)
		(location-is-machine-output ?location - location)
		(location-is-free ?location - location)
		(location-is-spacious ?location - location)
		(location-is-small ?location - location)
		(storage-is-full)
	)

	(:durative-action collect-regolith
		:parameters (?r - robot ?mine - location ?c - container)
		:duration (= ?duration 2)
		:condition (and
		              (at start (self ?r))
		              (at start (location-is-mine ?mine))
		              (over all (robot-at ?r ?mine))
		              (at start (robot-carries ?r ?c))
		              (at start (container-can-be-filled ?c)))
		:effect (and
		           (at start (not (container-can-be-filled ?c)))
		           (at end (container-filled ?c REGOLITH)))
	)

	(:durative-action put-regolith
		:parameters (?r - robot ?side - location ?machine - machine ?c - container)
		:duration (= ?duration 2)
		:condition (and
		             (at start (self ?r))
		             (at start (location-part-of-machine ?side ?machine))
		             (at start (location-is-machine-input ?side))
		             (at start (machine-in-state ?machine IDLE))
		             (over all (robot-at ?r ?side))
		             (at start (robot-carries ?r ?c))
		             (at start (container-filled ?c REGOLITH))
		             (at start (machine-for-material ?machine REGOLITH)))
		:effect (and
		          (at start (not (container-filled ?c REGOLITH)))
		          (at end (container-can-be-filled ?c))
		          (at end (machine-in-state ?machine FILLED))
		          (at start (not (machine-in-state ?machine IDLE))))
	)

	(:durative-action pick-container
		:parameters (?r - robot ?c - container)
		:duration (= ?duration 2)
		:condition (and
		             (at start (self ?r))
		             (over all (robot-at ?r CONTAINER-DEPOT))
		             (at start (robot-can-carry ?r))
		             (at start (container-at ?c CONTAINER-DEPOT)))
		:effect (and
		          (at start (not (robot-can-carry ?r)))
		          (at end (robot-carries ?r ?c))
		          (at start (not (container-at ?c CONTAINER-DEPOT))))
	)

	(:durative-action put-container
		:parameters (?r - robot ?c - container ?side - location)
		:duration (= ?duration 2)
		:condition (and
		             (at start (self ?r))
		             (over all (robot-at ?r ?side))
		             (at start (robot-carries ?r ?c)))
		:effect (and
		          (at start (not (robot-carries ?r ?c)))
              (at end (robot-can-carry ?r))
		          (at end (container-at ?c ?side)))
	)

	(:durative-action move
		:parameters (?r - robot ?l1 - location ?l2 - location)
		:duration (= ?duration 20)
		:condition (and
		             (at start (self ?r))
		             (at start (location-is-small ?l1))
		             (at start (location-is-small ?l2))
		             (at start (robot-at ?r ?l1))
		             (at start (location-is-free ?l2)))
		:effect (and
		          (at start (not (robot-at ?r ?l1)))
		          (at start (not (location-is-free ?l2)))
		          (at end (robot-at ?r ?l2))
		          (at end (location-is-free ?l1)))
	)

	(:durative-action move-to-plaza
		:parameters (?r - robot ?l1 - location ?l2 - location)
		:duration (= ?duration 20)
		:condition (and
		             (at start (self ?r))
		             (at start (location-is-small ?l1))
		             (at start (location-is-spacious ?l2))
		             (at start (robot-at ?r ?l1)))
		:effect (and
		          (at start (not (robot-at ?r ?l1)))
		          (at end (robot-at ?r ?l2))
		          (at end (location-is-free ?l1)))
	)

	(:durative-action move-from-plaza
		:parameters (?r - robot ?l1 - location ?l2 - location)
		:duration (= ?duration 20)
		:condition (and
		             (at start (self ?r))
		             (at start (location-is-spacious ?l1))
		             (at start (location-is-small ?l2))
		             (at start (robot-at ?r ?l1))
		             (at start (location-is-free ?l2)))
		:effect (and
		          (at start (not (robot-at ?r ?l1)))
		          (at end (robot-at ?r ?l2)))
	)

	(:durative-action move-plaza-plaza
		:parameters (?r - robot ?l1 - location ?l2 - location)
		:duration (= ?duration 20)
		:condition (and
		             (at start (self ?r))
		             (at start (location-is-spacious ?l1))
		             (at start (location-is-spacious ?l2))
		             (at start (robot-at ?r ?l1)))
		:effect (and
		          (at start (not (robot-at ?r ?l1)))
		          (at end (robot-at ?r ?l2)))
	)

	(:durative-action start-machine
		:parameters (?m - machine)
		:duration (= ?duration 15)
		:condition (and
		             (at start (machine-in-state ?m FILLED)))
		:effect (and
		          (at end (machine-in-state ?m READY))
		          (at start (not (machine-in-state ?m FILLED))))
	)

	(:durative-action collect-processite
		:parameters (?r - robot ?machine - machine ?output - location ?c - container)
		:duration (= ?duration 2)
		:condition (and
		             (at start (self ?r))
		             (at start (location-is-machine-output ?output))
		             (at start (machine-in-state ?machine READY))
		             (over all (robot-at ?r ?output))
		             (at start (robot-carries ?r ?c))
		             (at start (container-can-be-filled ?c))
		             (at start (machine-makes-material ?machine PROCESSITE)))
		:effect (and
		          (at start (not (container-can-be-filled ?c)))
		          (at end (container-filled ?c PROCESSITE))
		          (at start (not (machine-in-state ?machine READY)))
		          (at end (machine-in-state ?machine IDLE)))
	)

	(:durative-action put-processite
		:parameters (?r - robot ?side - location ?machine - machine ?c - container)
		:duration (= ?duration 2)
		:condition (and
		             (at start (self ?r))
		             (at start (location-part-of-machine ?side ?machine))
		             (at start (location-is-machine-input ?side))
		             (at start (machine-in-state ?machine IDLE))
		             (over all (robot-at ?r ?side))
		             (at start (robot-carries ?r ?c))
		             (at start (container-filled ?c PROCESSITE))
		             (at start (machine-for-material ?machine PROCESSITE)))
		:effect (and
		          (at start (not (container-filled ?c PROCESSITE)))
		          (at end (container-can-be-filled ?c))
		          (at end (machine-in-state ?machine FILLED))
		          (at start (not (machine-in-state ?machine IDLE))))
	)

	(:durative-action collect-xenonite
		:parameters (?r - robot ?machine - machine ?output - location ?c - container)
		:duration (= ?duration 2)
		:condition (and
		             (at start (self ?r))
		             (at start (location-is-machine-output ?output))
		             (at start (machine-in-state ?machine READY))
		             (over all (robot-at ?r ?output))
		             (at start (robot-carries ?r ?c))
		             (at start (container-can-be-filled ?c))
		             (at start (machine-makes-material ?machine XENONITE)))
		:effect (and
		          (at start (not (container-can-be-filled ?c)))
		          (at end (container-filled ?c XENONITE))
		          (at start (not (machine-in-state ?machine READY)))
		          (at end (machine-in-state ?machine IDLE)))
	)
)
