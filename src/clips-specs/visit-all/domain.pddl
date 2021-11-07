;---------------------------------------------------------------------------
;  domain.pddl - A simple hello world domain
;
;  Created: Wed 29 Nov 2017 15:42:46 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

(define (domain visit-machines)
  (:requirements :strips :typing)
  (:types
    robot - object
    team-color - object
    location - object
    mps - location
    mps-typename - object
    mps-statename - object
    mps-side - object
  )
  (:constants
    CYAN MAGENTA - team-color
    START - location
    BS CS DS RS SS - mps-typename
    IDLE BROKEN PREPARED PROCESSING PROCESSED WAIT-IDLE READY-AT-OUTPUT DOWN - mps-statename
    INPUT OUTPUT WAIT - mps-side
  )
  (:predicates
    (visited ?m - mps)
    (team-color ?t - team-color)
    (at ?r - robot ?m - location ?side - mps-side)
    (entered-field ?r - robot)
    (robot-waiting ?r - robot)
    (mps-type ?m - mps ?t - mps-typename)
    (mps-state ?m - mps ?s - mps-statename)
    (mps-team ?m - mps ?col - team-color)
    (mps-side-free ?m - mps ?side - mps-side)
  )
  (:action visit
    :parameters (?to - mps ?to-side - mps-side ?t - team-color)
    :precondition (and (team-color ?t)
                       (mps-team ?to ?t)
                       (not (visited ?to)))
    :effect (visited ?to)
  )
)
