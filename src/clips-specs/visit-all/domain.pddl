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
  (:requirements :strips :typing :universal-preconditions)
  (:types machine marked)
  (:constants
    VISITED UNVISITED - marked
  )
  (:predicates
    (visited-status ?m - machine ?marked - marked)
  )
  (:action visit
    :parameters (?m - machine)
    :precondition (visited-status ?m UNVISITED)
    :effect (visited-status ?m VISITED)
  )
)
