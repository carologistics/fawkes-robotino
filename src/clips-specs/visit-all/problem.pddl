;---------------------------------------------------------------------------
;  problem.pddl - A simple Hello World problem
;
;  Created: Thu 30 Nov 2017 10:31:32 CET
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

(define (problem visit-all-machines)
  (:domain visit-machines)
  (:objects
   ins-c C-BS C-RS1 C-RS2 C-DS C-CS1 C-CS2 - machine
  )
  (:init (visited-status C-BS UNVISITED)
         (visited-status C-RS1 UNVISITED)
         (visited-status C-RS2 UNVISITED)
         (visited-status C-CS1 UNVISITED)
         (visited-status C-CS2 UNVISITED)
         (visited-status C-DS UNVISITED)
  )
  (:goal (and (visited-status C-BS VISITED)
              (visited-status C-RS1 VISITED)
              (visited-status C-RS2 VISITED)
              (visited-status C-CS1 VISITED)
              (visited-status C-CS2 VISITED)
              (visited-status C-DS VISITED))
  )
)
