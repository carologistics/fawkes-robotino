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
    ins-m M-BS M-RS1 M-RS2 M-DS M-CS1 M-CS2 - machine
  )
  (:init (visited ins-c)
         (team-color CYAN)
         (team-machine CYAN C-BS)
         (team-machine CYAN C-RS1)
         (team-machine CYAN C-RS2)
         (team-machine CYAN C-CS1)
         (team-machine CYAN C-CS2)
         (team-machine CYAN C-DS)

         (team-machine MAGENTA M-BS)
         (team-machine MAGENTA M-RS1)
         (team-machine MAGENTA M-RS2)
         (team-machine MAGENTA M-CS1)
         (team-machine MAGENTA M-CS2)
         (team-machine MAGENTA M-DS)
         <<#DOMAINFACTS|{relation:'domain-fact'}>>(<<name>> <<param_values>>)
         <</DOMAINFACTS>>
  )
  (:goal
    <<GOAL>>
  )
)
