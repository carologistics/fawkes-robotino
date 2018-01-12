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

(define (problem rcll-production-fulfill-order)
  (:domain rcll-production)
  (:objects
    R-1 - robot
    <<#ORDEROBJECTS|{relation:'domain-fact',name:'order-complexity'}>>
    <<param_values_0>> - order
    WP<<param_values_0>> - workpiece
    <</ORDEROBJECTS>>
    CCB1 - cap-carrier
    CCB2 - cap-carrier
    CCB3 - cap-carrier
    CCG1 - cap-carrier
    CCG2 - cap-carrier
    CCG3 - cap-carrier

    C-BS C-CS1 C-CS2 C-DS C-RS1 C-RS2 C-SS - mps
    CYAN - team-color
  )
  (:init
    (mps-type C-BS BS)
    (mps-type C-CS1 CS)
    (mps-type C-CS2 CS)
    (mps-type C-DS DS)
    (mps-type C-RS1 RS)
    (mps-type C-RS2 RS)
    (mps-type C-SS SS)
    (location-free START INPUT)
    (location-free C-BS INPUT)
    (location-free C-BS OUTPUT)
    (location-free C-CS1 INPUT)
    (location-free C-CS1 OUTPUT)
    (location-free C-CS2 INPUT)
    (location-free C-CS2 OUTPUT)
    (location-free C-DS INPUT)
    (location-free C-DS OUTPUT)
    (location-free C-RS1 INPUT)
    (location-free C-RS1 OUTPUT)
    (location-free C-RS2 INPUT)
    (location-free C-RS2 OUTPUT)
    (cs-can-perform C-CS1 CS_RETRIEVE)
    (cs-can-perform C-CS2 CS_RETRIEVE)
    (cs-free C-CS1)
    (cs-free C-CS2)
    <<#ORDEROBJECTS|{relation:'domain-fact',name:'order-complexity'}>>
    (wp-base-color WP<<param_values_0>> BASE_NONE)
    (wp-cap-color WP<<param_values_0>> CAP_NONE)
    (wp-ring1-color WP<<param_values_0>> RING_NONE)
    (wp-ring2-color WP<<param_values_0>> RING_NONE)
    (wp-ring3-color WP<<param_values_0>> RING_NONE)
    (wp-unused WP<<param_values_0>>)

    <</ORDEROBJECTS>>
    (wp-cap-color CCB1 CAP_BLACK)
    (wp-cap-color CCB2 CAP_BLACK)
    (wp-cap-color CCB3 CAP_BLACK)
    (wp-cap-color CCG1 CAP_GREY)
    (wp-cap-color CCG2 CAP_GREY)
    (wp-cap-color CCG3 CAP_GREY)

    (wp-on-shelf CCB1 C-CS2 LEFT)
    (wp-on-shelf CCB2 C-CS2 MIDDLE)
    (wp-on-shelf CCB3 C-CS2 RIGHT)

    (wp-on-shelf CCG1 C-CS1 LEFT)
    (wp-on-shelf CCG2 C-CS1 MIDDLE)
    (wp-on-shelf CCG3 C-CS1 RIGHT)

    (robot-waiting R-1)

    (spot-free C-CS1 LEFT)
    (spot-free C-CS1 MIDDLE)
    (spot-free C-CS1 RIGHT)
    (spot-free C-CS2 LEFT)
    (spot-free C-CS2 MIDDLE)
    (spot-free C-CS2 RIGHT)
    <<#DOMAINFACTS|{relation:'domain-fact'}>>(<<name>> <<param_values>>)
    <</DOMAINFACTS>>
  )
  (:goal
    <<GOAL>>
  )
)
