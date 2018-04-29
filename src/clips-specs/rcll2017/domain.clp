;---------------------------------------------------------------------------
;  domain.clp - Domain configuration
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;             2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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

(defrule domain-load
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "rcll2017/domain.pddl"))
  (assert (domain-loaded))
)

(defrule test-domain-set-sensed-predicates
  (domain-loaded)
  ?p <- (domain-predicate (name mps-state) (sensed FALSE))
=>
  (modify ?p (sensed TRUE))
)

(defrule domain-set-value-predicates
  ?p <- (domain-predicate (name mps-state) (value-predicate FALSE))
=>
  (modify ?p (value-predicate TRUE))
)

(defrule domain-wp-put-nowait
  (domain-loaded)
	?o <- (domain-operator (name wp-put) (wait-sensed ~FALSE))
	=>
	(modify ?o (wait-sensed FALSE))
)

(defrule domain-exogenous-actions
  ?op <- (domain-operator
    (name bs-dispense | cs-mount-cap | cs-retrieve-cap | rs-mount-ring1 |
          rs-mount-ring2 | rs-mount-ring3 | fulfill-order-c0 |
          fulfill-order-c1 | fulfill-order-c2 | fulfill-order-c3)
    (exogenous FALSE)
  )
  =>
  (modify ?op (exogenous TRUE))
)

(defrule domain-load-initial-facts
  (domain-loaded)
  (wm-fact (key config rcll robot-name) (value ?robot-name))
  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  =>
  (bind ?self (sym-cat ?robot-name))
  (if (eq ?team-color CYAN)
    then
        (bind ?bs C-BS)
        (bind ?cs1 C-CS1)
        (bind ?cs2 C-CS2)
        (bind ?rs1 C-RS1)
        (bind ?rs2 C-RS2)
        (bind ?ds C-DS)
        (bind ?ss C-SS)
    else
        (bind ?bs M-BS)
        (bind ?cs1 M-CS1)
        (bind ?cs2 M-CS2)
        (bind ?rs1 M-RS1)
        (bind ?rs2 M-RS2)
        (bind ?ds M-DS)
        (bind ?ss M-SS)
  )

  (assert
    (domain-object (name R-1) (type robot))
    (domain-object (name R-2) (type robot))
    (domain-object (name R-3) (type robot))

    (domain-object (name CCB1) (type cap-carrier))
    (domain-object (name CCB2) (type cap-carrier))
    (domain-object (name CCB3) (type cap-carrier))
    (domain-object (name CCG1) (type cap-carrier))
    (domain-object (name CCG2) (type cap-carrier))
    (domain-object (name CCG3) (type cap-carrier))
    (domain-object (name ?bs) (type mps))
    (domain-object (name ?cs1) (type mps))
    (domain-object (name ?cs2) (type mps))
    (domain-object (name ?ds) (type mps))
    (domain-object (name ?rs1) (type mps))
    (domain-object (name ?rs2) (type mps))
    (domain-object (name ?ss) (type mps))
    (domain-object (name ?team-color) (type team-color))
    (domain-object (name O1) (type order))
    (domain-object (name O2) (type order))
    (domain-object (name O3) (type order))
    (domain-object (name O4) (type order))
    (domain-object (name O5) (type order))
    (domain-object (name O6) (type order))
    (domain-object (name O7) (type order))
    (domain-object (name O8) (type order))
    (domain-object (name O9) (type order))
    (domain-fact (name wp-base-color) (param-values WP1 BASE_NONE))
    (domain-fact (name wp-cap-color) (param-values WP1 CAP_NONE))
    (domain-fact (name wp-ring1-color) (param-values WP1 RING_NONE))
    (domain-fact (name wp-ring2-color) (param-values WP1 RING_NONE))
    (domain-fact (name wp-ring3-color) (param-values WP1 RING_NONE))
    (domain-fact (name wp-unused) (param-values WP1))
    ; (domain-object (name WP1) (type workpiece))

    ; (domain-fact (name wp-base-color) (param-values WP1 BASE_NONE))
    ; (domain-fact (name wp-cap-color) (param-values WP1 CAP_NONE))
    ; (domain-fact (name wp-ring1-color) (param-values WP1 RING_NONE))
    ; (domain-fact (name wp-ring2-color) (param-values WP1 RING_NONE))
    ; (domain-fact (name wp-ring3-color) (param-values WP1 RING_NONE))
    ; (domain-fact (name wp-unused) (param-values WP1))
    ; (domain-fact (name wp-spawned-by) (param-values R-1 WP1))
    (domain-fact (name self) (param-values ?self))
    (domain-fact (name at) (param-values ?self START INPUT))
    (domain-fact (name can-hold) (param-values ?self))
    (domain-fact (name location-free) (param-values START INPUT))
    (domain-fact (name location-free) (param-values ?bs INPUT))
    (domain-fact (name location-free) (param-values ?bs OUTPUT))
    (domain-fact (name location-free) (param-values ?cs1 INPUT))
    (domain-fact (name location-free) (param-values ?cs1 OUTPUT))
    (domain-fact (name location-free) (param-values ?cs2 INPUT))
    (domain-fact (name location-free) (param-values ?cs2 OUTPUT))
    (domain-fact (name location-free) (param-values ?ds INPUT))
    ; (domain-fact (name location-free) (param-values ?ds OUTPUT))
    (domain-fact (name location-free) (param-values ?rs1 INPUT))
    (domain-fact (name location-free) (param-values ?rs1 OUTPUT))
    (domain-fact (name location-free) (param-values ?rs2 INPUT))
    (domain-fact (name location-free) (param-values ?rs2 OUTPUT))

    (domain-fact (name location-free) (param-values ?ss INPUT))
    
    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name mps-team) (param-values ?ds ?team-color))
    (domain-fact (name mps-team) (param-values ?ss ?team-color))
    (domain-fact (name mps-team) (param-values ?cs1 ?team-color))
    (domain-fact (name mps-team) (param-values ?cs2 ?team-color))
    (domain-fact (name mps-team) (param-values ?rs1 ?team-color))
    (domain-fact (name mps-team) (param-values ?rs2 ?team-color))
    
    (domain-fact (name mps-type) (param-values C-BS BS))
    (domain-fact (name mps-type) (param-values C-DS DS))
    (domain-fact (name mps-type) (param-values C-SS SS))
    (domain-fact (name mps-type) (param-values C-CS1 CS))
    (domain-fact (name mps-type) (param-values C-CS2 CS))
    (domain-fact (name mps-type) (param-values C-RS1 RS))
    (domain-fact (name mps-type) (param-values C-RS2 RS))

    (domain-fact (name mps-type) (param-values M-BS BS))
    (domain-fact (name mps-type) (param-values M-DS DS))
    (domain-fact (name mps-type) (param-values M-SS SS))
    (domain-fact (name mps-type) (param-values M-CS1 CS))
    (domain-fact (name mps-type) (param-values M-CS2 CS))
    (domain-fact (name mps-type) (param-values M-RS1 RS))
    (domain-fact (name mps-type) (param-values M-RS2 RS))

    (domain-fact (name cs-can-perform) (param-values ?cs1 RETRIEVE_CAP))
    (domain-fact (name cs-can-perform) (param-values ?cs2 RETRIEVE_CAP))
    (domain-fact (name cs-free) (param-values ?cs1))
    (domain-fact (name cs-free) (param-values ?cs2))
    (domain-fact (name cs-color) (param-values ?cs1 CAP_GREY))
    (domain-fact (name cs-color) (param-values ?cs2 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCB1 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCB2 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCB3 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCG1 CAP_GREY))
    (domain-fact (name wp-cap-color) (param-values CCG2 CAP_GREY))
    (domain-fact (name wp-cap-color) (param-values CCG3 CAP_GREY))
    (domain-fact (name wp-on-shelf) (param-values CCB1 ?cs2 LEFT))
    (domain-fact (name wp-on-shelf) (param-values CCB2 ?cs2 MIDDLE))
    (domain-fact (name wp-on-shelf) (param-values CCB3 ?cs2 RIGHT))
    (domain-fact (name wp-on-shelf) (param-values CCG1 ?cs1 LEFT))
    (domain-fact (name wp-on-shelf) (param-values CCG2 ?cs1 MIDDLE))
    (domain-fact (name wp-on-shelf) (param-values CCG3 ?cs1 RIGHT))
    (domain-fact (name robot-waiting) (param-values ?self))
    (domain-fact (name rs-sub) (param-values THREE TWO ONE))
    (domain-fact (name rs-sub) (param-values THREE ONE TWO))
    (domain-fact (name rs-sub) (param-values THREE ZERO THREE))
    (domain-fact (name rs-sub) (param-values TWO TWO ZERO))
    (domain-fact (name rs-sub) (param-values TWO ONE ONE))
    (domain-fact (name rs-sub) (param-values TWO ZERO TWO))
    (domain-fact (name rs-sub) (param-values ONE ONE ZERO))
    (domain-fact (name rs-sub) (param-values ONE ZERO ONE))
    (domain-fact (name rs-sub) (param-values ZERO ZERO ZERO))
    (domain-fact (name rs-inc) (param-values ZERO ONE))
    (domain-fact (name rs-inc) (param-values ONE TWO))
    (domain-fact (name rs-inc) (param-values TWO THREE))
    (domain-fact (name rs-filled-with) (param-values ?rs1 ZERO))
    (domain-fact (name rs-filled-with) (param-values ?rs2 ZERO))
  )

  (assert (domain-facts-loaded))
)
