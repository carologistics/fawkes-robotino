;---------------------------------------------------------------------------
;  domain.clp - Domain configuration
;
;  Created: Tue 09 Jan 2018 17:03:31 CET
;  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
  (executive-init)
  (domain-loaded)
  ?p <- (domain-predicate (name mps-state) (sensed FALSE))
=>
  (modify ?p (sensed TRUE))
)

(defrule domain-wp-put-nowait
  (executive-init)
  (domain-loaded)
	?o <- (domain-operator (name wp-put) (wait-sensed ~FALSE))
	=>
	(modify ?o (wait-sensed FALSE))
)

(defrule load-initial-facts
  (executive-init)
  (domain-loaded)
  =>
  (assert
    (domain-object (name R-1) (type robot))
    (domain-object (name WP1) (type workpiece))
    (domain-object (name CCB1) (type cap-carrier))
    (domain-object (name CCB2) (type cap-carrier))
    (domain-object (name CCB3) (type cap-carrier))
    (domain-object (name CCG1) (type cap-carrier))
    (domain-object (name CCG2) (type cap-carrier))
    (domain-object (name CCG3) (type cap-carrier))

    (domain-object (name C-BS) (type mps))
    (domain-object (name C-CS1) (type mps))
    (domain-object (name C-CS2) (type mps))
    (domain-object (name C-DS) (type mps))
    (domain-object (name C-RS1) (type mps))
    (domain-object (name C-RS2) (type mps))
    (domain-object (name C-SS) (type mps))
    (domain-object (name CYAN) (type team-color))
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
    (domain-fact (name location-free) (param-values START INPUT))
    (domain-fact (name location-free) (param-values C-BS INPUT))
    (domain-fact (name location-free) (param-values C-BS OUTPUT))
    (domain-fact (name location-free) (param-values C-CS1 INPUT))
    (domain-fact (name location-free) (param-values C-CS1 OUTPUT))
    (domain-fact (name location-free) (param-values C-CS2 INPUT))
    (domain-fact (name location-free) (param-values C-CS2 OUTPUT))
    (domain-fact (name location-free) (param-values C-DS INPUT))
    (domain-fact (name location-free) (param-values C-DS OUTPUT))
    (domain-fact (name location-free) (param-values C-RS1 INPUT))
    (domain-fact (name location-free) (param-values C-RS1 OUTPUT))
    (domain-fact (name location-free) (param-values C-RS2 INPUT))
    (domain-fact (name location-free) (param-values C-RS2 OUTPUT))
    (domain-fact (name cs-can-perform) (param-values C-CS1 RETRIEVE_CAP))
    (domain-fact (name cs-can-perform) (param-values C-CS2 RETRIEVE_CAP))
    (domain-fact (name cs-free) (param-values C-CS1))
    (domain-fact (name cs-free) (param-values C-CS2))
    (domain-fact (name wp-cap-color) (param-values CCB1 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCB2 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCB3 CAP_BLACK))
    (domain-fact (name wp-cap-color) (param-values CCG1 CAP_GREY))
    (domain-fact (name wp-cap-color) (param-values CCG2 CAP_GREY))
    (domain-fact (name wp-cap-color) (param-values CCG3 CAP_GREY))
    (domain-fact (name wp-on-shelf) (param-values CCB1 C-CS2 LEFT))
    (domain-fact (name wp-on-shelf) (param-values CCB2 C-CS2 MIDDLE))
    (domain-fact (name wp-on-shelf) (param-values CCB3 C-CS2 RIGHT))
    (domain-fact (name wp-on-shelf) (param-values CCG1 C-CS1 LEFT))
    (domain-fact (name wp-on-shelf) (param-values CCG2 C-CS1 MIDDLE))
    (domain-fact (name wp-on-shelf) (param-values CCG3 C-CS1 RIGHT))
    (domain-fact (name robot-waiting) (param-values R-1))
    (domain-fact (name spot-free) (param-values C-CS1 LEFT))
    (domain-fact (name spot-free) (param-values C-CS1 MIDDLE))
    (domain-fact (name spot-free) (param-values C-CS1 RIGHT))
    (domain-fact (name spot-free) (param-values C-CS2 LEFT))
    (domain-fact (name spot-free) (param-values C-CS2 MIDDLE))
    (domain-fact (name spot-free) (param-values C-CS2 RIGHT))
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
    (domain-fact (name rs-filled-with) (param-values C-RS1 ZERO))
    (domain-fact (name rs-filled-with) (param-values C-RS2 ZERO))
  )
)
