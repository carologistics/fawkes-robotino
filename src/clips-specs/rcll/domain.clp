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
  (parse-pddl-domain (path-resolve "rcll/domain.pddl"))
  (assert (skiller-control))
  (assert (domain-loaded))
)


(defrule domain-set-sensed-predicates
  " Mark some predicates as sensed predicates.
    That means, the truth value of these predicates can be changed not directly but by some external trigger
  "
  (domain-loaded)
  ?p <- (domain-predicate (name mps-state|location-locked) (sensed FALSE))
=>
  (modify ?p (sensed TRUE))
)


(defrule domain-set-value-predicates
  ?p <- (domain-predicate (name mps-state) (value-predicate FALSE))
=>
  (modify ?p (value-predicate TRUE))
)


(defrule domain-nowait-actions
  " Mark some actions that have a sensed effect as non-waiting. That means the effect is applied without sensing for it "
  (domain-loaded)
	?o <- (domain-operator (name wp-put|wp-get|prepare-bs|prepare-rs|prepare-ds|prepare-cs|location-unlock) (wait-sensed ~FALSE))
=>
	(modify ?o (wait-sensed FALSE))
)


(defrule domain-exogenous-actions
  "Mark all actions, that model state changes of the machines, as exogenous"
  ?op <- (domain-operator
    (name bs-dispense-for-order | bs-dispense-trash | cs-mount-cap | cs-retrieve-cap | rs-mount-ring1 |
          rs-mount-ring2 | rs-mount-ring3 | fulfill-order-c0 |
          fulfill-order-c1 | fulfill-order-c2 | fulfill-order-c3 | ss-retrieve-c0)
    (exogenous FALSE)
  )
=>
  (modify ?op (exogenous TRUE))
)

(defrule domain-worldmodel-flush
	(executive-init)
	(wm-fact (key cx identity))
	(wm-fact (key refbox phase) (value SETUP))
	=>
	(printout warn "Flushing worldmodel!" crlf)
	(wm-robmem-flush)
	(do-for-all-facts ((?df domain-fact)) TRUE
	  (retract ?df)
	)
	(assert (domain-wm-flushed))
)

(deffunction domain-load-local-facts (?self ?team-color)
	"Initialize facts that are not synced."
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
	  (domain-fact (name self) (param-values ?self))
	  (domain-object (name Icks) (type robot))
    (domain-object (name Upsilan) (type robot))
    (domain-object (name Set) (type robot))
    (domain-fact (name at) (param-values ?self START INPUT))
    (domain-fact (name robot-waiting) (param-values ?self))
    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name can-hold) (param-values ?self))
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
    (domain-fact (name cs-color) (param-values ?cs1 CAP_GREY))
    (domain-fact (name cs-color) (param-values ?cs2 CAP_BLACK))
    (domain-fact (name tag-matching) (param-values C-BS INPUT CYAN 65))
    (domain-fact (name tag-matching) (param-values C-CS1 INPUT CYAN 1))
    (domain-fact (name tag-matching) (param-values C-CS2 INPUT CYAN 17))
    (domain-fact (name tag-matching) (param-values C-RS1 INPUT CYAN 33))
    (domain-fact (name tag-matching) (param-values C-RS2 INPUT CYAN 177))
    (domain-fact (name tag-matching) (param-values C-DS INPUT CYAN 81))
    (domain-fact (name tag-matching) (param-values C-SS INPUT CYAN 193))
    (domain-fact (name tag-matching) (param-values C-BS OUTPUT CYAN 66))
    (domain-fact (name tag-matching) (param-values C-CS1 OUTPUT CYAN 2))
    (domain-fact (name tag-matching) (param-values C-CS2 OUTPUT CYAN 18))
    (domain-fact (name tag-matching) (param-values C-RS1 OUTPUT CYAN 34))
    (domain-fact (name tag-matching) (param-values C-RS2 OUTPUT CYAN 178))
    (domain-fact (name tag-matching) (param-values C-DS OUTPUT CYAN 82))
    (domain-fact (name tag-matching) (param-values C-SS OUTPUT CYAN 194))

    (domain-fact (name tag-matching) (param-values M-BS INPUT MAGENTA 161))
    (domain-fact (name tag-matching) (param-values M-CS1 INPUT MAGENTA 97))
    (domain-fact (name tag-matching) (param-values M-CS2 INPUT MAGENTA 113))
    (domain-fact (name tag-matching) (param-values M-RS1 INPUT MAGENTA 129))
    (domain-fact (name tag-matching) (param-values M-RS2 INPUT MAGENTA 145))
    (domain-fact (name tag-matching) (param-values M-DS INPUT MAGENTA 49))
    (domain-fact (name tag-matching) (param-values M-SS INPUT MAGENTA 209))
    (domain-fact (name tag-matching) (param-values M-BS OUTPUT MAGENTA 162))
    (domain-fact (name tag-matching) (param-values M-CS1 OUTPUT MAGENTA 98))
    (domain-fact (name tag-matching) (param-values M-CS2 OUTPUT MAGENTA 114))
    (domain-fact (name tag-matching) (param-values M-RS1 OUTPUT MAGENTA 130))
    (domain-fact (name tag-matching) (param-values M-RS2 OUTPUT MAGENTA 146))
    (domain-fact (name tag-matching) (param-values M-DS OUTPUT MAGENTA 50))
    (domain-fact (name tag-matching) (param-values M-SS OUTPUT MAGENTA 210))

    (domain-fact (name mirror-orientation) (param-values 0 180))
    (domain-fact (name mirror-orientation) (param-values 45 135))
    (domain-fact (name mirror-orientation) (param-values 90 90))
    (domain-fact (name mirror-orientation) (param-values 135 45))
    (domain-fact (name mirror-orientation) (param-values 180 0))
    (domain-fact (name mirror-orientation) (param-values 225 315))
    (domain-fact (name mirror-orientation) (param-values 270 270))
    (domain-fact (name mirror-orientation) (param-values 315 225))

    (domain-object (name CCB1) (type cap-carrier))
    (domain-object (name CCB2) (type cap-carrier))
    (domain-object (name CCB3) (type cap-carrier))
    (domain-object (name CCG1) (type cap-carrier))
    (domain-object (name CCG2) (type cap-carrier))
    (domain-object (name CCG3) (type cap-carrier))
    (domain-object (name ?bs) (type bs))
    (domain-object (name ?cs1) (type cs))
    (domain-object (name ?cs2) (type cs))
    (domain-object (name ?ds) (type ds))
    (domain-object (name ?rs1) (type rs))
    (domain-object (name ?rs2) (type rs))
    (domain-object (name ?ss) (type ss))
    (domain-object (name INPUT) (type mps-side))
    (domain-object (name OUTPUT) (type mps-side))
    (domain-object (name WAIT) (type mps-side))
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
    (domain-fact (name rs-ring-spec) (param-values ?rs1 RING_NONE ZERO))
    (domain-fact (name rs-ring-spec) (param-values ?rs2 RING_NONE ZERO))
	)
)

(defrule domain-load-initial-facts
" Load all initial domain facts on startup of the game "
  (domain-loaded)
  ?flushed <- (domain-wm-flushed)
  (wm-fact (key config agent name) (value ?robot-name))
  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  (wm-fact (key refbox phase) (value SETUP))
  =>
  (retract ?flushed)
  (bind ?self (sym-cat ?robot-name))
  (printout info "Initializing worldmodel" crlf)
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

	(domain-load-local-facts ?self ?team-color)
  (assert
    (domain-object (name SPAWNING-MASTER) (type master-token))
    (domain-object (name PRODUCE-EXCLUSIVE-COMPLEXITY) (type token))

    (domain-fact (name wp-base-color) (param-values WP1 BASE_NONE))
    (domain-fact (name wp-cap-color) (param-values WP1 CAP_NONE))
    (domain-fact (name wp-ring1-color) (param-values WP1 RING_NONE))
    (domain-fact (name wp-ring2-color) (param-values WP1 RING_NONE))
    (domain-fact (name wp-ring3-color) (param-values WP1 RING_NONE))
    (domain-fact (name wp-unused) (param-values WP1))

    (domain-fact (name cs-can-perform) (param-values ?cs1 RETRIEVE_CAP))
    (domain-fact (name cs-can-perform) (param-values ?cs2 RETRIEVE_CAP))
    (domain-fact (name cs-free) (param-values ?cs1))
    (domain-fact (name cs-free) (param-values ?cs2))
    (domain-fact (name cs-color) (param-values ?cs1 CAP_GREY))
    (domain-fact (name cs-color) (param-values ?cs2 CAP_BLACK))

    (domain-fact (name mps-side-free) (param-values ?bs INPUT))
    (domain-fact (name mps-side-free) (param-values ?cs1 INPUT))
    (domain-fact (name mps-side-free) (param-values ?cs2 INPUT))
    (domain-fact (name mps-side-free) (param-values ?rs1 INPUT))
    (domain-fact (name mps-side-free) (param-values ?rs2 INPUT))
    (domain-fact (name mps-side-free) (param-values ?ds INPUT))
    (domain-fact (name mps-side-free) (param-values ?ss INPUT ))
    (domain-fact (name mps-side-free) (param-values ?bs OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?cs1 OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?cs2 OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?rs1 OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?rs2 OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?ds OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?ss OUTPUT))
    (domain-fact (name mps-has-side) (param-values ?bs INPUT))
    (domain-fact (name mps-has-side) (param-values ?cs1 INPUT))
    (domain-fact (name mps-has-side) (param-values ?cs2 INPUT))
    (domain-fact (name mps-has-side) (param-values ?rs1 INPUT))
    (domain-fact (name mps-has-side) (param-values ?rs2 INPUT))
    (domain-fact (name mps-has-side) (param-values ?ds INPUT))
    (domain-fact (name mps-has-side) (param-values ?ss INPUT ))
    (domain-fact (name mps-has-side) (param-values ?bs OUTPUT))
    (domain-fact (name mps-has-side) (param-values ?cs1 OUTPUT))
    (domain-fact (name mps-has-side) (param-values ?cs2 OUTPUT))
    (domain-fact (name mps-has-side) (param-values ?rs1 OUTPUT))
    (domain-fact (name mps-has-side) (param-values ?rs2 OUTPUT))
    (domain-fact (name mps-has-side) (param-values ?ds OUTPUT))
    (domain-fact (name mps-has-side) (param-values ?ss OUTPUT))

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

    (domain-fact (name rs-filled-with) (param-values ?rs1 ZERO))
    (domain-fact (name rs-filled-with) (param-values ?rs2 ZERO))
    (domain-fact (name rs-filled-for) (param-values ?rs1 ZERO))
    (domain-fact (name rs-filled-for) (param-values ?rs2 ZERO))
  )

  (assert (domain-facts-loaded))
)

(defrule domain-restore-worldmodel-after-maintenance
  "Domain facts have not been loaded but the game is already running.
   Restore the world model from the database."
  (not (domain-facts-loaded))
  (wm-fact (key refbox phase) (value EXPLORATION|PRODUCTION))
  (wm-fact (key config agent name) (value ?robot-name))
 (wm-fact (key refbox team-color) (value ?team-color&~nil))

 =>
  (printout warn "Restoring world model from the database" crlf)
	(domain-load-local-facts (sym-cat ?robot-name) ?team-color)
	(wm-robmem-sync-restore)
	(assert (domain-facts-loaded))
)

; ----------------- PDDL Goal Formulation Objects and Predicates -------------------
; the following objects and predicates cary information that make it easier to
; define goal-formulation preconditions in PDDL by either removing the need for
; quantification, or by reducing the size of the grounding space in the case of object
; types.
; (e.g. to detect if the given order has a workpiece assigned to it,
; order-has-wp can be used instead just for that order instead of having to use
; wp-for-order and quantifying over all workpieces)

(defrule domain-assert-mps-polymorphism
    (domain-object (name ?mps) (type bs|cs|ds|rs|ss))
    =>
    (assert (domain-object (name ?mps) (type mps)))
)
(defrule domain-assert-fs-polymorphism
    (domain-object (name ?mps) (type cs|ss))
    =>
    (assert (domain-object (name ?mps) (type fs)))
)

(defrule domain-assert-order-has-wp
  (domain-fact (name wp-for-order) (param-values ?wp ?order))
  (not (domain-fact (name order-has-wp) (param-values ?order)))
  =>
  (assert (domain-fact (name order-has-wp) (param-values ?order)))
)

(defrule domain-retract-order-has-wp
  ?df <- (domain-fact (name order-has-wp) (param-values ?order))
  (not (domain-fact (name wp-for-order) (param-values ?wp ?order)))
  =>
  (retract ?df)
)

(defrule goal-class-wp-has-order
    (wm-fact (key domain fact wp-for-order args? wp ?wp ord ?order))
    (not (domain-fact (name wp-has-order) (param-values ?wp)))
    =>
    (assert (domain-fact (name wp-has-order) (param-values ?wp)))
)

(defrule goal-class-not-wp-has-order
    ?wmf <- (domain-fact (name wp-has-order) (param-values ?wp))
    (not (wm-fact (key domain fact wp-for-order args? wp ?wp ord ?order)))
    =>
    (retract ?wmf)
)

(defrule domain-assert-rs-paid-for
  (domain-fact (name rs-filled-with) (param-values ?rs ?bases-filled))
  (not (domain-fact (name rs-filled-with) (param-values ?rs ?other-bases&~?bases-filled)))
  (domain-constant (type ring-num) (value ?required))
  (not (domain-fact (name rs-paid-for) (param-values ?rs ?required)))
  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?required
                                         difference ZERO|ONE|TWO|THREE))
  =>
  (assert (domain-fact (name rs-paid-for) (param-values ?rs ?required)))
)

(defrule domain-retract-rs-paid-for
  (domain-fact (name rs-filled-with) (param-values ?rs ?bases-filled))
  (not (domain-fact (name rs-filled-with) (param-values ?rs ?other-bases&~?bases-filled)))
  (domain-constant (type ring-num) (value ?required))
  ?df <- (domain-fact (name rs-paid-for) (param-values ?rs ?required))
  (not (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
                                         subtrahend ?required
                                         difference ZERO|ONE|TWO|THREE)))
  =>
  (retract ?df)
)
