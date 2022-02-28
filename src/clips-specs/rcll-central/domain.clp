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
(defglobal
  ?*BBSYNC_PEER_CONFIG* = "/fawkes/bbsync/peers/"
)

(defrule domain-load
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "rcll-central/domain.pddl"))
  (parse-pddl-goal-domain (path-resolve "rcll-central/goal-domain.pddl"))
  (assert (domain-loaded))
)


(defrule domain-set-sensed-predicates
  " Mark some predicates as sensed predicates.
    That means, the truth value of these predicates can be changed not directly but by some external trigger
  "
  (domain-loaded)
  ?p <- (domain-predicate (name mps-state|zone-content) (sensed FALSE))
=>
  (modify ?p (sensed TRUE))
)


(defrule domain-set-value-predicates
  ?p <- (domain-predicate (name mps-state|zone-content) (value-predicate FALSE))
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
    (name bs-dispense | cs-mount-cap | cs-retrieve-cap | rs-mount-ring1 |
          rs-mount-ring2 | rs-mount-ring3 | fulfill-order-c0 |
          fulfill-order-c1 | fulfill-order-c2 | fulfill-order-c3 | ss-retrieve-c0)
    (exogenous FALSE)
  )
=>
  (modify ?op (exogenous TRUE))
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
    (domain-fact (name at) (param-values robot1 START INPUT))
    (domain-fact (name at) (param-values robot2 START INPUT))
    (domain-fact (name at) (param-values robot3 START INPUT))
    (domain-fact (name mps-team) (param-values ?bs ?team-color))
    (domain-fact (name can-hold) (param-values robot1))
    (domain-fact (name can-hold) (param-values robot2))
    (domain-fact (name can-hold) (param-values robot3))
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
    (domain-object (name ?bs) (type mps))
    (domain-object (name ?cs1) (type mps))
    (domain-object (name ?cs2) (type mps))
    (domain-object (name ?ds) (type mps))
    (domain-object (name ?rs1) (type mps))
    (domain-object (name ?rs2) (type mps))
    (domain-object (name ?ss) (type mps))
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
  (wm-fact (key config agent name) (value ?robot-name))
  (wm-fact (key refbox team-color) (value ?team-color&~nil))
  (wm-fact (key refbox phase) (value SETUP))
  =>
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
    (domain-fact (name cs-can-perform) (param-values ?cs1 RETRIEVE_CAP))
    (domain-fact (name cs-can-perform) (param-values ?cs2 RETRIEVE_CAP))
    (domain-fact (name cs-free) (param-values ?cs1))
    (domain-fact (name cs-free) (param-values ?cs2))
    (domain-fact (name cs-color) (param-values ?cs1 CAP_GREY))
    (domain-fact (name cs-color) (param-values ?cs2 CAP_BLACK))
    (domain-fact (name at) (param-values robot1 START INPUT))
    (domain-fact (name at) (param-values robot2 START INPUT))
    (domain-fact (name at) (param-values robot3 START INPUT))
    (domain-fact (name can-hold) (param-values robot1))
    (domain-fact (name can-hold) (param-values robot2))
    (domain-fact (name can-hold) (param-values robot3))
    (domain-fact (name mps-side-free) (param-values ?bs INPUT))
    (domain-fact (name mps-side-free) (param-values ?cs1 INPUT))
    (domain-fact (name mps-side-free) (param-values ?cs2 INPUT))
    (domain-fact (name mps-side-free) (param-values ?rs1 INPUT))
    (domain-fact (name mps-side-free) (param-values ?rs2 INPUT))
    (domain-fact (name mps-side-free) (param-values ?ds INPUT))
    (domain-fact (name mps-side-free) (param-values ?ss INPUT))
    (domain-fact (name mps-side-free) (param-values ?bs OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?cs1 OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?cs2 OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?rs1 OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?rs2 OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?ds OUTPUT))
    (domain-fact (name mps-side-free) (param-values ?ss OUTPUT))
    (domain-fact (name mps-side-approachable) (param-values ?bs INPUT))
    (domain-fact (name mps-side-approachable) (param-values ?cs1 INPUT))
    (domain-fact (name mps-side-approachable) (param-values ?cs2 INPUT))
    (domain-fact (name mps-side-approachable) (param-values ?rs1 INPUT))
    (domain-fact (name mps-side-approachable) (param-values ?rs2 INPUT))
    (domain-fact (name mps-side-approachable) (param-values ?ds INPUT))
    (domain-fact (name mps-side-approachable) (param-values ?ss INPUT))
    (domain-fact (name mps-side-approachable) (param-values ?bs OUTPUT))
    (domain-fact (name mps-side-approachable) (param-values ?cs1 OUTPUT))
    (domain-fact (name mps-side-approachable) (param-values ?cs2 OUTPUT))
    (domain-fact (name mps-side-approachable) (param-values ?rs1 OUTPUT))
    (domain-fact (name mps-side-approachable) (param-values ?rs2 OUTPUT))
    (domain-fact (name mps-side-approachable) (param-values ?ds OUTPUT))
    (domain-fact (name mps-side-approachable) (param-values ?ss OUTPUT))
    (domain-fact (name mps-side-approachable) (param-values ?bs WAIT))
    (domain-fact (name mps-side-approachable) (param-values ?cs1 WAIT))
    (domain-fact (name mps-side-approachable) (param-values ?cs2 WAIT))
    (domain-fact (name mps-side-approachable) (param-values ?rs1 WAIT))
    (domain-fact (name mps-side-approachable) (param-values ?rs2 WAIT))
    (domain-fact (name mps-side-approachable) (param-values ?ds WAIT))
    (domain-fact (name mps-side-approachable) (param-values ?ss WAIT))

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
  )

  (assert (domain-facts-loaded))
)

(defrule domain-load-active-robots-from-bbsync-peers-config
" Initialize all remote robots using the active bbsync connections."
	(not (confval (path ?p&:(str-prefix ?*BBSYNC_PEER_CONFIG* ?p))))
	; only load the info after initial fact flushing
	(domain-facts-loaded)
	=>
	(config-load ?*BBSYNC_PEER_CONFIG*)
	(delayed-do-for-all-facts ((?cf confval))
		(str-prefix ?*BBSYNC_PEER_CONFIG* ?cf:path)
		(bind ?name (sub-string (str-length ?*BBSYNC_PEER_CONFIG*)
		            (str-length ?cf:path)
		            ?cf:path))
		(bind ?r-active (wm-id-to-key (str-cat ?name)))
		(if (and (= (length$ ?r-active) 2) (eq (nth$ 2 ?r-active) active)
		                                   (eq ?cf:value TRUE))
		  then
		    (printout error ?r-active crlf)
		    (bind ?curr-robot (nth$ 1 ?r-active))
		    (assert (wm-fact (key central agent robot args? r ?curr-robot))
		            (domain-object (name ?curr-robot) (type robot))
		            (domain-fact (name at) (param-values ?curr-robot START INPUT))
		            (domain-fact (name robot-waiting) (param-values ?curr-robot))
		            (domain-fact (name at) (param-values ?curr-robot START INPUT)))
		  else
		    (retract ?cf)
		)
	)
)

(defrule start-with-waiting-robots
	(wm-fact (key config rcll start-with-waiting-robots) (value TRUE))
	(wm-fact (key central agent robot args? r ?robot))
	(not (wm-fact (key central agent robot-waiting args? r ?robot)))
	(wm-fact (key refbox phase) (value SETUP))
	=>
	(assert (wm-fact (key central agent robot-waiting args? r ?robot)))
)

;Rules for fixing goal-domain issues
(defrule domain-goal-fix-assert-wp-get-pending
  ?pa <- (plan-action (plan-id ?plan-id)
                       (state PENDING)
                       (action-name wp-get)
                       (param-values $?param-values))
  =>
  (bind ?wp-get-params (create$ (nth$ 2 $?param-values) (nth$ 3 $?param-values) (nth$ 4 $?param-values)))
  (assert (domain-fact (name wp-get-pending) (param-values ?wp-get-params)))
  (printout t "WP-GET DETECTED" $?param-values crlf)
)

(defrule domain-goal-fix-retract-wp-get-pending
(declare (salience 1000))
  ?pa <- (plan-action (plan-id ?plan-id)
                       (state ?state&~PENDING)
                       (action-name wp-get)
                       (param-values $?param-values))
  ?df <- (domain-fact (name wp-get-pending) (param-values $?wp-get-params))
  =>
  (if (eq $?wp-get-params (create$ (nth$ 2 $?param-values) (nth$ 3 $?param-values) (nth$ 4 $?param-values)))
    then
    (retract ?df)
  )
)

(defrule domain-goal-fix-assert-wp-and-order-matching
  "Asserts a domain predicate once a workpiece matches its order"
  ;Order-Facts
  (domain-fact (name order-ring1-color) (param-values ?ord ?ring1))
  (domain-fact (name order-ring2-color) (param-values ?ord ?ring2))
  (domain-fact (name order-ring3-color) (param-values ?ord ?ring3))
  (domain-fact (name order-base-color) (param-values ?ord ?base))
  (domain-fact (name order-cap-color) (param-values ?ord ?cap))
  ;WP-Facts
  (domain-fact (name wp-ring1-color) (param-values ?wp ?ring1))
  (domain-fact (name wp-ring2-color) (param-values ?wp ?ring2))
  (domain-fact (name wp-ring3-color) (param-values ?wp ?ring3))
  (domain-fact (name wp-base-color) (param-values ?wp ?base))
  (domain-fact (name wp-cap-color) (param-values ?wp ?cap))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?ord))
  =>
  (assert (domain-fact (name order-matches-wp) (param-values ?wp ?ord)))
)

(defrule domain-goal-fix-assert-instruct-rs-mount-ring-running
  (goal (class INSTRUCT-RS-MOUNT-RING) (mode EXPANDED|SELECTED|DISPATCHED|COMMITTED))
  =>
  (assert (domain-fact (name instruct-rs-mount-ring-running) (param-values)))
)


(defrule domain-goal-fix-retract-instruct-rs-mount-ring-running
  (not (goal (class INSTRUCT-RS-MOUNT-RING) (mode EXPANDED|SELECTED|DISPATCHED|COMMITTED)))
  ?df <- (domain-fact (name instruct-rs-mount-ring-running) (param-values))
  =>
  (retract ?df)
)

(defrule domain-goal-fix-payment-fillable
  (wm-fact (key domain fact rs-filled-with args? m ?target-mps n ?rs-before&ZERO|ONE|TWO))
  ;check that not to many robots try to fill the rs at the same time
  (or (not (goal (class PAY-FOR-RINGS-WITH-BASE| PAY-FOR-RINGS-WITH-CAP-CARRIER|
                        PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
                 (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
                 (params $? target-mps ?target-mps $?)))
      (and (goal (class PAY-FOR-RINGS-WITH-BASE| PAY-FOR-RINGS-WITH-CAP-CARRIER|
                        PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF)
                 (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)
                 (params $? target-mps ?target-mps $?))
           (test (< (+ (length$ (find-all-facts ((?other-goal goal))
                           (and (or (eq ?other-goal:class PAY-FOR-RINGS-WITH-BASE)
                                    (eq ?other-goal:class PAY-FOR-RINGS-WITH-CAP-CARRIER)
                                    (eq ?other-goal:class PAY-FOR-RINGS-WITH-CARRIER-FROM-SHELF))
                                ;(is-goal-running ?other-goal:mode)
                                (or (eq ?other-goal:mode SELECTED) (eq ?other-goal:mode EXPANDED)
                      	            (eq ?other-goal:mode COMMITTED) (eq ?other-goal:mode DISPATCHED))
                                (member$ ?target-mps ?other-goal:params))))
                       (sym-to-int ?rs-before)) 3))
     )
  )
  =>
  (assert (domain-fact (name rs-payment-fillable) (param-values ?target-mps)))
)

(defrule domain-goal-fix-retract-payment-fillable
  (domain-fact (name rs-filled-with) (param-values ?target-mps THREE))
  ?df <- (domain-fact (name rs-payment-fillable) (param-values ?target-mps))
  =>
  (retract ?df)
)

(defrule domain-goal-fix-next-ring-mountable
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (domain-fact (name rs-ring-spec) (param-values ?mps ?ring-color&~RING_NONE ?bases-needed))
  (domain-fact (name sufficient-payment-for) (param-values ?mps ?ring-color))
  (or
    (and
      (domain-fact (name order-ring1-color) (param-values ?order ?ord-ring1-color&:(eq ?ord-ring1-color ?ring-color)))
      (domain-fact (name wp-ring1-color) (param-values ?wp ?wp-ring1-color&:(eq ?wp-ring1-color RING_NONE)))
    )
    (and
      (domain-fact (name order-ring1-color) (param-values ?order ?ord-ring1-color))
      (domain-fact (name order-ring2-color) (param-values ?order ?ord-ring2-color&:(eq ?ord-ring2-color ?ring-color)))
      (domain-fact (name wp-ring1-color) (param-values ?wp ?wp-ring1-color&:(eq ?wp-ring1-color ?ord-ring1-color)))
      (domain-fact (name wp-ring2-color) (param-values ?wp ?wp-ring2-color&:(eq ?wp-ring2-color RING_NONE)))
    )
    (and
        (domain-fact (name order-ring1-color) (param-values ?order ?ord-ring1-color))
        (domain-fact (name order-ring2-color) (param-values ?order ?ord-ring2-color))
        (domain-fact (name order-ring3-color) (param-values ?order ?ord-ring3-color&:(eq ?ord-ring3-color ?ring-color)))
        (domain-fact (name wp-ring1-color) (param-values ?wp ?wp-ring1-color&:(eq ?wp-ring1-color ?ord-ring1-color)))
        (domain-fact (name wp-ring2-color) (param-values ?wp ?wp-ring2-color&:(eq ?wp-ring2-color ?ord-ring2-color)))
        (domain-fact (name wp-ring3-color) (param-values ?wp ?wp-ring3-color&:(eq ?wp-ring3-color RING_NONE)))
    )
  )
  =>
  (assert (domain-fact (name next-ring-mountable) (param-values ?mps ?order ?wp ?ring-color)))
)

;Formalismus ausdenken
;(defrule domain-goal-fix-retract-next-ring-mountable
;  ?df <- (domain-fact (name next-ring-mountable) (param-values ?mps ?order ?wp ?ring-color))
;  (or
;    (not (domain-fact (name sufficient-payment-for) (param-values ?mps ?ring-color)))
;    (not (name rs-ring-spec) (param-values ?mps ?ring-color&~RING_NONE ?bases-needed))
;    (not (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order)))
;     (not
;      (or
;        (and
;          (domain-fact (name order-ring1-color) (param-values ?order ?ord-ring1-color&:(eq ?ord-ring1-color ?ring-color)))
;          (domain-fact (name wp-ring1-color) (param-values ?wp ?wp-ring1-color&:(eq ?wp-ring1-color RING_NONE)))
;        )
;        (and
;          (domain-fact (name order-ring1-color) (param-values ?order ?ord-ring1-color))
;          (domain-fact (name order-ring2-color) (param-values ?order ?ord-ring2-color&:(eq ?ord-ring2-color ?ring-color)))
;          (domain-fact (name wp-ring1-color) (param-values ?wp ?wp-ring1-color&:(eq ?wp-ring1-color ?ord-ring1-color)))
;          (domain-fact (name wp-ring2-color) (param-values ?wp ?wp-ring2-color&:(eq ?wp-ring2-color RING_NONE)))
;        )
;        (and
;            (domain-fact (name order-ring1-color) (param-values ?order ?ord-ring1-color))
;            (domain-fact (name order-ring2-color) (param-values ?order ?ord-ring2-color))
;            (domain-fact (name order-ring3-color) (param-values ?order ?ord-ring3-color&:(eq ?ord-ring3-color ?ring-color)))
;            (domain-fact (name wp-ring1-color) (param-values ?wp ?wp-ring1-color&:(eq ?wp-ring1-color ?ord-ring1-color)))
;            (domain-fact (name wp-ring2-color) (param-values ?wp ?wp-ring2-color&:(eq ?wp-ring2-color ?ord-ring2-color)))
;            (domain-fact (name wp-ring3-color) (param-values ?wp ?wp-ring3-color&:(eq ?wp-ring3-color RING_NONE)))
;        )
;      )
;    )
;  )
;  =>
;  (retract ?df)
;)

;(defrule domain-goal-fix-sufficient-payment
;  (wm-fact (key domain fact rs-filled-with args? m ?mps n ?bases-filled))
;  (wm-fact (key domain fact rs-ring-spec
;            args? m ?mps r ?ring-color&~RING_NONE rn ?bases-needed))
;  (wm-fact (key domain fact rs-sub args? minuend ?bases-filled
;                                         subtrahend ?bases-needed
;                                         difference ?bases-remain&ZERO|ONE|TWO|THREE))
;  =>
;  (assert (domain-fact (name sufficient-payment-for) (param-values ?mps ?ring-color)))
;)

;(defrule derived-predicate
;  (order-matching-wp ?wp)
;  (forall (wp-for-order ?wp ?ord)
;    (forall (order-ring-color ?ord ?ring1 ONE) (not (wp-ring-color ?wp ?ring1 ONE))
;    )
;  )
;  =>
;
;)
