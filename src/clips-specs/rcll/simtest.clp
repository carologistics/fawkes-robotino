;---------------------------------------------------------------------------
;  simtest.clp - CX simulation tests
;
;  Created: Wed 19 Jun 2019 15:36:07 CEST
;  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
(defglobal ?*DEFAULT-TESTCASE-HANDLING* = -1)

(deftemplate testcase
	(slot id (type SYMBOL))
	(slot type (type SYMBOL))
	(slot parent (type SYMBOL))
	(slot termination (type SYMBOL) (allowed-values CUSTOM FAILURE SUCCESS) (default CUSTOM))
	(slot state (type SYMBOL) (allowed-values SUCCEEDED FAILED PENDING) (default PENDING))
	(slot msg (type STRING))
	(multislot args)
)

(deffunction simtest-quit (?success)
	(printout info "Exiting (success: " ?success ") ..." crlf)
	(printout info "SIMTEST: " (if ?success then "SUCCEEDED" else "FAILED") crlf)
	(quit)
)

(deffunction simtest-connect (?type $?tests)
	(bind ?name (sym-cat ?type - (gensym*)))
  (foreach ?test-fact ?tests
		(bind ?test-id-fact (modify ?test-fact (parent ?name)))
	)
	(return (assert (testcase (id ?name) (type ?type) (termination FAILURE))))
)

(defrule simtest-initialize
	(not (simtest-initialized))
	(wm-fact (key config simtest enabled) (value TRUE))
	(wm-fact (key config simtest testbed) (value ?testbed))
	=>
	(switch ?testbed
		(case "FAST" then
			(assert (testcase (type POINTS-AFTER-MINUTE) (args minute 1 points 1)))
		)
		(case "DELIVERY" then
			(assert (testcase (type DELIVERY-COUNT) (args count 1)))
		)
		(case "COMPLEX" then
			(simtest-connect OR
			  (assert (testcase (type DELIVERY-COUNT) (args count 5)))
				(simtest-connect AND
					(assert (testcase (type DELIVERY) (termination FAILURE) (args complexity C0)))
					(simtest-connect OR
						(assert (testcase (type DELIVERY) (termination FAILURE) (args complexity C2)))
						(assert (testcase (type DELIVERY) (termination FAILURE) (args complexity C3)))
					)
				)
			)
			(assert (testcase (type FULL-GAME)))
		)
		(case "FULL" then
			(assert (testcase (type POINTS-AFTER-MINUTE) (args minute 8 points 30)))
			(assert (testcase (type POINTS-AFTER-MINUTE) (args minute 17 points 150)))
			(assert (testcase (type DELIVERY) (termination FAILURE) (args complexity C0)))
			(simtest-connect OR
				(assert (testcase (type DELIVERY) (termination FAILURE) (args complexity C2)))
				(assert (testcase (type DELIVERY) (termination FAILURE) (args complexity C3)))
			)
			(assert (testcase (type FULL-GAME)))
		)
		(default none)
	)
	(assert (testcase (type NO-BROKEN-MPS) (termination SUCCESS)))
	(assert (simtest-initialized))
)

(defrule simtest-gen-id
  (testcase (id nil))
 =>
  (do-for-all-facts ((?test testcase))
    (eq ?test:id nil)
    (modify ?test (id (sym-cat TEST- (gensym*))))
  )
)

(defrule simtest-or-success
  ?t <- (testcase (type OR) (id ?name) (state PENDING))
  (testcase (id ?id) (parent ?name) (state SUCCEEDED) (msg ?msg))
  =>
  (modify ?t (state SUCCEEDED) (msg (str-cat ?id SUCCEEDED)))
)

(defrule simtest-and-success
  ?t <- (testcase (type AND) (id ?name) (state PENDING))
  (not (testcase (parent ?name) (state ~SUCCEEDED)))
  =>
  (modify ?t (state SUCCEEDED) (msg (str-cat "all child tests " SUCCEEDED)))
)

(defrule simtest-game-over-default-failure
  (declare (salience ?*DEFAULT-TESTCASE-HANDLING*))
	(wm-fact (key refbox phase) (value POST_GAME))
  =>
	(delayed-do-for-all-facts ((?custom-test testcase))
		(and (eq ?custom-test:termination CUSTOM)
					(eq ?custom-test:state PENDING))
    (modify ?custom-test (state FAILED) (msg "testcase did not terminate"))
  )
)

(defrule simtest-full-game-success
	?testcase <- (testcase (type FULL-GAME) (state PENDING))
	(wm-fact (key refbox phase) (value POST_GAME))
  =>
  (modify ?testcase (state SUCCEEDED) (msg "Game completed"))
)

(defrule simtest-termination-success
	?testcase <- (testcase (termination SUCCESS) (state PENDING))
  (not (testcase (termination CUSTOM) (state PENDING)))
  =>
  (modify ?testcase (state SUCCEEDED) (msg "No failure detected"))
)

(defrule simtest-termination-failure
	?testcase <- (testcase (termination FAILURE) (state PENDING))
  (not (testcase (termination CUSTOM) (state PENDING)))
  =>
  (modify ?testcase (state FAILED) (msg "Fail per default"))
)

(defrule simtest-delivery-success
	?testcase <- (testcase (type DELIVERY) (state PENDING) (args complexity ?com))
	(wm-fact (key refbox team-color) (value ?team-color&~nil))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key domain fact quantity-delivered args? ord ?ord team ?team-color) (value ?delivered&:(> ?delivered 0)))
	(wm-fact (key domain fact order-complexity args? ord ?ord com ?com))
	=>
	(modify ?testcase (state SUCCEEDED) (msg (str-cat "Delivery of complexity " ?com " done")))
)

(defrule simtest-delivery-count-success
	(wm-fact (key refbox team-color) (value ?team-color&~nil))
	(wm-fact (key domain fact quantity-delivered args? ord ? team ?team-color) (value ?delivered&:(> ?delivered 0)))
	?testcase <- (testcase (type DELIVERY-COUNT) (state PENDING) (args count ?count))
	(wm-fact (key refbox phase) (value PRODUCTION))
	=>
	(bind ?curr 0)
  (delayed-do-for-all-facts ((?qd wm-fact))
		(and 	(wm-key-prefix ?qd:key (create$ domain fact quantity-delivered))
					(eq (wm-key-arg ?qd:key team) ?team-color)
					(> ?qd:value 0))
    (bind ?curr (+ ?curr ?qd:value))
  )
  (if (<= ?count ?curr)
		then
			(modify ?testcase (state SUCCEEDED))
	)
)

(defrule simtest-points-after-minute-success
	?testcase <- (testcase (type POINTS-AFTER-MINUTE) (state PENDING)
								(args minute ?time points ?desired-points))
	(wm-fact (key refbox team-color) (value ?team-color&~nil))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key refbox game-time) (values $?gt&:(< (nth$ 1 ?gt) (* ?time 60))))
	(wm-fact (key refbox points ?lc-team-color&:(eq ?lc-team-color (lowcase ?team-color)))
	         (value ?points&:(>= ?points ?desired-points)))
	=>
	(modify ?testcase (state SUCCEEDED) (msg (str-cat "Scored " ?points " points")))
)

(defrule simtest-points-after-minute-failure
	?testcase <- (testcase (type POINTS-AFTER-MINUTE) (state PENDING)
								(args minute ?time points ?desired-points))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key refbox game-time) (values $?gt&:(>= (nth$ 1 ?gt) (* ?time 60))))
	=>
	(modify ?testcase (state FAILED) (msg (str-cat "Insufficient points after " (nth$ 1 ?gt) " seconds")))
)

(defrule simtest-flawless-mps-failure
	?testcase <- (testcase (type NO-BROKEN-MPS) (state PENDING))
	(wm-fact (key domain fact mps-state args? m ?m s BROKEN))
	=>
	(modify ?testcase (state FAILED) (msg (str-cat "Broken MPS " ?m)))
)

(defrule simtest-finished
	(simtest-initialized)
	(not (testcase (state PENDING)))
	=>
	(if (do-for-all-facts ((?testcase testcase))
		(and (eq ?testcase:state FAILED) (eq ?testcase:parent nil))
		(printout error ?testcase:type " failed: " ?testcase:msg crlf)
	)
	 then
		(simtest-quit FALSE)
	else
		(printout info "All tests successful!" crlf)
		(simtest-quit TRUE)
	)
)
