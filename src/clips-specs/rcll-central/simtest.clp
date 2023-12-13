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
	; unique id, is set automatically
	(slot id (type SYMBOL))
	; the test type along with the specified arguments defines the termination behavior
	(slot type (type SYMBOL))
	(multislot args)
	; id of parent in case hierarchical tests are used.
	; is set via simtest-connect function
	; only tests without parents determine the termination status of testbeds
	(slot parent (type SYMBOL))
	; termination behaviour:
	;   CUSTOM:  Specific termination rules required.
	;            Tests with CUSTOM termination determine the termination of testbeds,
	;            hence each testbed should have at least one CUSTOM terminating test.
	;   FAILURE: Specific termination rule required to reach SUCCEEDED state.
	;            PENDING Test is FAILED if all CUSTOM tests terminate.
	;            Suitable if a situation has to be observed eventually.
	;   SUCCESS: Specific termination rule required to reach FAILED state.
	;            PENDING Test is SUCCEEDED if all CUSTOM tests terminate.
	;            Suitable if an erroneous situation has to be avoided.
	(slot termination (type SYMBOL) (allowed-values CUSTOM FAILURE SUCCESS) (default CUSTOM))
	; termination status
	; All tests have to terminate in product for a testbed to be completed.
	; If a test without parent fails, all other tests fail as well.
	(slot state (type SYMBOL) (allowed-values SUCCEEDED FAILED PENDING) (default PENDING))
	; additional information on the reason for test termination
	(slot msg (type STRING))
)

(deffunction simtest-quit (?success)
" Quit test by terminating CLIPS
  @param success termination status of the executed testbed
"
	(printout info "Exiting (success: " ?success ") ..." crlf)
	(printout info "SIMTEST: " (if ?success then "SUCCEEDED" else "FAILED") crlf)
	(quit)
)

(deffunction simtest-connect (?type $?tests)
" Create hierarchical tests with:
  @param ?type type
  @param ?$tests child tests
  @return fact id of parent test
"
	(bind ?name (sym-cat ?type - (gensym*)))
	(foreach ?test-fact ?tests
		(bind ?test-id-fact (modify ?test-fact (parent ?name)))
	)
	(return (assert (testcase (id ?name) (type ?type) (termination FAILURE))))
)

(defrule simtest-initialize
" Creates the configured testbed. "
	(not (simtest-initialized))
	(wm-fact (key config simtest enabled) (value TRUE))
	(wm-fact (key config simtest testbed) (value ?testbed))
	=>
	(switch ?testbed
		; integration test like tests
		(case "ENTER-FIELD" then
			(assert (testcase (type ENTER-FIELD)))
		)
		(case "PRODUCE-C1" then
			(assert (testcase (type DELIVERY-COUNT) (args count 1)))
			(assert (testcase (type DELIVERY) (termination FAILURE) (args complexity C1)))
			(assert (testcase (type POINTS-AFTER-MINUTE) (args minute 12 points 30)))
		)
		; agent performance-focused tests
		(case "FULL" then
			(assert (testcase (type POINTS-AFTER-MINUTE) (args minute 12 points 30)))
			(assert (testcase (type POINTS-AFTER-MINUTE) (args minute 20 points 200)))
			(assert (testcase (type DELIVERY) (termination FAILURE) (args complexity C0)))
			(assert (testcase (type DELIVERY-COUNT) (args count 3)))
			(assert (testcase (type FULL-GAME)))
		)
		; combines conditions for ENTER-FIELD, PRODUCE-C1 and FULL for
		(case "BUILDTEST" then
			(assert (testcase (type ENTER-FIELD)))
			(assert (testcase (type POINTS-AFTER-MINUTE) (args minute 12 points 30)))
			(assert (testcase (type POINTS-AFTER-MINUTE) (args minute 20 points 100)))
			(assert (testcase (type DELIVERY-COUNT) (args count 3)))
			(assert (testcase (type FULL-GAME)))
		)

		(default none)
	)
	;(assert (testcase (type NO-BROKEN-MPS) (termination SUCCESS)))
	(assert (simtest-initialized))
)

(defrule simtest-gen-id
" Create unique ids for each testcase that has no id yet. "
	(testcase (id nil))
	=>
	(do-for-all-facts ((?test testcase))
		(eq ?test:id nil)
		(modify ?test (id (sym-cat TEST- (gensym*))))
	)
)

; ===================== default termination rules ============================

(defrule simtest-game-over-default-failure
" Fail all tests that are still pending after the game with low priority. "
	(declare (salience ?*DEFAULT-TESTCASE-HANDLING*))
	(wm-fact (key refbox phase) (value POST_GAME))
	=>
	(delayed-do-for-all-facts ((?custom-test testcase))
		(and (eq ?custom-test:termination CUSTOM)
		     (eq ?custom-test:state PENDING))
		(modify ?custom-test (state FAILED) (msg "testcase did not terminate"))
	)
)

(defrule simtest-termination-success
" All custom tests are terminated. Succeed all tests that passively watch
  watch out for erroneous situations as no such situation was observed. "
	?testcase <- (testcase (termination SUCCESS) (state PENDING))
	(not (testcase (termination CUSTOM) (state PENDING)))
	=>
	(modify ?testcase (state SUCCEEDED) (msg "No failure detected"))
)

(defrule simtest-termination-failure
" All custom tests are terminated. Fail all tests that passively watch
  watch out for desired situations as no such situation was observed. "
	?testcase <- (testcase (termination FAILURE) (state PENDING))
	(not (testcase (termination CUSTOM) (state PENDING)))
	=>
	(modify ?testcase (state FAILED) (msg "Fail per default"))
)

; ===================== type-specific termination rules =======================

(defrule simtest-or-success
" A child test succeeded, succeed. "
	?t <- (testcase (type OR) (id ?name) (state PENDING))
	(testcase (id ?id) (parent ?name) (state SUCCEEDED) (msg ?msg))
	=>
	(modify ?t (state SUCCEEDED) (msg (str-cat ?id SUCCEEDED)))
)

(defrule simtest-and-success
" A child test failed, fail. "
	?t <- (testcase (type AND) (id ?name) (state PENDING))
	(not (testcase (parent ?name) (state ~SUCCEEDED)))
	=>
	(modify ?t (state SUCCEEDED) (msg (str-cat "all child tests " SUCCEEDED)))
)

(defrule simtest-full-game-success
" Tests of type FULL-GAME succeed once the game is over. "
	?testcase <- (testcase (type FULL-GAME) (state PENDING))
	(wm-fact (key refbox phase) (value POST_GAME))
	=>
	(modify ?testcase (state SUCCEEDED) (msg "Game completed"))
)

(defrule simtest-delivery-success
" Tests of type DELIVERY succeed once a delivery of the specified complexity
  happens. "
	?testcase <- (testcase (type DELIVERY) (state PENDING) (args complexity ?com))
	(wm-fact (key refbox team-color) (value ?team-color&~nil))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key domain fact quantity-delivered args? prod ?ord team ?team-color)
	         (value ?delivered&:(> ?delivered 0)))
	(wm-fact (key domain fact product-complexity args? prod ?ord com ?com))
	=>
	(modify ?testcase (state SUCCEEDED) (msg (str-cat "Delivery of complexity " ?com " done")))
)

(defrule simtest-delivery-count-success
" Tests of type DELIVERY-COUNT succeed once the specified number of deliveries
  happens. "
	(wm-fact (key refbox team-color) (value ?team-color&~nil))
	(wm-fact (key domain fact quantity-delivered args? prod ? team ?team-color)
	         (value ?delivered&:(> ?delivered 0)))
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
" Tests of type POINT-AFTER-MINUTE succeed if the specified number of points
  happen before a given deadline. "
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
" Tests of type POINT-AFTER-MINUTE fail if the deadline is crossed. "
	?testcase <- (testcase (type POINTS-AFTER-MINUTE) (state PENDING)
	             (args minute ?time points ?desired-points))
	(wm-fact (key refbox phase) (value PRODUCTION))
	(wm-fact (key refbox game-time) (values $?gt&:(>= (nth$ 1 ?gt) (* ?time 60))))
	=>
	(modify ?testcase (state FAILED) (msg (str-cat "Insufficient points after " (nth$ 1 ?gt) " seconds")))
)

(defrule simtest-no-broken-mps-failure
" Tests of type NO-BROKEN-MPS fail if a mps state BROKEN is observed. "
	?testcase <- (testcase (type NO-BROKEN-MPS) (state PENDING))
	(wm-fact (key domain fact mps-state args? m ?m s BROKEN))
	=>
	(modify ?testcase (state FAILED) (msg (str-cat "Broken MPS " ?m)))
)

(defrule simtest-enter-field-success
" The ENTER-FIELD goal succeeded; we can move."
	?testcase <- (testcase (type ENTER-FIELD) (state PENDING))
	(goal (class ENTER-FIELD) (mode FINISHED) (outcome COMPLETED))
	=>
	(modify ?testcase (state SUCCEEDED))
)

(defrule simtest-enter-field-failed
" The ENTER-FIELD goal failed; something is broken."
	?testcase <- (testcase (type ENTER-FIELD) (state PENDING))
	(goal (class ENTER-FIELD) (mode FINISHED) (outcome ~COMPLETED))
	=>
	(modify ?testcase (state FAILED))
)

; ============================================================================

(defrule simtest-abort-early
" A test has failed and the testbed will fail as well. Fail all other tests
  as well."
	(testcase (id ?id) (state FAILED) (parent nil))
	(testcase (state ~FAILED))
=>
	(do-for-all-facts ((?testcase testcase))
		(eq ?testcase:state PENDING)
		(modify ?testcase (state FAILED)
		                  (msg (str-cat "Test " ?id " failed already")))
	)
)

(defrule simtest-finished
" All tests terminated, the testbed is done. Evaluate the success status and
  terminate the agent."
	(simtest-initialized)
	(not (testcase (state PENDING)))
	=>
	(printout info "SIMTEST: Finished" crlf)
	(if (do-for-all-facts ((?testcase testcase))
		(and (eq ?testcase:state FAILED) (eq ?testcase:parent nil))
		(printout error ?testcase:type " failed: " ?testcase:msg crlf)
	)
	 then
		(printout info "SIMTEST: End" crlf)
		(simtest-quit FALSE)
	else
		(printout info "All tests successful!" crlf)
		(simtest-quit TRUE)
	)
)
