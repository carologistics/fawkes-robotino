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
	; All tests have to terminate in order for a testbed to be completed.
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
		(case "VISITED-ALL" then
			(assert (testcase (type VISITED-ALL)))
		)
		(case "VISITED-ONE" then
			(assert (testcase (type VISITED-ONE)))
		)
		(default none)
	)
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


(defrule simtest-visited-all-success
" The ENTER-FIELD goal succeeded; we can move."
	?testcase <- (testcase (type VISITED-ALL) (state PENDING))
	; At least one MPS state was received already
	(wm-fact (key domain fact mps-state args? m ?any-mps $?))
	; Each MPS with a known state was visited
	; (only states of machines from the own team are received)
	(forall
	  (wm-fact (key domain fact mps-state args? m ?mps $?))
	  (wm-fact (key domain fact visited args? m ?mps))
	)
	=>
	(modify ?testcase (state SUCCEEDED))
)

(defrule simtest-visited-one-success
" The ENTER-FIELD goal succeeded; we can move."
	?testcase <- (testcase (type VISITED-ONE) (state PENDING))
	; At least one MPS was visited already
	(wm-fact (key domain fact visited args? m ?mps))
	=>
	(modify ?testcase (state SUCCEEDED))
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
