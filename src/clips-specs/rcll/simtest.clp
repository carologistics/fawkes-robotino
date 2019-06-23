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

(deftemplate testcase
	(slot name (type SYMBOL))
	(slot state (type SYMBOL) (allowed-values SUCCEEDED FAILED PENDING) (default PENDING))
	(slot msg (type STRING))
)

(deffunction simtest-quit (?success)
	(printout info "Exiting (success: " ?success ") ..." crlf)
	(quit)
)

(defrule simtest-initialize
	(not (simtest-initialized))
	(wm-fact (key config simtest enabled) (value TRUE))
	=>
	(assert (testcase (name POINTS-AFTER-ONE-MINUTE)))
	(assert (simtest-initialized))
)

(defrule simtest-points-after-one-minute-success
	?testcase <- (testcase (name POINTS-AFTER-ONE-MINUTE) (state PENDING))
	(wm-fact (key refbox team-color) (value ?team-color&~nil))
	(wm-fact (key refbox game-time) (values $?gt&:(< (nth$ 1 ?gt) 60)))
	(wm-fact (key refbox points ?lc-team-color&:(eq ?lc-team-color (lowcase ?team-color)))
	         (value ?points&:(> ?points 0)))
	=>
	(modify ?testcase (state SUCCEEDED) (msg (str-cat "Scored " ?points " points")))
)

(defrule simtest-no-points-after-one-minute
	?testcase <- (testcase (name POINTS-AFTER-ONE-MINUTE) (state PENDING))
	(wm-fact (key refbox game-time) (values $?gt&:(>= (nth$ 1 ?gt) 60)))
	=>
	(modify ?testcase (state FAILED) (msg (str-cat "No points after " (nth$ 1 ?gt) " seconds")))
)

(defrule simtest-finished
	(simtest-initialized)
	(not (testcase (state PENDING)))
	=>
	(if (do-for-all-facts ((?testcase testcase)) (eq ?testcase:state FAILED)
		(printout error ?testcase:name " failed: " ?testcase:msg crlf)
	)
	 then
		(simtest-quit false)
	else
		(printout info "All tests successful!" crlf)
		(simtest-quit true)
	)
)
