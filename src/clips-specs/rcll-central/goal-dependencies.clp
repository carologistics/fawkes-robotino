;---------------------------------------------------------------------------
;  goal-dependencies.clp - Defines dependency-assignments and grounds
;                          them for executability
;
;  Created: Sat 22 May 2021 13:07:31 CET
;  Copyright  2021  Matteo Tschesche <matteo.tschesche@rwth-aachen.de>
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

(deftemplate dependency-assignment
	; id of goal that might rely on another goal/dependence-goal
	(slot goal-id (type SYMBOL))

	; goal class of dependency-goal
	; in case of CLEAR-OUTPUT: DELIVER or DISCARD
	(slot class (type SYMBOL))

	; defines if goal waits for dependency before wp-get (wait-for-wp),
	; before wp-put (wait-for-free-side), or not at all (nil)
	; set when asserting
	(slot wait-for (type SYMBOL) (allowed-values nil WP FREE-SIDE))

	; necessary parameters used for goal-expander of dependency-goal,
	; set in execution-check
	; for deliver-mount-cap:    wp, wp-loc, wp-side
	; for mount-cap-mount-ring: wp, wp-loc, wp-side
	; for discard-buffer-cap:   wp, wp-loc, wp-side
	(multislot params (type SYMBOL))

	; id of dependence-goal, nil if ungrounded
	(slot grounded-with (type SYMBOL))

	;(slot priority (type float) (default 0.0))
)

; ---------------------------- Create Dependencies ----------------------------
; A goal depends on a class of a dependency-goal if such a dependency-goal can
; be required for executing this goal
