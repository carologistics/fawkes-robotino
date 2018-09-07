;---------------------------------------------------------------------------
;  goal-reasoner-produce-c0.clp - Produce a C0 complexity product
;  - Produce a C0 complexity after the C3 has been done
;  - Work on the order O1 with C0 complexity as it has no delivery bounds
;  - Do not try this goal again
;
;  Created: Fri Sep 7 13:58
;  Copyright  2018 Igor Bongartz <bongartz@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
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

(defrule goal-reasoner-create-produce-c0
	"Create goal PRODUCE-C0"
	(not (goal (id PRODUCE-C0)))
	(not (goal-already-tried PRODUCE-C0))
  ; Ensure C0 order exists
	(wm-fact (key domain fact order-complexity args? ord ?order-id&O1 com C0) (value TRUE))
  ; and that PRODUCE-C3 plans of robots R-1 and R-2 are finished
  (wm-fact (key plan-action PRODUCE-C3 ?plan-id done-r1))
  (wm-fact (key plan-action PRODUCE-C3 ?plan-id done-r2))
=>
	(printout t "Create goal PRODUCE-C0" crlf)
	(assert (goal (id PRODUCE-C0)))
	(assert (goal-already-tried PRODUCE-C0))
)
