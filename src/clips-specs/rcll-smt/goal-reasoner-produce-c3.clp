;---------------------------------------------------------------------------
;  goal-reasoner-produce-c3.clp - Produce a C3 complexity product
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

(defrule goal-reasoner-create-produce-c3
	"Create goal PRODUCE-C3"
	(not (goal (id PRODUCE-C3)))
	(not (goal-already-tried PRODUCE-C3))
  ; Ensure C3 order exists
	(wm-fact (key domain fact order-complexity args? ord ?order-id com C3) (value TRUE))
=>
	(printout t "Create goal PRODUCE-C3" crlf)
	(assert (goal (id PRODUCE-C3)))
	(assert (goal-already-tried PRODUCE-C3))
)
