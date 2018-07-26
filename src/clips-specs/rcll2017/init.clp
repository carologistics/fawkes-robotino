;---------------------------------------------------------------------------
;  init.clp - Initialize RCLL
;
;  Created: Thu 11 Jan 2018 16:08:21 CET
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

(defrule initialization-done
  "Finish Initialization"
  (wm-fact (key refbox comm private-peer-enabled) (value TRUE) )
  ?i <- (executive-init)
	(wm-fact (key config rcll robot-name) (value ?robot-name))
  =>
  (printout t "Finished initialization" crlf)
  ; (retract ?i)

	(cx-identity-set ?robot-name)
	(wm-robmem-sync-enable
	"/plan-action/COMPLEXITY"
	)
	(wm-robmem-sync-enable
	"/plan-action/COMPLEXITY2"
	)
	; (wm-robmem-sync-enable
	; "/r-1-at"
	; )
	(wm-robmem-sync-enable
	"/r-2-at"
	)
	(wm-robmem-sync-enable
	"/r-3-at"
	)
	(wm-robmem-sync-enable
	"/domain/fact/location-free"
	)
	(wm-robmem-sync-enable
	"/domain/fact/bs-prepared-color"
	)
	(wm-robmem-sync-enable
	"/domain/fact/bs-prepared-side"
	)
	(wm-robmem-sync-enable
	"/domain/fact/cs-can-perform"
	)
	(wm-robmem-sync-enable
	"/domain/fact/cs-buffered"
	)
	(wm-robmem-sync-enable
	"/domain/fact/cs-free"
	)
	(wm-robmem-sync-enable
	"/domain/fact/rs-prepared-color"
	)
	(wm-robmem-sync-enable
	"/domain/fact/rs-filled-with"
	)
	(wm-robmem-sync-enable
	"/domain/fact/ds-prepared-gate"
	)
	(wm-robmem-sync-enable
	"/domain/fact/order-fulfilled"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-unused"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-usable"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-at"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-base-color"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-ring1-color"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-ring2-color"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-ring3-color"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-cap-color"
	)
	(wm-robmem-sync-enable
	"/domain/fact/wp-on-shelf"
	)
	(wm-robmem-sync-enable
	"/domain/fact/spot-free"
	)
)

; (defrule initialization-abort-on-finalize
;   "Abort initialization if we are in finalize"
;   (declare (salience 500))
;   (executive-finalize)
;   ?i <- (executive-init)
;   =>
;   (retract ?i)
; )
