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
  "We're done initializing if we have a team color."
  (wm-fact (key refbox team-color) (value ~NONE))
  (wm-fact (key refbox comm private-peer-enabled) (value TRUE) )
  ?i <- (executive-init)
  =>
  (retract ?i)
)
