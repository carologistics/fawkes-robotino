;---------------------------------------------------------------------------
;  init.clp - Initialize blackboard interfaces
;
;  Created: Wed 23 Dec 2020 18:46:42 CET
;  Copyright  2020  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

(defrule blackboard-init-open-skiller
  (executive-init)
  (ff-feature-loaded blackboard)
  (ff-feature-loaded skills)
 =>
  (blackboard-open "SkillerInterface" "skiller")
  (assert (skiller-control (skiller "Skiller")))
)
