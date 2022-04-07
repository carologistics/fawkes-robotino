;---------------------------------------------------------------------------
;  init-worldmodel.clp - Initialize the world model
;
;  Created: Fri 12 Jan 2018 15:01:59 CET
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

(defrule init-worldmodel-set-identity
  (wm-fact (key config xenonite agent name) (value ?name))
  =>
  (printout info "Setting /cx/identity to " ?name crlf)
  (assert (wm-fact (key cx identity) (value ?name)))
  (assert (domain-fact (name self) (param-values (sym-cat ?name))))
)

(defrule init-worldmodel-sync
  (executive-init)
  (wm-fact (key cx identity))
  (wm-robmem-sync-initialized)
  =>
  ;(wm-robmem-sync-enable "/domain/objects-by-type/workpiece")
  ;(wm-robmem-sync-enable "/domain/objects-by-type/cap-carrier")
  (wm-robmem-sync-enable "/domain/objects-by-type/robot")
  (wm-robmem-sync-enable "/domain/objects-by-type/container")
  (wm-robmem-sync-enable "/robot-ready")
  (wm-robmem-sync-enable "/domain/initialization")
  (wm-robmem-sync-enable "/domain/fact/robot-at")
  ;(wm-robmem-sync-enable "/domain/fact/robot-carries")
  ;(wm-robmem-sync-enable "/domain/fact/robot-can-carry")
  (wm-robmem-sync-enable "/domain/fact/container-at")
  (wm-robmem-sync-enable "/domain/fact/container-filled")
  (wm-robmem-sync-enable "/domain/fact/container-can-be-filled")
  (wm-robmem-sync-enable "/domain/fact/machine-in-state")
  ;(wm-robmem-sync-enable "/domain/fact/machine-for-material")
  ;(wm-robmem-sync-enable "/domain/fact/machine-makes-material")
  (wm-robmem-sync-enable "/domain/fact/location-is-free")
  (wm-robmem-sync-enable "/domain/fact/storage-is-full")
  (wm-robmem-sync-enable "/domain/promise")
  (wm-robmem-sync-restore)
)
