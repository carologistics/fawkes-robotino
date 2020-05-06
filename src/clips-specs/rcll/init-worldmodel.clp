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
  (wm-fact (key config rcll robot-name) (value ?name))
  =>
  (printout info "Setting /cx/identity to " ?name crlf)
  (assert (wm-fact (key cx identity) (value ?name)))
)

(defrule init-worldmodel-sync
  (executive-init)
  (wm-fact (key cx identity))
  =>
  (wm-robmem-sync-enable "/domain/objects-by-type/workpiece")
  (wm-robmem-sync-enable "/domain/objects-by-type/cap-carrier")
  (wm-robmem-sync-enable "/domain/fact/bs-prepared-for")
  (wm-robmem-sync-enable "/domain/fact/bs-prepared-side")
  (wm-robmem-sync-enable "/domain/fact/cs-can-perform")
  (wm-robmem-sync-enable "/domain/fact/cs-prepared-for")
  (wm-robmem-sync-enable "/domain/fact/cs-buffered")
  (wm-robmem-sync-enable "/domain/fact/rs-prepared-color")
  (wm-robmem-sync-enable "/domain/fact/rs-filled-with")
  (wm-robmem-sync-enable "/domain/fact/ds-prepared-gate")
  (wm-robmem-sync-enable "/domain/fact/ss-stored-wp")
  (wm-robmem-sync-enable "/domain/fact/ss-initialized")
  (wm-robmem-sync-enable "/domain/fact/ss-prepared-for")
  (wm-robmem-sync-enable "/domain/fact/mps-side-free")
  (wm-robmem-sync-enable "/domain/fact/wp-unused")
  (wm-robmem-sync-enable "/domain/fact/wp-usable")
  (wm-robmem-sync-enable "/domain/fact/wp-at")
  (wm-robmem-sync-enable "/domain/fact/wp-base-color")
  (wm-robmem-sync-enable "/domain/fact/wp-ring1-color")
  (wm-robmem-sync-enable "/domain/fact/wp-ring2-color")
  (wm-robmem-sync-enable "/domain/fact/wp-ring3-color")
  (wm-robmem-sync-enable "/domain/fact/wp-cap-color")
  (wm-robmem-sync-enable "/domain/fact/wp-on-shelf")
  (wm-robmem-sync-enable "/domain/fact/wp-spawned-for")
  (wm-robmem-sync-enable "/domain/fact/spot-free")
  (wm-robmem-sync-enable "/domain/fact/quantity-delivered")
  (wm-robmem-sync-enable "/mps-handling/")
  (wm-robmem-sync-enable "/order/meta/wp-for-order")
  (wm-robmem-sync-enable "/strategy/keep-mps-side-free")
  (wm-robmem-sync-enable "/exploration/fact/tag-vis")
  (wm-robmem-sync-enable "/exploration/fact/line-vis")
  (wm-robmem-sync-enable "/exploration/fact/time-searched")
)
