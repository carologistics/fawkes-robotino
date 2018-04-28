;---------------------------------------------------------------------------
;  init-wm-sync.clp - Initialize world model synchronization
;
;  Created: Tue 24 Apr 2018 19:50:36 CEST
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

;(defrule init-wm-sync-flush-facts
;  (executive-init)
;  =>
;  (robot-memory-sync-clean-domain-facts)
;)

(deffunction wm-sync-flush-locks-of-agent
  (?owner)
  (printout warn "Clearing all locks of " ?owner crlf)
  (bind ?doc (bson-create))
  (bson-append ?doc "locked-by" ?owner)
  (robmem-remove ?*MUTEX-COLLECTION* ?doc)
)

(deffunction wm-sync-flush-all-locks
  ()
  (printout warn "Clearing all locks of all agents" crlf)
  (bind ?doc (bson-create))
  (robmem-remove ?*MUTEX-COLLECTION* ?doc)
)

(defrule init-wm-sync-flush-locks
  (executive-init)
  (wm-fact (id "/cx/identity") (value ?self))
  =>
  (wm-sync-flush-locks-of-agent ?self)
)

(defrule init-wm-sync-flush-locks-during-setup
  (wm-fact (key refbox phase) (value SETUP))
  =>
  (wm-sync-flush-all-locks)
)
