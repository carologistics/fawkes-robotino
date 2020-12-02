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

(defrule init-wm-sync-flush-locks-during-setup
  (wm-fact (key refbox phase) (value SETUP))
  =>
  (printout warn "Flushing all locks!" crlf)
  (mutex-flush-locks-async)
)

(defrule init-wm-flush-done
  ?t <- (mutex-expire-task (task FLUSH) (state COMPLETED))
  =>
  (printout info "Flushing done" crlf)
  (retract ?t)
)

(defrule init-wm-flush-failed
  ?t <- (mutex-expire-task (task FLUSH) (state FAILED))
  =>
  (printout error "Flushing failed!" crlf)
  (retract ?t)
)
