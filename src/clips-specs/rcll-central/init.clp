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
  (wm-fact (key refbox comm private-peer-enabled) (type BOOL) (value TRUE) )
  ?i <- (executive-init)
  (domain-wm-flushed)
  =>
  (printout t "Finished initialization" crlf)
  (retract ?i)
)

(defrule initialization-abort-on-finalize
  "Abort initialization if we are in finalize"
  (declare (salience 500))
  (executive-finalize)
  ?i <- (executive-init)
  =>
  (retract ?i)
)

(defrule init-worldmodel-set-identity
  (wm-fact (key config agent name) (value ?name))
  =>
  (printout info "Setting /cx/identity to " ?name crlf)
  (assert (wm-fact (key cx identity) (value ?name)))
)

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
