;---------------------------------------------------------------------------
;  wait-actions.clp - Execution of wait actions
;
;  Created: Fri 15 Jun 2018 22:13:22 CEST
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

(defglobal
 ?*WAIT-DURATION* = 5
)

(defrule action-start-execute-wait-action
  ?pa <- (plan-action (id ?action-id) (plan-id ?plan-id) (goal-id ?goal-id)
                      (action-name wait) (state PENDING) (executable TRUE))
  =>
  (printout info "Starting to wait" crlf)
  (assert (timer (name (sym-cat ?goal-id - ?plan-id - ?action-id))))
  (modify ?pa (state RUNNING))
)

(defrule action-finish-execute-wait-action
  ?pa <- (plan-action (id ?action-id) (plan-id ?plan-id) (goal-id ?goal-id)
                      (action-name wait) (state RUNNING) (executable TRUE))
  (time $?now)
  ?timer <- (timer (name ?name &: (sym-cat ?goal-id - ?plan-id - ?action-id))
                   (time $?t&:(timeout ?now ?t ?*WAIT-DURATION*)))
  =>
  (printout info "Finished waiting" crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
  (retract ?timer)
)

(defrule action-fail-execute-wait-action
  ?pa <- (plan-action (id ?action-id) (plan-id ?plan-id) (goal-id ?goal-id)
                      (action-name wait) (state RUNNING) (executable TRUE))
  (not (timer (name ?name &: (sym-cat ?goal-id - ?plan-id - ?action-id))))
  =>
  (printout warn "Failed to wait, timer for action id " ?action-id
                 ", plan-id " ?plan-id ", goal-id " ?goal-id " does not exist"
                 crlf)
  (modify ?pa (state EXECUTION-FAILED))
)
