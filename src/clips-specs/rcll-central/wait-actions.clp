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
 ?*WAIT-DURATION* = 6
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

(defrule action-start-execute-wait-for-dependencies-action
  ?pa <- (plan-action (id ?action-id) (plan-id ?plan-id) (goal-id ?goal-id)
                      (action-name wait-for-dependencies) (state PENDING) (executable TRUE))
  =>
  (printout info "Starting to wait for dependencies" crlf)
  (modify ?pa (state RUNNING))
)

(defrule action-finish-execute-wait-for-dependencies-action
  ?pa <- (plan-action (id ?action-id) (plan-id ?plan-id) (goal-id ?goal-id)
                      (action-name wait-for-dependencies) (state RUNNING) (executable TRUE))
  ; check if every dependency-goal is executed successfully
  (not (and (dependency-assignment (goal-id ?goal-id) (grounded-with ?dependency-id)) 
            (goal (id ?dependency-id) (outcome ~COMPLETED))))
  =>
  (printout info "Finished waiting for dependencies" crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-fail-execute-wait-for-dependencies-action
  ?pa <- (plan-action (id ?action-id) (plan-id ?plan-id) (goal-id ?goal-id)
                      (action-name wait-for-dependencies) (state RUNNING) (executable TRUE))
  ; check if a dependency-goal failed
  (and (dependency-assignment (goal-id ?goal-id) (grounded-with ?dependency-id)) 
       (goal (id ?dependency-id) (outcome FAILED)))
  =>
  (printout warn "Goal " ?goal-id " failed to wait, because the dependency goal " ?dependency-id " failed" crlf)
  (modify ?pa (state EXECUTION-FAILED))
)
