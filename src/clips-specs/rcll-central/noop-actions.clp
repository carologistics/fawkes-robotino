;---------------------------------------------------------------------------
;  noop-actions.clp - Pseudo actions that are not actually executed
;
;  Created: Fri 12 Jan 2018 16:40:59 CET
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
 ?*WAIT-FOR-WP-TIMER* = 30
)

(defrule action-execute-exogenous-noops
  ?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
                   (action-name ?action&bs-dispense|cs-retrieve-cap
                        |cs-mount-cap
                        |rs-mount-ring1
                        |rs-mount-ring2
                        |rs-mount-ring3
                        |ss-retrieve-c0
                        |fulfill-order-discard
                        |fulfill-order-c0
                        |fulfill-order-c1
                        |fulfill-order-c2
                        |fulfill-order-c3
                        |wait-for-mps)
         (executable TRUE)
         (param-values $?param-values))
  =>
  (printout t "Executing " ?action ?param-values crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-execute-noop
  "Direclty finalize the action 'noop'"
  ?pa <- (plan-action  (action-name noop)
                         (plan-id ?plan-id)
                         (id ?id)
                         (state PENDING)
                         (executable TRUE))
  =>
  (printout t "Executing and Finalzing action 'noop' "crlf)
  ;Need to finalize directly since it has no effects to be aplied
  (modify ?pa (state FINAL))
)

(defrule action-execute-wp-spawn
	?pa <- (plan-action (plan-id ?plan-id) (state PENDING) (executable TRUE)
	                    (action-name spawn-wp) (param-values ?wp ?robot))
	=>
	(printout info "Spawning workpiece " ?wp " for robot " ?robot crlf)
	(assert (domain-object (name ?wp) (type workpiece)))
	(modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-execute-cc-spawn
	?pa <- (plan-action (plan-id ?plan-id) (state PENDING) (executable TRUE)
	                    (action-name refill-shelf) (param-values ?mps ?spot ?cc ?color))
	=>
	(printout info "Spawning cap carrier " ?cc " with color " ?color " at " ?mps " " ?spot crlf)
	(assert (domain-object (name ?cc) (type cap-carrier)))
	(modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-execute-ss-store-wp
	?pa <- (plan-action (plan-id ?plan-id) (state PENDING) (executable TRUE)
	                    (action-name ss-store-wp) (param-values ?robot ?m ?wp ?base ?cap))
	=>
	(printout info "Init  " ?m " with " ?wp ": " ?base " " ?cap crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-execute-move-wp-input-output
	?pa <- (plan-action (plan-id ?plan-id) (state PENDING) (executable TRUE)
	                    (action-name move-wp-input-output)
	                    (param-values ?mps ?wp))
	=>
	(printout info "At " ?mps " move  " ?wp " from input to output " crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-start-execute-wait-for-dependencies-action
  ?pa <- (plan-action (id ?action-id)
                      (plan-id ?plan-id)
                      (goal-id ?goal-id)
                      (action-name ?a-name&wait-for-wp)
                      (state PENDING)
                      (executable TRUE))
  =>
  (printout info "Goal " ?goal-id " started " ?a-name crlf)
  (modify ?pa (state RUNNING))
)
(defrule action-finish-execute-wait-for-wp-action
  ?pa <- (plan-action (id ?action-id)
                      (plan-id ?plan-id)
                      (goal-id ?goal-id)
                      (action-name wait-for-wp)
                      (state RUNNING)
                      (executable TRUE)
                      (param-values ? ?m ?side ?wp)
          )
  (wm-fact (key domain fact wp-at args? wp ?wp m ?m side ?side))
  =>
  (printout info "Goal " ?goal-id " finished wait-for-wp" crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

; ROBOCUP 2021 NAVIGATION CHALLENGE
(defrule action-execute-wait-for-reached
  ?pa <- (plan-action (plan-id ?plan-id) (state PENDING) (executable TRUE)
	                    (action-name wait-for-reached) (param-values ?robot ?target))
  =>
  (printout info "WAITING until target" ?target " reached!" crlf)
  (modify ?pa (state RUNNING))
)

; ROBOCUP 2021 NAVIGATION CHALLENGE
(defrule action-stop-execute-wait-for-reached
  (plan (id ?plan-id) (goal-id ?goal-id))
  (goal (id ?goal-id) (params zone ?zone))
  ?pa <- (plan-action (plan-id ?plan-id) (state RUNNING) (executable TRUE)
	                    (action-name wait-for-reached) (param-values ?robot ?target))
  (wm-fact (key domain fact reached args?) (values $?reached&:(member$ ?zone ?reached)))
  =>
  (printout info "Refbox confirmed position " ?target " reached!" crlf)
  (modify ?pa (state FINAL))
)

(defrule action-start-execute-wait-for-mps
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (state PENDING) (executable TRUE)
	                    (action-name wait-for-mps) (param-values ?robot ?wp ?mps INPUT))
  (goal (id ?goal-id) (class MOUNT-CAP|BUFFER-CAP|MOUNT-RING|PAY-FOR-RINGS-WITH-BASE|PAY-FOR-RINGS-WITH-CAP-CARRIER))
  (not (domain-fact (name wp-at) (param-values ?any-wp ?mps OUTPUT)))
	=>
	(printout info "Waiting for an MPS to do an operation " ?wp " for robot " ?robot " to move to the other side" crlf)
	(modify ?pa (state RUNNING))
	(assert (timer (name (sym-cat "wait-for-mps-" ?robot "-" ?wp "-" ?mps))))
)

(defrule action-finish-execute-wait-for-mps-early
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (state PENDING)
	                    (action-name wait-for-mps) (param-values ?robot ?wp ?mps INPUT))
  (goal (id ?goal-id) (class ~MOUNT-CAP&~BUFFER-CAP&~MOUNT-RING&~PAY-FOR-RINGS-WITH-BASE&~PAY-FOR-RINGS-WITH-CAP-CARRIER))
  =>
	(printout info "Not waiting for " ?wp " for robot " ?robot " to move to the other side" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-finish-execute-wait-for-mps-early-processing
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (state PENDING)
	                    (action-name wait-for-mps) (param-values ?robot ?wp ?mps INPUT))
  (domain-fact (name mps-state) (param-values ?mps READY-AT-OUTPUT))
  =>
	(printout info "Not waiting for " ?wp " for robot " ?robot ", MPS already processing" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-finish-execute-wait-for-mps-early-insufficient-ring-count
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (state PENDING|RUNNING)
	                    (action-name wait-for-mps) (param-values ?robot ?wp ?mps INPUT))
  (domain-fact (name mps-type) (param-values ?mps RS))
	(wm-fact (key domain fact rs-ring-spec args? m ?mps r $? rn ?req))
  ?wm2 <- (wm-fact (key domain fact rs-filled-with args? m ?rs n ?rs-num&:(> (sym-to-int ?req) (sym-to-int ?rs-num))))
  =>
	(printout info "Not waiting for " ?wp " for robot " ?robot ", MPS already processing" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
)


(defrule action-finish-execute-wait-for-mps-early-ring-station
	?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (state PENDING|RUNNING)
	                    (action-name wait-for-mps) (param-values ?robot ?wp ?mps INPUT))
  (domain-fact (name mps-type) (param-values ?mps RS))
	(time $?now)
	?wt <- (timer (name ?timer-name) (time $?t&:(timeout ?now ?t 10)))
  (test (eq ?timer-name (sym-cat "wait-for-mps-" ?robot "-" ?wp "-" ?mps)))
  =>
	(printout info "Not waiting for " ?wp " for robot " ?robot ", MPS already processing" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-finish-execute-wait-for-mps
  ?pa <- (plan-action (plan-id ?plan-id) (state RUNNING|RUNNING)
                      (action-name wait-for-mps) (param-values ?robot ?wp ?mps ?side))
	(time $?now)
	?wt <- (timer (name ?timer-name) (time $?t&:(timeout ?now ?t 20)))
  (test (eq ?timer-name (sym-cat "wait-for-mps-" ?robot "-" ?wp "-" ?mps)))
  =>
  (printout info "Finished waiting for " ?mps " with robot " ?robot crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
  (retract ?wt)
)
