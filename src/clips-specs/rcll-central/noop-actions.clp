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
                        |fulfill-order-c3)
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

(defrule action-execute-request-cs-mount-cap
  ?pa <- (plan-action (action-name request-cs-mount-cap) (state PENDING) (executable TRUE)
            (param-values ?r ?mps ?wp ?capcol))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-execute-request-cs-retrieve-cap
  ?pa <- (plan-action (action-name request-cs-retrieve-cap) (state PENDING) (executable TRUE)
            (param-values ?r ?mps ?cc ?capcol))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-execute-request-rs-mount-ring
  ?pa <- (plan-action (action-name request-rs-mount-ring) (state PENDING) (executable TRUE)
            (param-values ?r ?rs ?wp ?ring-pos ?rc ?rc1 ?rc2 ?rc3 ?rs-req))

  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
  (bind ?mount-ring-action-name (sym-cat rs-mount-ring (sym-to-int ?ring-pos)))
    (switch (sym-to-int ?ring-pos)
      (case 1 then
        (bind ?mount-ring-param-names m wp col r-req)
        (bind ?mount-ring-param-values ?rs ?wp ?rc ?rs-req))
      (case 2 then
        (bind ?mount-ring-param-names m wp col col1 r-req)
        (bind ?mount-ring-param-values ?rs ?wp ?rc ?rc1 ?rs-req))
      (case 3 then
        (bind ?mount-ring-param-names m wp col col1 col2 r-req)
        (bind ?mount-ring-param-values ?rs ?wp ?rc ?rc1 ?rc2 ?rs-req))
     (default
        (printout t "ERROR, plan-action params of request-rs-mount-ring are wrong" crlf)))
)

(defrule action-execute-request-ds-deliver
  ?pa <- (plan-action (action-name request-ds-fulfill-order) (state PENDING) (executable TRUE)
            (param-values ?r ?ds ?wp ?order))
  (wm-fact (key domain fact wp-base-color args? wp ?wp col ?base-color))
  (wm-fact (key domain fact wp-ring1-color args? wp ?wp col ?ring1-color))
  (wm-fact (key domain fact wp-ring2-color args? wp ?wp col ?ring2-color))
  (wm-fact (key domain fact wp-ring3-color args? wp ?wp col ?ring3-color))
  (wm-fact (key domain fact wp-cap-color args? wp ?wp col ?cap-color))
  ;Order-CEs
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key domain fact order-complexity args? ord ?order com ?complexity))
  (wm-fact (key domain fact order-base-color args? ord ?order col ?base-color))
  (wm-fact (key domain fact order-ring1-color args? ord ?order col ?ring1-color))
  (wm-fact (key domain fact order-ring2-color args? ord ?order col ?ring2-color))
  (wm-fact (key domain fact order-ring3-color args? ord ?order col ?ring3-color))
  (wm-fact (key domain fact order-cap-color args? ord ?order col ?cap-color))
  (wm-fact (key domain fact order-gate args? ord ?order gate ?gate))
  =>
 (bind ?params (create$ ord ?order wp ?wp m ?ds g ?gate basecol ?base-color capcol ?cap-color))
 (switch ?complexity
  (case C1 then
      (bind ?params (create$ ord ?order wp ?wp m ?ds g ?gate basecol ?base-color capcol ?cap-color ring1col ?ring1-color)))
  (case C2 then
      (bind ?params (create$ ord ?order wp ?wp m ?ds g ?gate bascol ?base-color capcol ?cap-color ring1col ?ring1-color ring2col ?ring2-color)))
  (case C3 then
      (bind ?params (create$ ord ?order wp ?wp m ?ds g ?gate basecol ?base-color capcol ?cap-color ring1col ?ring1-color ring2col ?ring2-color ring3col ?ring3-color)))
 )
  (modify ?pa (state EXECUTION-SUCCEEDED))
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
  (time $?now)
  =>
  (printout info "Goal " ?goal-id " started " ?a-name crlf)
  (modify ?pa (state RUNNING))
	(assert (action-timer (plan-id ?plan-id)
	            (action-id ?action-id)
	            (timeout-duration ?*WAIT-FOR-WP-TIMER*)
	            (status RUNNING)
	            (start-time ?now)))
)

(defrule action-abort-execute-wait-for-wp-action
  ?pa <- (plan-action (id ?action-id)
                      (plan-id ?plan-id)
                      (goal-id ?goal-id)
                      (action-name wait-for-wp)
                      (state RUNNING)
                      (executable TRUE)
                      (param-values ? ?m ?side)
          )
  (time $?now)
	?pt <- (action-timer (plan-id ?plan-id) (status RUNNING)
	                     (action-id ?action-id)
	                     (start-time $?st)
	                     (timeout-duration ?timeout&:(timeout ?now ?st ?timeout)))
  =>
  (printout info "Goal " ?goal-id " failed wait-for-wp after timeout" crlf)
  (retract ?pt)
  (modify ?pa (state FAILED))
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
