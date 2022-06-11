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

(defrule action-execute-exogenous-noops
  ?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
                   (action-name ?action&bs-dispense|cs-retrieve-cap
                        |cs-mount-cap
                        |rs-mount-ring1
                        |rs-mount-ring2
                        |rs-mount-ring3
                        |ss-retrieve-c0
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
  (assert
    (wm-fact (key mps-handling prepare prepare-cs ?mps args? m ?mps op MOUNT_CAP))
    (wm-fact (key mps-handling process cs-mount-cap ?mps args? m ?mps wp ?wp capcol ?capcol))
  )
)

(defrule action-execute-request-cs-retrieve-cap
  ?pa <- (plan-action (action-name request-cs-retrieve-cap) (state PENDING) (executable TRUE)
            (param-values ?r ?mps ?cc ?capcol))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
  (assert
    (wm-fact (key mps-handling prepare prepare-cs ?mps args? m ?mps op RETRIEVE_CAP))
    (wm-fact (key mps-handling process cs-retrieve-cap ?mps args? m ?mps cc ?cc capcol ?capcol))
  )
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
  (assert
    (wm-fact (key mps-handling prepare prepare-rs ?rs args? m ?rs rc ?rc r-req ?rs-req))
    (wm-fact (key mps-handling process ?mount-ring-action-name ?rs args?
                        (merge-params ?mount-ring-param-names ?mount-ring-param-values)))
  )
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
  (assert
    (wm-fact (key mps-handling prepare prepare-ds ?ds args? m ?ds ord ?order))
    (wm-fact (key mps-handling process (sym-cat fulfill-order- (lowcase ?complexity)) ?ds args? ?params))
  )
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
                      (action-name ?a-name&wait-for-wp|wait-for-free-side)
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
                      (param-values ? ?m ?side)
          )
  (goal (id ?goal-id) (params wp ?wp $?))
  (wm-fact (key domain fact wp-at args? wp ?wp m ?m side ?side))
  =>
  (printout info "Goal " ?goal-id " finished wait-for-wp" crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-finish-execute-wait-for-free-side-action
  ?pa <- (plan-action (id ?action-id)
                      (plan-id ?plan-id)
                      (goal-id ?goal-id)
                      (action-name wait-for-free-side)
                      (state RUNNING)
                      (executable TRUE))
  ; check if every wait-for-free-side dependency-goal is executed successfully
  (not (and (dependency-assignment (goal-id ?goal-id)
                                   (wait-for FREE-SIDE)
                                   (grounded-with ?dependency-id))
            (goal (id ?dependency-id) (outcome ~COMPLETED))))
  =>
  (printout info "Goal " ?goal-id " finished wait-for-free-side" crlf)
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule action-fail-execute-wait-for-dependencies-action
  ?pa <- (plan-action (id ?action-id)
                      (plan-id ?plan-id)
                      (goal-id ?goal-id)
                      (action-name ?a-name&wait-for-wp|wait-for-free-side)
                      (state RUNNING)
                      (executable TRUE))
  ; check if a dependency-goal failed
  (dependency-assignment (goal-id ?goal-id)
                              (grounded-with ?dependency-id))
  (goal (id ?dependency-id) (outcome FAILED) (mode EVALUATED))
  =>
  (printout warn "Goal " ?goal-id
                       " failed waiting, because dependency-goal "
                       ?dependency-id " failed" crlf)
  (modify ?pa (state EXECUTION-FAILED))
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
