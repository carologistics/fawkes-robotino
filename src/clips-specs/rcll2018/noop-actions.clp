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

