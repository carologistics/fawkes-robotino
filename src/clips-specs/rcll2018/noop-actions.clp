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
