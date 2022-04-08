;---------------------------------------------------------------------------
;  goal-expansion-pddl.clp - Expand goals with PDDL
;
;  Created: Tue Dec 13 2021
;  Copyright  2021 Till Hofmann <hofmann@kbsg.rwth-aachen.de
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


(defrule goal-expander-get-container-and-fill
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class FILL-CONTAINER) (mode SELECTED)
              (params robot ?r container ?c start ?start mine ?mine) (parent ?parent-id))
  =>
  (printout info "Expanding " ?goal-id crlf)
  (pddl-call ?goal-id (str-cat "(and (robot-at " ?r " " ?mine ") "
                                    "(robot-carries " ?r " " ?c ") "
                                    "(container-filled " ?c " REGOLITH))"))
)


(defrule goal-expander-deliver-to-machine
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class DELIVER) (mode SELECTED)
              (params robot ?r side ?side machine ?machine container ?c material ?mat) (parent ?parent-id))
  (domain-fact (name robot-at) (param-values ?r ?start))
  =>
  (printout info "Expanding " ?goal-id crlf)
  (pddl-call ?goal-id (str-cat "(and (robot-at " ?r " " BASE ") "
                                    "(machine-in-state " ?machine " FILLED)"
                                    "(container-can-be-filled " ?c "))"))
)


(defrule goal-expander-start-machine
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class START-MACHINE) (mode SELECTED)
              (params robot ? side ? machine ?m) (parent ?parent-id))
  =>
  ;(pddl-call ?goal-id (str-cat "(machine-in-state " ?m " OPERATING)"))

  (printout info "Expanding " ?goal-id crlf)
  (bind ?plan-id (sym-cat START-MACHINE-PLAN (gensym*)))
  (assert
    (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL))
    (plan-action (id 1) (plan-id ?plan-id) (goal-id ?goal-id)
                 (action-name start-machine) (param-values ?m))
  )
  (modify ?g (mode EXPANDED) (committed-to ?plan-id))
)


(defrule goal-expander-clean-machine
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class CLEAN-MACHINE) (mode SELECTED)
              (params robot ?r side ?s machine ?m container ?c material ?mat) (parent ?parent-id))
  (domain-fact (name robot-at) (param-values ?r ?start))
  =>
  (printout info "Expanding " ?goal-id crlf)
  (pddl-call ?goal-id (str-cat "(and (robot-carries " ?r " " ?c ") "
                                    "(container-filled " ?c " " ?mat ") "
                                    "(robot-at " ?r " BASE)"
                               ")"))
)

(defrule goal-expander-deliver-xenonite
  ?p <- (goal (mode DISPATCHED) (id ?parent-id))
  ?g <- (goal (id ?goal-id) (class DELIVER-XENONITE) (mode SELECTED)
              (params robot ?r container ?c) (parent ?parent-id))
  (domain-fact (name robot-at) (param-values ?r ?start))
  =>
  (printout info "Expanding " ?goal-id crlf)
  (pddl-call ?goal-id (str-cat "(and (container-at " ?c " STORAGE-INPUT) "
                                    "(robot-at " ?r " BASE) "
                               ")"))
)
