;---------------------------------------------------------------------------
;  goal-classes.clp - Define the goal classes and their preconditions
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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
  ; Set it to a negative value so the promise never times out.
  ?*PROMISES-MAX-FUTURE* = -1
)

(deffunction compute-required-resources (?required-resources ?promised-from-goals ?disable-delay)
" Given a list of resources, the ones that are requested upon goal committment
  are returned.
  Goals that are not based on a promise request each resource '<res>' along
  with a duplicate 'PROMISE-<res>', the latter is freed directly after being
  acquired.
  Goals that are acting based on promises from goals will not be able to lock
  a resource '<res>' that is also acquired by a goal that emitted the promise.
  In that case, only the 'PROMISE-<res>' resource is requested upon goal
  commitment and is traded for the actual '<res>' during execution.

  @param: ?required-resources list of resources
  @param: ?promised-from-goals list of '<goal-id>@<agent>' entries of goals
          that emitted promises relevant for the required resources that are
          being computed
  @param: ?disable-delay if TRUE, then the content of ?promised-from-goals
          has no effect on the result
  @return: list of required resources and PROMISE-resources
"
  (bind ?delayed-resources (create$))
  (progn$ (?goal-agent ?promised-from-goals)
     (bind ?str-goal-agent (str-cat ?goal-agent))
     (bind ?sep (str-index "@" ?str-goal-agent))
     (bind ?promising-goal (sym-cat (sub-string 1 (- ?sep 1) ?str-goal-agent)))
     (bind ?promising-agent (sym-cat (sub-string (+ ?sep 1) (length$ ?str-goal-agent) ?str-goal-agent)))
     (do-for-fact ((?resource-promise domain-promise))
                  (and (eq ?resource-promise:name RESOURCES)
                       (eq ?resource-promise:promising-goal ?promising-goal)
                       (eq ?resource-promise:promising-agent ?promising-agent)
                  )
                  (bind ?delayed-resources (append$ ?delayed-resources ?resource-promise:param-values))
     )
  )
  (bind ?final-resources (create$))
  (progn$ (?res ?required-resources)
    (bind ?final-resources (append$ ?final-resources (sym-cat PROMISE- ?res)))
    (if (or (eq ?disable-delay TRUE) (not (member$ ?res ?delayed-resources))) then
        (bind ?final-resources (append$ ?final-resources ?res))
    )
  )
  (return ?final-resources)
)

; ------------------------- ASSERT GOAL CLASSES -----------------------------------


(defrule goal-class-create-get-container-and-fill
    (domain-constant (type location) (value ?mine))
    (domain-fact (name location-is-mine) (param-values ?mine))
    =>
    (assert
        (goal-class (class FILL-CONTAINER)
                    (id (sym-cat FILL-CONTAINER - ?mine))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta mine ?mine)
                    (param-names     r     base     mine     c)
                    (param-constants nil   BASE     ?mine    nil)
                    (param-types     robot location location container)
                    (param-quantified)
                    (lookahead-time 30)
                    (preconditions "
                        (and
                            (robot-at ?r ?base)
                            (robot-can-carry ?r)
                            (container-at ?c CONTAINER-DEPOT)
                            (location-is-free ?mine)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-deliver-to-machine
    (domain-constant (type machine) (value ?machine))
    (domain-constant (type location) (value ?side))
    (domain-fact (name location-part-of-machine) (param-values ?side ?machine))
    (domain-fact (name location-is-machine-input) (param-values ?side))
    =>
    (assert
        (goal-class (class DELIVER)
                    (id (sym-cat DELIVER - ?machine))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta machine ?machine)
                    (param-names     r     side     machine  c         material)
                    (param-constants nil   ?side    ?machine nil       nil)
                    (param-types     robot location machine  container material)
                    (param-quantified)
                    (lookahead-time 10)
                    (preconditions "
                        (and
                            (robot-carries ?r ?c)
                            (container-filled ?c ?material)
                            (location-is-free ?side)
                            (location-is-machine-input ?side)
                            (location-part-of-machine ?side ?machine)
                            (machine-in-state ?machine IDLE)
                            (machine-for-material ?machine ?material)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-start-machine
    (domain-constant (type machine) (value ?machine))
    =>
    (assert
        (goal-class (class START-MACHINE)
                    (id (sym-cat START-MACHINE - ?machine))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta machine ?machine)
                    (param-names     r     side     machine)
                    (param-constants nil   nil      ?machine)
                    (param-types     robot location machine)
                    (param-quantified)
                    (lookahead-time 0)
                    (preconditions "
                        (and
                            (location-is-machine-input ?side)
                            (location-part-of-machine ?side ?machine)
                            (machine-in-state ?machine FILLED)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-clean-machine
    (domain-constant (type machine) (value ?machine))
    (domain-constant (type location) (value ?side))
    (domain-fact (name location-part-of-machine) (param-values ?side ?machine))
    (domain-fact (name location-is-machine-output) (param-values ?side))
    =>
    (assert
        (goal-class (class CLEAN-MACHINE)
                    (id (sym-cat CLEAN-MACHINE - ?machine))
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (meta machine ?machine)
                    (param-names     r     side     machine  c         mat)
                    (param-constants nil   ?side    ?machine nil       nil)
                    (param-types     robot location machine  container material)
                    (param-quantified)
                    (lookahead-time 20)
                    (preconditions "
                        (and
                            (robot-carries ?r ?c)
                            (container-can-be-filled ?c)
                            (location-is-free ?side)
                            (location-is-machine-output ?side)
                            (location-part-of-machine ?side ?machine)
                            (machine-in-state ?machine READY)
                            (machine-makes-material ?machine ?mat)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)

(defrule goal-class-create-deliver-xenonite
    =>
    (assert
        (goal-class (class DELIVER-XENONITE)
                    (id DELIVER-XENONITE)
                    (type ACHIEVE)
                    (sub-type SIMPLE)
                    (param-names     r     c)
                    (param-constants nil   nil)
                    (param-types     robot container)
                    (param-quantified)
                    (lookahead-time 20)
                    (preconditions "
                        (and
                            (robot-carries ?r ?c)
                            (container-filled ?c XENONITE)
                            (location-is-free STORAGE-INPUT)
                            (not (storage-is-full))
                        )
                    ")
                    (effects "")
        )
    )
)
; ------------------------- ASSERT GOALS -----------------------------------

(defrule goal-class-assert-get-container-and-fill
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (domain-fact (name self) (param-values ?r))
    (goal (class PRODUCTION-TRY-ALL) (id ?parent) (meta $? host ?r) (mode SELECTED))
    (goal-class (class ?class&FILL-CONTAINER) (id ?cid) (sub-type ?subtype) (lookahead-time ?lt))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied ?sat) (grounding ?grounding-id)
      (promised-from ?from) (promised-from-goals $?promised-from-goals))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?base ?mine ?c))
    (promise-time (usecs ?now))
    (test (sat-or-promised ?sat ?now ?from ?lt))
    =>
    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (printout t "Goal " ?goal-id " (class " ?class ") formulated from PDDL" crlf)
    (if (neq ?sat TRUE) then
        (printout t "Debug: Goal formulated from promise" crlf)
    )
    (bind ?resources (create$ ?c))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            container ?c
                            start ?base
                            mine ?mine
                    )
                    (required-resources
                      (compute-required-resources ?resources ?promised-from-goals ?sat))
    ))

    ;assert promises resulting from the plan-action of this goal
    (assert
        (domain-promise (name location-is-free) (param-values ?mine) (promising-goal ?goal-id) (valid-at (+ 22 ?now)) (negated TRUE))
        (domain-promise (name RESOURCES) (param-values ?resources) (promising-goal ?goal-id) (valid-at ?*PROMISES-MAX-FUTURE*) (negated FALSE))
    )
)

(defrule goal-class-assert-deliver-to-machine
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (domain-fact (name self) (param-values ?r))
    (goal (class PRODUCTION-TRY-ALL) (id ?parent) (meta $? host ?r) (mode SELECTED))
    (goal-class (class ?class&DELIVER) (id ?cid) (sub-type ?subtype) (lookahead-time ?lt))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied ?sat) (grounding ?grounding-id)
      (promised-from ?from) (promised-from-goals $?promised-from-goals))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?side ?machine ?c ?mat))
    (domain-fact (name robot-at) (param-values ?r ?start))

    (promise-time (usecs ?now))
    (test (sat-or-promised ?sat ?now ?from ?lt))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)
    (if (neq ?sat TRUE) then
        (printout t "Debug: Goal formulated from promise" crlf)
    )

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (bind ?resources (create$ ?machine ?side ?c))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            side ?side
                            machine ?machine
                            container ?c
                            material ?mat
                    )
                    (required-resources
                      (compute-required-resources ?resources ?promised-from-goals ?sat))
    ))

    ;assert promises resulting from the plan-action of this goal
    (assert
        (domain-promise (name location-is-free) (param-values ?start) (promising-goal ?goal-id) (valid-at (+ 10 ?now)) (negated FALSE))
        (domain-promise (name machine-in-state) (param-values ?machine FILLED) (promising-goal ?goal-id) (valid-at (+ 12 ?now)) (negated FALSE))
        (domain-promise (name machine-in-state) (param-values ?machine IDLE) (promising-goal ?goal-id) (valid-at (+ 12 ?now)) (negated TRUE))
        (domain-promise (name RESOURCES) (param-values ?resources) (promising-goal ?goal-id) (valid-at ?*PROMISES-MAX-FUTURE*) (negated FALSE))
    )
)

(defrule goal-class-assert-start-machine
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (domain-fact (name self) (param-values ?r))
    (goal (class PRODUCTION-TRY-ALL) (id ?parent) (meta $? host ?r) (mode SELECTED))
    (goal-class (class ?class&START-MACHINE) (id ?cid) (sub-type ?subtype) (lookahead-time ?lt))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied ?sat) (grounding ?grounding-id)
      (promised-from ?from) (promised-from-goals $?promised-from-goals))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?side ?machine))
    (promise-time (usecs ?now))
    (test (sat-or-promised ?sat ?now ?from ?lt))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)
    (if (neq ?sat TRUE) then
        (printout t "Debug: Goal formulated from promise" crlf)
    )

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (bind ?resources (create$ ?machine))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            side ?side
                            machine ?machine
                    )
                    (required-resources
                      (compute-required-resources ?resources ?promised-from-goals ?sat))
    ))

    ;assert promises resulting from the plan-action of this goal
    (assert
        (domain-promise (name machine-in-state) (param-values ?machine READY) (promising-goal ?goal-id) (valid-at (+ 20 ?now)) (negated FALSE) (do-not-invalidate TRUE))
        (domain-promise (name machine-in-state) (param-values ?machine FILLED) (promising-goal ?goal-id) (valid-at (+ 5 ?now)) (negated TRUE) (do-not-invalidate TRUE))
        (domain-promise (name machine-in-state) (param-values ?machine OPERATING) (promising-goal ?goal-id) (valid-at (+ 5 ?now)) (negated TRUE) (do-not-invalidate TRUE))
        (domain-promise (name RESOURCES) (param-values ?resources) (promising-goal ?goal-id) (valid-at ?*PROMISES-MAX-FUTURE*) (negated FALSE))
    )
)

(defrule goal-class-assert-clean-machine
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (domain-fact (name self) (param-values ?r))
    (goal (class PRODUCTION-TRY-ALL) (id ?parent) (meta $? host ?r) (mode SELECTED))
    (goal-class (class ?class&CLEAN-MACHINE) (id ?cid) (sub-type ?subtype) (lookahead-time ?lt))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied ?sat) (grounding ?grounding-id)
      (promised-from ?from) (promised-from-goals $?promised-from-goals))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?side ?machine ?c ?mat))
    (domain-fact (name robot-at) (param-values ?r ?start))

    (promise-time (usecs ?now))
    (test (sat-or-promised ?sat ?now ?from ?lt))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)
    (if (neq ?sat TRUE) then
        (printout t "Debug: Goal formulated from promise" crlf)
    )

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (bind ?resources (create$ ?side ?c))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            side ?side
                            machine ?machine
                            container ?c
                            material ?mat
                    )
                    (required-resources
                      (compute-required-resources ?resources ?promised-from-goals ?sat))
    ))

    ;assert promises resulting from the plan-action of this goal
    (assert
        (domain-promise (name location-is-free) (param-values ?start) (promising-goal ?goal-id) (valid-at (+ 10 ?now)) (negated FALSE))
        (domain-promise (name machine-in-state) (param-values ?machine READY) (promising-goal ?goal-id) (valid-at (+ 12 ?now)) (negated TRUE))
        (domain-promise (name machine-in-state) (param-values ?machine IDLE) (promising-goal ?goal-id) (valid-at (+ 12 ?now)) (negated FALSE))
        (domain-promise (name RESOURCES) (param-values ?resources) (promising-goal ?goal-id) (valid-at ?*PROMISES-MAX-FUTURE*) (negated FALSE))
    )
)

(defrule goal-class-assert-deliver-xenonite
    (declare (salience ?*SALIENCE-GOAL-FORMULATE*))
    (domain-fact (name self) (param-values ?r))
    (goal (class PRODUCTION-TRY-ALL) (id ?parent) (meta $? host ?r) (mode SELECTED))
    (goal-class (class ?class&DELIVER-XENONITE) (id ?cid) (sub-type ?subtype) (lookahead-time ?lt))
    (pddl-formula (part-of ?cid) (id ?formula-id))
    (grounded-pddl-formula (formula-id ?formula-id) (is-satisfied ?sat) (grounding ?grounding-id)
      (promised-from ?from) (promised-from-goals $?promised-from-goals))
    (pddl-grounding (id ?grounding-id) (param-values ?r ?c))
    (promise-time (usecs ?now))
    (test (sat-or-promised ?sat ?now ?from ?lt))
    =>
    (printout t "Goal " ?class " formulated from PDDL" crlf)
    (if (neq ?sat TRUE) then
        (printout t "Debug: Goal formulated from promise" crlf)
    )

    (bind ?goal-id (sym-cat ?class - (gensym*)))
    (bind ?resources (create$ ?c))
    (assert (goal (id ?goal-id)
                    (class ?class) (sub-type ?subtype)
                    (parent ?parent)
                    (priority 10.0)
                    (params robot ?r
                            container ?c
                    )
                    (required-resources
                      (compute-required-resources ?resources ?promised-from-goals ?sat))
    ))

    (assert
        (domain-promise (name RESOURCES) (param-values ?resources) (promising-goal ?goal-id) (valid-at ?*PROMISES-MAX-FUTURE*) (negated FALSE))
    )
)

(defrule goal-class-production-run-one-is-expanded
  (declare (salience ?*SALIENCE-GOAL-EXPAND*))
  ?goal <- (goal (class PRODUCTION-TRY-ALL) (id ?parent) (mode SELECTED))
  (goal (parent ?parent) (mode FORMULATED))
  =>
  (modify ?goal (mode EXPANDED))
)
