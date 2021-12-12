;****************************************************************************
;  debug.pddl: Debug output for the test domain
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;****************************************************************************

(defrule debug-create-scenario-check-formula
  (not (pddl-formula (id SCENARIO-CHECK1)))
  (domain-object (type container))
  =>
  (parse-pddl-formula (str-cat
    "(forall (?c - container) "
             "(and (container-at ?c STORAGE-INPUT)
                   (container-filled ?c XENONITE)))")
    "SCENARIO-CHECK")
  (assert (pddl-grounding (id GROUNDING-SCENARIO-CHECK)
                          (formula-root SCENARIO-CHECK)
                          (param-names c)
                          (param-values nil)))
)

(defrule debug-print-container-at
    (domain-fact (name container-at) (param-values ?c ?l))
    =>
    (printout t "DEBUG: Container " ?c " is now at " ?l crlf)
)

(defrule debug-print-robot-at
    (domain-fact (name robot-at) (param-values ?r ?l))
    =>
    (printout t "DEBUG: Robot " ?r " is now at " ?l crlf)
)

(defrule debug-print-machine-in-state
    (domain-fact (name machine-in-state) (param-values ?m ?s))
    =>
    (printout t "DEBUG: Machine " ?m " is now in state " ?s crlf)
)

(defrule debug-print-container-filled-with
    (domain-fact (name container-filled) (param-values ?c ?mat))
    =>
    (printout t "DEBUG: Container " ?c " is now filled with " ?mat crlf)
)

(defrule debug-assert-start-session-timer
    (time ?now ?mills)
    (not (scenario-timer ?))
    (goal (class PRODUCTION-TRY-ALL))
    =>
    (assert (scenario-timer ?now))
)

(defrule debug-scenario-fulfilled
  (grounded-pddl-formula (formula-id SCENARIO-CHECK1) (is-satisfied TRUE))
  (not (scenario-fulfilled))
  (time ?now ?mills)
  ?st <- (scenario-timer ?start)
  =>
  (assert (domain-fact (name storage-is-full)))
  (assert (scenario-fulfilled))
  (do-for-all-facts ((?g goal))
      (retract ?g)
  )
  (printout t "DEBUG: Scenario fulfilled in " (- ?now ?start) "s!" crlf)
  (retract ?st)
)
