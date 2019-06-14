(defrule hm-add-component
  ?hc <- (hm-component (name ?name) (state ?state))
  (domain-fact (name self) (param-values ?r))
  =>
  (retract ?hc)
  (assert (domain-fact (name comp-state) (param-values ?name ?state)))
)

(defrule hm-add-edge
  ?he <- (hm-edge (component ?c) (from ?from) (to ?to) (transition ?trans) (probability ?prob) (executable ?ex))
  =>
  (retract ?he)
  (assert (wm-fact (key hardware edge args? comp ?c from ?from to ?to trans ?trans prob ?prob exec (sym-cat ?ex))))
)

(defrule hm-execute-transition
  (domain-fact (name self) (param-values ?r))
  ?t <- (hm-transition (component ?comp) (transition ?trans))
  ?c <- (domain-fact (name comp-state) (param-values ?name ?state))
  (wm-fact (key hardware edge args? comp ?comp from ?state to ?to transition ?trans))
  =>
  (modify ?c (param-values ?name ?to))
  (retract ?t)
)

(defrule hm-invalid-transition
  (domain-fact (name self) (param-values ?r))
  ?t <- (hm-transition (component ?comp) (transition ?trans))
  (domain-fact (name comp-state) (param-values ?comp ?state))
  (not (wm-fact (key hardware edge args? comp ?comp from ?state to ?to transition ?trans $?)))
  =>
  (retract ?t)
  (printout error "Invalid transition " ?trans " for component " ?comp " in state " ?state crlf)
)

(defrule hm-mps-components
  (domain-fact (name mps-state) (param-values ?mps ?state))
  ?cs <- (domain-fact (name comp-state) (param-values ?mps ?prev-state&:(neq ?state ?prev-state)))
  =>
  (if (any-factp ((?wm wm-fact)) (wm-key-prefix ?wm:key (create$ hardware edge args? comp ?mps from ?prev-state to ?state))) then
    (printout t ?mps " changed state from " ?prev-state " to " ?state crlf)
  else
    (printout error ?mps " changed state from " ?prev-state " to " ?state " which is not part of the model" crlf)
  )
  (modify ?cs (param-values ?mps ?state))
)
