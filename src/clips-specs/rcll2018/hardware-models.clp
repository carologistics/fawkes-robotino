(defrule hm-add-component
  ?hc <- (hm-component (name ?name) (state ?state))
  (domain-fact (name self) (param-values ?r))
  =>
  (retract ?hc)
  (assert (wm-fact (key hardware component args? r ?r n ?name s ?state)))
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
  ?c <- (wm-fact (key hardware component args? r ?r n ?name ?s ?state))
  (wm-fact (key hardware edge args? comp ?comp from ?state to ?to transition ?trans))
  =>
  (modify ?c (key hardware component args? r ?r n ?name s ?to))
  (retract ?t)
)

(defrule hm-invalid-transition
  (domain-fact (name self) (param-values ?r))
  ?t <- (hm-transition (component ?comp) (transition ?trans))
  (wm-fact (key hardware component args? r ?r n ?comp s ?state))
  (not (wm-fact (key hardware edge args? comp ?comp from ?state to ?to transition ?trans $?)))
  =>
  (retract ?t)
  (printout error "Invalid transition " ?trans " for component " ?comp " in state " ?state crlf)
)
