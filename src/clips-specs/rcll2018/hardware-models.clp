(deffunction robot-needs-repair ()
  (do-for-fact ((?df domain-fact)) (and (eq ?df:name comp-state) (eq (nth$ 1 ?df:param-values) BROKEN))
      (printout error "Component " (nth$ 2 ?df:param-values) " is broken" crlf)
      (return TRUE)
  )
  (do-for-fact ((?hc hm-component)) TRUE
    (if (not (any-factp ((?df domain-fact)) (and (eq ?df:name comp-state)
                                              (eq (nth$ 1 ?df:param-values) ?hc:name))
         )) then
      (printout error "Component " ?hc:name " has no state " crlf)
      (return TRUE)
    )
  )
  (return FALSE)
)

(defrule hm-add-component
  ?hc <- (hm-component (name ?name) (initial-state ?state))
  (domain-fact (name self) (param-values ?r))
  =>
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
