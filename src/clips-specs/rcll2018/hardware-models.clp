(deffunction robot-needs-repair ()
  (do-for-fact ((?df domain-fact)) (and (eq ?df:name comp-state) (eq (nth$ 2 ?df:param-values) BROKEN)
                                        (not (any-factp ((?do domain-object)) (and (eq ?do:name (nth$ 1 ?df:param-values)) (eq ?do:type mps))))
                                   )
     ; (printout error "Component " (nth$ 1 ?df:param-values) " is broken" crlf)
      (return TRUE)
  )
  (do-for-fact ((?hc hm-component)) TRUE
    (if (not (any-factp ((?df domain-fact)) (and (eq ?df:name comp-state)
                                              (eq (nth$ 1 ?df:param-values) ?hc:name))
         )) then
     ; (printout error "Component " ?hc:name " has no state " crlf)
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


(defrule repair-action-realsense-activated-start
    ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (action-name realsense-activate) (state PENDING))
    (GazsimSensingInterface (busy FALSE))    
    =>
    (bind ?msg (blackboard-create-msg "GazsimSensingInterface::GazsimSensing" "ActivateRealsenseMessage"))
    (blackboard-set-msg-field ?msg "dummy" 1)
    (bind ?msg-id (blackboard-send-msg ?msg))
    (modify ?pa (state WAITING) (param-values ?msg-id))  
)

(defrule repair-action-realsense-activated-running
    ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (action-name realsense-activate) (state WAITING))
    (GazsimSensingInterface (busy TRUE))
    =>
    (modify ?pa (state RUNNING))
)

(defrule repair-action-realsense-activated-final
    ?pa <- (plan-action (plan-id ?plan-id) (goal-id ?goal-id) (action-name realsense-activate) (state RUNNING|WAITING) (param-values ?msg-id))
    (GazsimSensingInterface (busy FALSE) (last_sensed "realsense") (ret ?ret) (msgid ?msg-id))
    =>
    (modify ?pa (state (if (eq ?ret 1) then FINAL else FAILED)))
)
