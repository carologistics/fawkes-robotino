(deftemplate machine-used (slot mps) (slot order-id))

(defrule select-orders

; There are not already two goals running
  (not
    and (
      (goal (id ?goal-idr1) (class ?rclass1&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal1
      (goal (id ?goal-idr2) (class ?rclass2&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal2
      not (eq ?goal-idr1 ?goal-idr2)                                                                                           ; and these are not the same
    )
  )

; Goal to select
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord1))

; When there is a running C2 don't select a second C2 
 (if (goal (id ?goal-idr1) (class ?rclass1&GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED))
    not (eq ?class GOAL-ORDER-C2)
 )

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin 60) (> ?sec (- ?begin 60) )) (and (< ?begin 60) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 120) )))

; Earliest (modified) Deadline First
  (not (and 
            (goal (id ?goal-id2) (mode FOmachine-used ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 60) (> ?sec (- ?begin2 60) )) (and (< ?begin2 60) (>= ?sec ?begin2 )))))
            (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 120) ))))
        )
  )

;Always check for BS availability 
  (domain-fact (name mps-team) (param-values ?bs ?team-color))
  (domain-fact (name mps-type) (param-values ?bs BS))
  not (machine-used (mps ?bs))


; If not C0 also check RS
 (if (goal (id ?goal-id) (class ?rclass1&~GOAL-ORDER-C0))
    (domain-fact (name order-ring1-color) (param-values ?ord1 ?col))
    (domain-fact (name rs-ring-spec) (param-values ?rs ?col))
    not (machine-used (mps ?rs))
 )

  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))

  (assert (machine-used (mps ?bs) (order-id ?ord1)))
  (printout (log-debug ?v) ?bs " is in use now for order " ?ord1 crlf)

  ; If not C0 also set rs to "in use"
  (if (goal (id ?goal-id) (class ?rclass1&~GOAL-ORDER-C0))
      (assert (machine-used (mps ?rs) (order-id ?ord1)))
      (printout (log-debug ?v) ?rs " is in use now for order " ?ord1 crlf)
  )
)