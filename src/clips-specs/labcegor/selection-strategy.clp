;(wm-fact (key refbox order ?order-id delivery-begin) (type UINT) (value ?begin) )
;(wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec (/ ?nsec 1000)))

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

; When threre is a running C2 don't select a second C2 
 (if (goal (id ?goal-idr1) (class ?rclass1&GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)
  not (eq ?class GOAL-ORDER-C2)
 )

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin 60) (> ?sec (- ?begin 60) )) (and (< ?begin 60) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 120) )))

; Earliest (modified) Deadline First
  (not (and 
            (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2|GOAL-ORDER-C3))
            (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
            (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 60) (> ?sec (- ?begin2 60) )) (and (< ?begin2 60) (>= ?sec ?begin2 )))))
            (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 120) ))))
        )
  )

; Select maschines which will be used

;Always select BS
  (domain-fact (name mps-team) (param-values ?bs ?team-color))
  (domain-fact (name mps-type) (param-values ?bs BS))


  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
  (assert (machine-used (mps ?bs) (order-id ?ord1)))
)