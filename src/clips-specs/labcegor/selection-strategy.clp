;(wm-fact (key refbox order ?order-id delivery-begin) (type UINT) (value ?begin) )
;(wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec (/ ?nsec 1000)))

(defrule select-orders

  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2|GOAL-ORDER-C3)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord))

  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin 60) (> ?sec (- ?begin 60) )) (and (< ?begin 60) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord delivery-end) (type UINT) (value ?end&:(< ?sec (- ?end 120) )))

    (not (goal (class GOAL-ORDER-C0) (mode ~FORMULATED) (outcome ~COMPLETED)))
    (not (goal (class GOAL-ORDER-C1) (mode ~FORMULATED) (outcome ~COMPLETED)))
    (not (goal (class GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)))
    (not (goal (class GOAL-ORDER-C3) (mode ~FORMULATED) (outcome ~COMPLETED))) ;This has the effect, that always there is only one root-order selected which is not COMPLETED


;  (not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (outcome ?outcome&~COMPLETED)
;    (priority ?priority2&:(< ?priority2 ?priority)) (is-executable TRUE)))

  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))
)