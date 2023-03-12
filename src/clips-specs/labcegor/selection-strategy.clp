;(wm-fact (key refbox order ?order-id delivery-begin) (type UINT) (value ?begin) )
;(wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec (/ ?nsec 1000)))


(defrule select-orders

  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2|GOAL-ORDER-C3)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord1))

  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin 60) (> ?sec (- ?begin 60) )) (and (< ?begin 60) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 120) )))
  ; Earliest (modified) Deadline First

  ; (not (and 
  ;           (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2|GOAL-ORDER-C3))
  ;           (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
  ;           (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 60) (> ?sec (- ?begin2 60) )) (and (< ?begin2 60) (>= ?sec ?begin2 )))))
  ;           (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 120) ))))
  ;       )
  ; )

    ; (not (goal (class GOAL-ORDER-C0) (mode ~FORMULATED) (outcome ~COMPLETED)))
    ; (not (goal (class GOAL-ORDER-C1) (mode ~FORMULATED) (outcome ~COMPLETED)))
    ; (not (goal (class GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)))
    ; (not (goal (class GOAL-ORDER-C3) (mode ~FORMULATED) (outcome ~COMPLETED))) ;This has the effect, that always there is only one root-order selected which is not COMPLETED


;  (not (goal (id ~?sub-goal) (parent ?id) (type ACHIEVE) (outcome ?outcome&~COMPLETED)
;    (priority ?priority2&:(< ?priority2 ?priority)) (is-executable TRUE)))
  
  ;HARDCODED for Goals of Complexity C0:
  (domain-fact (name mps-team) (param-values ?bs ?team-color))
  (domain-fact (name mps-type) (param-values ?bs BS))
  (not 
    (and
      (domain-fact (name mps-team) (param-values ?bs ?team-color))
      (domain-fact (name mps-type) (param-values ?bs BS))
      (machine-used (mps ?bs) (order-id ?some-order-id1))
    )
  )

  (domain-fact (name order-cap-color) (param-values ?ord1 ?capcol))
  (domain-fact (name mps-team) (param-values ?cs ?team-color))
  (domain-fact (name mps-type) (param-values ?cs CS))
  (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
  (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))

  (not 
    (and 
         (domain-fact (name order-cap-color) (param-values ?ord1 ?capcol))
         (domain-fact (name mps-team) (param-values ?cs ?team-color))
         (domain-fact (name mps-type) (param-values ?cs CS))
         (domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
         (domain-fact (name wp-cap-color) (param-values ?wp ?capcol))
         (machine-used (mps ?cs) (order-id ?some-order-id2))
    )
  )
      

  =>
  (printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
  (modify ?g (mode SELECTED))

  ;HARDCODED for Goals of Complexity C0:
  (assert (machine-used (mps ?bs) (order-id ?ord1)))
  (assert (machine-used (mps ?cs) (order-id ?ord1)))
)

; Machine releases 

; Releases in c0-order:
; release bs in c0-order
(defrule free-c0-bs
  ?mu <- (machine-used (mps ?bs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?bs BS))
  (goal (id ?goal-id) (class GOAL-TO-CS) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C0))

  =>
  (retract ?mu)
)
; release cs and ds in c0-order
(defrule free-c0-cs-and-ds
  ?mucs <- (machine-used (mps ?cs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?cs CS))
  ?muds <- (machine-used (mps ?ds) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?ds DS))
  (goal (id ?goal-id) (class GOAL-DELIVER-C0) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C0))

  =>
  (retract ?mucs)
  (retract ?muds)
)
; release ds in c0-order

; Releases in c1-order:

(defrule free-c1-bs
  ?mu <- (machine-used (mps ?bs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?bs BS))
  (goal (id ?goal-id) (class GOAL-TO-RS1) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C1))
  =>
  (retract ?mu)
)

(defrule free-c1-rs
  ?mu <- (machine-used (mps ?rs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?rs RS))
  (goal (id ?goal-id) (class GOAL-TO-CS) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C1))
  =>
  (retract ?mu)
)

(defrule free-c1-cs-and-ds
  ?mucs <- (machine-used (mps ?cs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?cs CS))
  ?muds <- (machine-used (mps ?ds) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?ds DS))
  (goal (id ?goal-id) (class GOAL-DELIVER-C1) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C1))
  =>
  (retract ?mucs)
  (retract ?muds)
)

; Releases in c2-order:

(defrule free-c2-bs
  ?mu <- (machine-used (mps ?bs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?bs BS))
  (goal (id ?goal-id) (class GOAL-TO-RS1) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C2))
  =>
  (retract ?mu)
)

(defrule free-c2-rs1

)

(defrule free-c2-rs2

)


(defrule free-c2-cs-and-ds
  ?mucs <- (machine-used (mps ?cs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?cs CS))
  ?muds <- (machine-used (mps ?ds) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?ds DS))
  (goal (id ?goal-id) (class GOAL-DELIVER-C2) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C2))
  =>
  (retract ?mucs)
  (retract ?muds)
)
