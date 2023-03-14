(defrule select-c0-orders

; Goal to select
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C0)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord1))

; There are not already two goals running
  (not
    (and 
      (goal (id ?goal-idr1) (class ?rclass1&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal1
      (goal (id ?goal-idr2) (class ?rclass2&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal2
      (not (eq ?goal-idr1 ?goal-idr2))                                                                                          ; and these are not the same
    )
  )

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin 60) (> ?sec (- ?begin 60) )) (and (< ?begin 60) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 120) )))
; Earliest (modified) Deadline First

  (not (and 
            (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class2&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2))
            (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
            (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 60) (> ?sec (- ?begin2 60) )) (and (< ?begin2 60) (>= ?sec ?begin2 )))))
            (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 120) ))))
        )
  )

; Check for BS availability 
(domain-fact (name mps-team) (param-values ?bs ?team-color))
(domain-fact (name mps-type) (param-values ?bs BS))
(not (machine-used (mps ?bs) (order-id ?some-order-id1)))


; Check for CS availability 
(domain-fact (name order-cap-color) (param-values ?ord1 ?col))
(domain-fact (name cs-color) (param-values ?cs ?col))
(not (machine-used (mps ?cs) (order-id ?some-order-id2)))

=>

(printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
(modify ?g (mode SELECTED))

(assert (machine-used (mps ?bs) (order-id ?ord1)))
(printout (log-debug ?v) ?bs " is in use now for order " ?ord1 crlf)

(assert (machine-used (mps ?cs) (order-id ?ord1)))
(printout (log-debug ?v) ?cs " is in use now for order " ?ord1 crlf)
)



(defrule select-c1-orders

; Goal to select
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C1)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord1))

; There are not already two goals running
  (not
    (and 
      (goal (id ?goal-idr1) (class ?rclass1&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal1
      (goal (id ?goal-idr2) (class ?rclass2&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal2
      (not (eq ?goal-idr1 ?goal-idr2))                                                                                          ; and these are not the same
    )
  )

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin 60) (> ?sec (- ?begin 60) )) (and (< ?begin 60) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 120) )))

; Earliest (modified) Deadline First

(not (and 
          (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class2&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2))
          (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
          (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 60) (> ?sec (- ?begin2 60) )) (and (< ?begin2 60) (>= ?sec ?begin2 )))))
          (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 120) ))))
      )
)

; Check for BS availability 
(domain-fact (name mps-team) (param-values ?bs ?team-color))
(domain-fact (name mps-type) (param-values ?bs BS))
(not (machine-used (mps ?bs)))


; Check for RS availability 
(domain-fact (name order-ring1-color) (param-values ?ord1 ?col))
(domain-fact (name rs-ring-spec) (param-values ?rs ?col ?rn))
(not (machine-used (mps ?rs)))

=>

(printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
(modify ?g (mode SELECTED))

(assert (machine-used (mps ?bs) (order-id ?ord1)))
(printout (log-debug ?v) ?bs " is in use now for order " ?ord1 crlf)

(assert (machine-used (mps ?rs) (order-id ?ord1)))
(printout (log-debug ?v) ?rs " is in use now for order " ?ord1 crlf)
)




(defrule select-c2-orders

; Goal to select
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C2)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord1))

; There are not already two goals running
  (not
    (and 
      (goal (id ?goal-idr1) (class ?rclass1&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal1
      (goal (id ?goal-idr2) (class ?rclass2&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal2
      (not (eq ?goal-idr1 ?goal-idr2))                                                                                          ; and these are not the same
    )
  )

  ; There is not already a C2 goal running
  (not (goal (id ?goal-idr3) (class ?rclass1&GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)))

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin 60) (> ?sec (- ?begin 60) )) (and (< ?begin 60) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 120) )))

; Earliest (modified) Deadline First

(not (and 
          (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class2&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2))
          (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
          (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 60) (> ?sec (- ?begin2 60) )) (and (< ?begin2 60) (>= ?sec ?begin2 )))))
          (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 120) ))))
      )
)

; Check for BS availability 
(domain-fact (name mps-team) (param-values ?bs ?team-color))
(domain-fact (name mps-type) (param-values ?bs BS))
(not (machine-used (mps ?bs)))


; Check for RS availability 
(domain-fact (name order-ring1-color) (param-values ?ord1 ?col))
(domain-fact (name rs-ring-spec) (param-values ?rs ?col ?rn))
(not (machine-used (mps ?rs)))

=>

(printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
(modify ?g (mode SELECTED))

 (assert (machine-used (mps ?bs) (order-id ?ord1)))
 (printout (log-debug ?v) ?bs " is in use now for order " ?ord1 crlf)

(assert (machine-used (mps ?rs) (order-id ?ord1)))
(printout (log-debug ?v) ?rs " is in use now for order " ?ord1 crlf)
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
  (printout t ?bs " is freed from order " ?ord crlf)
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
  (printout t ?cs " is freed from order " ?ord crlf)
  (printout t ?ds " is freed from order " ?ord crlf)
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
  (printout t ?bs " is freed from order " ?ord crlf)
)

(defrule free-c1-rs
  ?mu <- (machine-used (mps ?rs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?rs RS))
  (goal (id ?goal-id) (class GOAL-TO-CS) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C1))
  =>
  (retract ?mu)
  (printout t ?rs " is freed from order " ?ord crlf)
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
  (printout t ?cs " is freed from order " ?ord crlf)
  (printout t ?ds " is freed from order " ?ord crlf)
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
  (printout t ?bs " is freed from order " ?ord crlf)
)

(defrule free-c2-rs1
  ?mu <- (machine-used (mps ?rs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?rs RS))
  (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
  (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
  (domain-fact (name rs-ring-spec) (param-values ?rs1 ?ring1col ?num1))
  (domain-fact (name rs-ring-spec) (param-values ?rs2 ?ring2col ?num2))
  (not (eq ?rs1 ?rs2))

  (goal (id ?goal-id) (class GOAL-TO-RS2) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C2))
  =>
  (retract ?mu)
  (printout t ?rs " is freed from order " ?ord crlf)

)

(defrule free-c2-rs2
  ?mu <- (machine-used (mps ?rs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?rs RS))
  (goal (id ?goal-id) (class GOAL-TO-CS) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C2))
  =>
  (retract ?mu)
  (printout t ?rs " is freed from order " ?ord crlf)
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
  (printout t ?cs " is freed from order " ?ord crlf)
  (printout t ?ds " is freed from order " ?ord crlf)
)
