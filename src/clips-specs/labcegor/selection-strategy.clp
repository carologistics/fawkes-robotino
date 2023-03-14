(defrule select-orders

; Goal to select
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord1))

; There are not already two goals running
  (not
    (and 
      (goal (id ?goal-idr1) (class ?rclass1&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal1
      (goal (id ?goal-idr2) (class ?rclass2&GOAL-ORDER-C0|GOAL-ORDER-C1|GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)) ; Running goal2
      (not (eq ?goal-idr1 ?goal-idr2))                                                                                           ; and these are not the same
    )
  )
; When there is a running C2 don't select a second C2 
 (if (goal (id ?goal-idr1) (class ?rclass1&GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED))
    not (eq ?class GOAL-ORDER-C2)
 )

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin 60) (> ?sec (- ?begin 60) )) (and (< ?begin 60) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 120) )))

; Earliest (modified) Deadline First - Merge Fehler?
  ; (not (and 
  ;           (goal (id ?goal-id2) (mode FOmachine-used ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 60) (> ?sec (- ?begin2 60) )) (and (< ?begin2 60) (>= ?sec ?begin2 )))))
  ;           (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 120) ))))
  ;       )
  ; )

;Always check for BS availability 
  (domain-fact (name mps-team) (param-values ?bs ?team-color))
  (domain-fact (name mps-type) (param-values ?bs BS))
  (not (machine-used (mps ?bs)))


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
)

(defrule free-c2-rs2
  ?mu <- (machine-used (mps ?rs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?rs RS))
  (goal (id ?goal-id) (class GOAL-TO-CS) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C2))
  =>
  (retract ?mu)
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
