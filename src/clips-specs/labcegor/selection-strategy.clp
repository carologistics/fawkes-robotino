(defglobal ?*C0_start* = 100)
(defglobal ?*C0_end* = 80)
(defglobal ?*C1_start* = 100)
(defglobal ?*C1_end* = 100)
(defglobal ?*C2_start* = 100)
(defglobal ?*C2_end* = 110)
(defglobal ?*C3_start* = 180)
(defglobal ?*C3_end* = 160)

(defrule select-c0-orders

; Goal to select
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C0)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord1))

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin ?*C0_start*) (> ?sec (- ?begin ?*C0_start*) )) (and (< ?begin ?*C0_start*) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 ?*C0_end*) )))

; Earliest (modified) Deadline First

(not (and 
          (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class1&GOAL-ORDER-C0))
          (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
          (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 ?*C0_start*) (> ?sec (- ?begin2 ?*C0_start*) )) (and (< ?begin2 ?*C0_start*) (>= ?sec ?begin2 )))))
          (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 ?*C0_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id3) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class2&GOAL-ORDER-C1))
          (goal-meta (goal-id ?goal-id3) (order-id ?ord3))
          (wm-fact (key refbox order ?ord3 delivery-begin) (type UINT) (value ?begin3&:(or (and (>= ?begin3 ?*C1_start*) (> ?sec (- ?begin3 ?*C1_start*) )) (and (< ?begin3 ?*C1_start*) (>= ?sec ?begin3 )))))
          (wm-fact (key refbox order ?ord3 delivery-end) (type UINT) (value ?end3&:(and (< ?end3 ?end1) (< ?sec (- ?end3 ?*C1_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id4) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class3&GOAL-ORDER-C2))
          (goal-meta (goal-id ?goal-id4) (order-id ?ord4))
          (wm-fact (key refbox order ?ord4 delivery-begin) (type UINT) (value ?begin4&:(or (and (>= ?begin4 ?*C2_start*) (> ?sec (- ?begin4 ?*C2_start*) )) (and (< ?begin4 ?*C2_start*) (>= ?sec ?begin4 )))))
          (wm-fact (key refbox order ?ord4 delivery-end) (type UINT) (value ?end4&:(and (< ?end4 ?end1) (< ?sec (- ?end4 ?*C2_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id5) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class4&GOAL-ORDER-C3))
          (goal-meta (goal-id ?goal-id5) (order-id ?ord5))
          (wm-fact (key refbox order ?ord5 delivery-begin) (type UINT) (value ?begin5&:(or (and (>= ?begin5 ?*C3_start*) (> ?sec (- ?begin5 ?*C3_start*) )) (and (< ?begin5 ?*C3_start*) (>= ?sec ?begin5 )))))
          (wm-fact (key refbox order ?ord5 delivery-end) (type UINT) (value ?end5&:(and (< ?end5 ?end1) (< ?sec (- ?end5 ?*C3_end*) ))))
      )
)

; Check for BS availability 
(domain-fact (name mps-team) (param-values ?bs ?team-color))
(domain-fact (name mps-type) (param-values ?bs BS))
(not (machine-used (mps ?bs) (order-id ?some-order-id1)))


; Check for CS availability 
(domain-object (name ?team-color) (type team-color)) ; This selects our team color, as this fact only exists for our own team
(domain-fact (name order-cap-color) (param-values ?ord1 ?capcol))
(domain-fact (name mps-team) (param-values ?cs ?team-color))
(domain-fact (name mps-type) (param-values ?cs CS))
(domain-fact (name wp-on-shelf) (param-values ?wp ?cs ?spot))
(domain-fact (name wp-cap-color) (param-values ?wp ?capcol))

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

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin ?*C1_start*) (> ?sec (- ?begin ?*C1_start*) )) (and (< ?begin ?*C1_start*) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 ?*C1_end*) )))

; Earliest (modified) Deadline First

(not (and 
          (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class1&GOAL-ORDER-C0))
          (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
          (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 ?*C0_start*) (> ?sec (- ?begin2 ?*C0_start*) )) (and (< ?begin2 ?*C0_start*) (>= ?sec ?begin2 )))))
          (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 ?*C0_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id3) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class2&GOAL-ORDER-C1))
          (goal-meta (goal-id ?goal-id3) (order-id ?ord3))
          (wm-fact (key refbox order ?ord3 delivery-begin) (type UINT) (value ?begin3&:(or (and (>= ?begin3 ?*C1_start*) (> ?sec (- ?begin3 ?*C1_start*) )) (and (< ?begin3 ?*C1_start*) (>= ?sec ?begin3 )))))
          (wm-fact (key refbox order ?ord3 delivery-end) (type UINT) (value ?end3&:(and (< ?end3 ?end1) (< ?sec (- ?end3 ?*C1_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id4) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class3&GOAL-ORDER-C2))
          (goal-meta (goal-id ?goal-id4) (order-id ?ord4))
          (wm-fact (key refbox order ?ord4 delivery-begin) (type UINT) (value ?begin4&:(or (and (>= ?begin4 ?*C2_start*) (> ?sec (- ?begin4 ?*C2_start*) )) (and (< ?begin4 ?*C2_start*) (>= ?sec ?begin4 )))))
          (wm-fact (key refbox order ?ord4 delivery-end) (type UINT) (value ?end4&:(and (< ?end4 ?end1) (< ?sec (- ?end4 ?*C2_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id5) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class4&GOAL-ORDER-C3))
          (goal-meta (goal-id ?goal-id5) (order-id ?ord5))
          (wm-fact (key refbox order ?ord5 delivery-begin) (type UINT) (value ?begin5&:(or (and (>= ?begin5 ?*C3_start*) (> ?sec (- ?begin5 ?*C3_start*) )) (and (< ?begin5 ?*C3_start*) (>= ?sec ?begin5 )))))
          (wm-fact (key refbox order ?ord5 delivery-end) (type UINT) (value ?end5&:(and (< ?end5 ?end1) (< ?sec (- ?end5 ?*C3_end*) ))))
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

  ; There is not already a C2 goal running
  (not (goal (id ?goal-idr3) (class ?rclass1&GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)))
  ; There is not already a C3 goal running
  (not (goal (id ?goal-idr4) (class ?rclass2&GOAL-ORDER-C3) (mode ~FORMULATED) (outcome ~COMPLETED)))

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin ?*C2_start*) (> ?sec (- ?begin ?*C2_start*) )) (and (< ?begin ?*C2_start*) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 ?*C2_end*) )))

  ; Earliest (modified) Deadline First
(not (and 
          (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class1&GOAL-ORDER-C0))
          (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
          (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 ?*C0_start*) (> ?sec (- ?begin2 ?*C0_start*) )) (and (< ?begin2 ?*C0_start*) (>= ?sec ?begin2 )))))
          (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 ?*C0_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id3) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class2&GOAL-ORDER-C1))
          (goal-meta (goal-id ?goal-id3) (order-id ?ord3))
          (wm-fact (key refbox order ?ord3 delivery-begin) (type UINT) (value ?begin3&:(or (and (>= ?begin3 ?*C1_start*) (> ?sec (- ?begin3 ?*C1_start*) )) (and (< ?begin3 ?*C1_start*) (>= ?sec ?begin3 )))))
          (wm-fact (key refbox order ?ord3 delivery-end) (type UINT) (value ?end3&:(and (< ?end3 ?end1) (< ?sec (- ?end3 ?*C1_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id4) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class3&GOAL-ORDER-C2))
          (goal-meta (goal-id ?goal-id4) (order-id ?ord4))
          (wm-fact (key refbox order ?ord4 delivery-begin) (type UINT) (value ?begin4&:(or (and (>= ?begin4 ?*C2_start*) (> ?sec (- ?begin4 ?*C2_start*) )) (and (< ?begin4 ?*C2_start*) (>= ?sec ?begin4 )))))
          (wm-fact (key refbox order ?ord4 delivery-end) (type UINT) (value ?end4&:(and (< ?end4 ?end1) (< ?sec (- ?end4 ?*C2_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id5) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class4&GOAL-ORDER-C3))
          (goal-meta (goal-id ?goal-id5) (order-id ?ord5))
          (wm-fact (key refbox order ?ord5 delivery-begin) (type UINT) (value ?begin5&:(or (and (>= ?begin5 ?*C3_start*) (> ?sec (- ?begin5 ?*C3_start*) )) (and (< ?begin5 ?*C3_start*) (>= ?sec ?begin5 )))))
          (wm-fact (key refbox order ?ord5 delivery-end) (type UINT) (value ?end5&:(and (< ?end5 ?end1) (< ?sec (- ?end5 ?*C3_end*) ))))
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


(defrule select-c3-orders

; Goal to select
  ?g <- (goal (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class&GOAL-ORDER-C3)
      (id ?goal-id) (mode FORMULATED) (is-executable TRUE) (verbosity ?v))
  (goal-meta (goal-id ?goal-id) (order-id ?ord1))

  ; There is not already a C2 goal running
  (not (goal (id ?goal-idr3) (class ?rclass1&GOAL-ORDER-C2) (mode ~FORMULATED) (outcome ~COMPLETED)))
  ; There is not already a C3 goal running
  (not (goal (id ?goal-idr4) (class ?rclass2&GOAL-ORDER-C3) (mode ~FORMULATED) (outcome ~COMPLETED)))

; Goal is in sliding window 
  (wm-fact (key refbox game-time) (is-list TRUE) (type UINT) (values ?sec ?nsec))
  (wm-fact (key refbox order ?ord1 delivery-begin) (type UINT) (value ?begin&:(or (and (>= ?begin ?*C3_start*) (> ?sec (- ?begin ?*C3_start*) )) (and (< ?begin ?*C3_start*) (>= ?sec ?begin )))))
  (wm-fact (key refbox order ?ord1 delivery-end) (type UINT) (value ?end1&:(< ?sec (- ?end1 ?*C3_end*) )))

; Earliest (modified) Deadline First

(not (and 
          (goal (id ?goal-id2) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class1&GOAL-ORDER-C0))
          (goal-meta (goal-id ?goal-id2) (order-id ?ord2))
          (wm-fact (key refbox order ?ord2 delivery-begin) (type UINT) (value ?begin2&:(or (and (>= ?begin2 ?*C0_start*) (> ?sec (- ?begin2 ?*C0_start*) )) (and (< ?begin2 ?*C0_start*) (>= ?sec ?begin2 )))))
          (wm-fact (key refbox order ?ord2 delivery-end) (type UINT) (value ?end2&:(and (< ?end2 ?end1) (< ?sec (- ?end2 ?*C0_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id3) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class2&GOAL-ORDER-C1))
          (goal-meta (goal-id ?goal-id3) (order-id ?ord3))
          (wm-fact (key refbox order ?ord3 delivery-begin) (type UINT) (value ?begin3&:(or (and (>= ?begin3 ?*C1_start*) (> ?sec (- ?begin3 ?*C1_start*) )) (and (< ?begin3 ?*C1_start*) (>= ?sec ?begin3 )))))
          (wm-fact (key refbox order ?ord3 delivery-end) (type UINT) (value ?end3&:(and (< ?end3 ?end1) (< ?sec (- ?end3 ?*C1_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id4) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class3&GOAL-ORDER-C2))
          (goal-meta (goal-id ?goal-id4) (order-id ?ord4))
          (wm-fact (key refbox order ?ord4 delivery-begin) (type UINT) (value ?begin4&:(or (and (>= ?begin4 ?*C2_start*) (> ?sec (- ?begin4 ?*C2_start*) )) (and (< ?begin4 ?*C2_start*) (>= ?sec ?begin4 )))))
          (wm-fact (key refbox order ?ord4 delivery-end) (type UINT) (value ?end4&:(and (< ?end4 ?end1) (< ?sec (- ?end4 ?*C2_end*) ))))
      )
)
(not (and 
          (goal (id ?goal-id5) (mode FORMULATED) (parent nil) (type ACHIEVE) (sub-type ~nil) (class ?class4&GOAL-ORDER-C3))
          (goal-meta (goal-id ?goal-id5) (order-id ?ord5))
          (wm-fact (key refbox order ?ord5 delivery-begin) (type UINT) (value ?begin5&:(or (and (>= ?begin5 ?*C3_start*) (> ?sec (- ?begin5 ?*C3_start*) )) (and (< ?begin5 ?*C3_start*) (>= ?sec ?begin5 )))))
          (wm-fact (key refbox order ?ord5 delivery-end) (type UINT) (value ?end5&:(and (< ?end5 ?end1) (< ?sec (- ?end5 ?*C3_end*) ))))
      )
)
; Check for BS availability 
(domain-fact (name mps-team) (param-values ?bs ?team-color))
(domain-fact (name mps-type) (param-values ?bs BS))
(not (machine-used (mps ?bs)))


; Check for RS availability (we will block all Ringstations needed to fulfill the C3-Order)
(domain-fact (name order-ring1-color) (param-values ?ord1 ?col1))
(domain-fact (name rs-ring-spec) (param-values ?rs1 ?col1 ?rn1))
(not (machine-used (mps ?rs1)))
(domain-fact (name order-ring2-color) (param-values ?ord1 ?col2))
(domain-fact (name rs-ring-spec) (param-values ?rs2 ?col2 ?rn2))
(not (machine-used (mps ?rs2)))
(domain-fact (name order-ring3-color) (param-values ?ord1 ?col3))
(domain-fact (name rs-ring-spec) (param-values ?rs3 ?col3 ?rn3))
(not (machine-used (mps ?rs3)))

=>

(printout (log-debug ?v) "Goal " ?goal-id " SELECTED" crlf)
(modify ?g (mode SELECTED))
; We no block the base station
(assert (machine-used (mps ?bs) (order-id ?ord1)))
(printout (log-debug ?v) ?bs " is in use now for order " ?ord1 crlf)
; We now block the ringstations
(assert (machine-used (mps ?rs1) (order-id ?ord1)))
(printout (log-debug ?v) ?rs1 " is in use now for order " ?ord1 crlf)
(assert (machine-used (mps ?rs2) (order-id ?ord1)))
(printout (log-debug ?v) ?rs2 " is in use now for order " ?ord1 crlf)
(assert (machine-used (mps ?rs3) (order-id ?ord1)))
(printout (log-debug ?v) ?rs3 " is in use now for order " ?ord1 crlf)
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
  (test(not (eq ?rs1 ?rs2)))

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

; Releases for C3-Orders:

(defrule free-c3-bs
  ?mu <- (machine-used (mps ?bs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?bs BS))
  (goal (id ?goal-id) (class GOAL-TO-RS1) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C3))
  =>
  (retract ?mu)
  (printout t ?bs " is freed from order " ?ord crlf)
)

; We do this just in case rs1 is not used for second or third ring
(defrule free-c3-rs1
  ?mu <- (machine-used (mps ?rs1) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?rs RS))
  (domain-fact (name order-ring1-color) (param-values ?ord ?ring1col))
  (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
  (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
  (domain-fact (name rs-ring-spec) (param-values ?rs1 ?ring1col ?num1))
  (domain-fact (name rs-ring-spec) (param-values ?rs2 ?ring2col ?num2))
  (domain-fact (name rs-ring-spec) (param-values ?rs3 ?ring3col ?num3))
  (test(not (eq ?rs1 ?rs2)))
  (test(not (eq ?rs1 ?rs3)))

  (goal (id ?goal-id) (class GOAL-TO-RS2) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C3))
  =>
  (retract ?mu)
  (printout t ?rs1 " is freed from order " ?ord crlf)
)

; We do this just in case rs2 is not used for the third ring
(defrule free-c3-rs2
  ?mu <- (machine-used (mps ?rs2) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?rs RS))
  (domain-fact (name order-ring2-color) (param-values ?ord ?ring2col))
  (domain-fact (name order-ring3-color) (param-values ?ord ?ring3col))
  (domain-fact (name rs-ring-spec) (param-values ?rs2 ?ring2col ?num2))
  (domain-fact (name rs-ring-spec) (param-values ?rs3 ?ring3col ?num3))
  (test(not (eq ?rs2 ?rs3)))

  (goal (id ?goal-id) (class GOAL-TO-RS3) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C3))
  =>
  (retract ?mu)
  (printout t ?rs2 " is freed from order " ?ord crlf)
)

; We do this always
(defrule free-c3-rs3
  ?mu <- (machine-used (mps ?rs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?rs RS))
  (goal (id ?goal-id) (class GOAL-TO-CS) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C3))
  =>
  (retract ?mu)
  (printout t ?rs " is freed from order " ?ord crlf)
)


(defrule free-c3-cs-and-ds
  ?mucs <- (machine-used (mps ?cs) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?cs CS))
  ?muds <- (machine-used (mps ?ds) (order-id ?ord))
  (domain-fact (name mps-type) (param-values ?ds DS))
  (goal (id ?goal-id) (class GOAL-DELIVER-C3) (outcome COMPLETED))
  (goal-meta (goal-id ?goal-id) (order-id ?ord) (root-for-order ?root-id))
  (goal (id ?root-id) (class GOAL-ORDER-C3))
  =>
  (retract ?mucs)
  (retract ?muds)
  (printout t ?cs " is freed from order " ?ord crlf)
  (printout t ?ds " is freed from order " ?ord crlf)
)