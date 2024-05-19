(defrule rs-input-ready-to-mount-ring-positive
" RS is ready to mount if a wp is at input that needs a ring for which
  there is enough payment.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name mps-type) (param-values ?mps RS))
  (domain-fact (name wp-at) (param-values ?wp ?mps INPUT))
  (domain-fact (name rs-ring-spec) (param-values ?mps ?ring-color ?bases-needed))
  (not (domain-fact (name rs-input-ready-to-mount-ring) (param-values ?mps ?ring-color)))
  (wm-fact (key wp meta next-step args? wp ?wp)
    (value ?ring&:(str-index RING ?ring)))

  (domain-fact (name ?wp-ring-color) (param-values ?wp RING_NONE))
  (test
    (eq ?wp-ring-color (sym-cat wp-ring (sub-string 5 5 ?ring) -color))
  )
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (domain-fact (name ?order-ring-color) (param-values ?order ?ring-color))
  (test
    (eq ?order-ring-color (sym-cat order-ring (sub-string 5 5 ?ring) -color))
  )
  (domain-fact (name rs-filled-with) (param-values ?mps ?bases-filled))
  (domain-fact (name rs-sub) (param-values ?bases-filled ?bases-needed ?bases-remain&ZERO|ONE|TWO|THREE))
 =>
  (assert (domain-fact (name rs-input-ready-to-mount-ring) (param-values ?mps ?ring-color)))
)

(defrule rs-input-ready-to-mount-ring-negative
" RS is not ready anymore once the input is free again."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name rs-input-ready-to-mount-ring) (param-values ?mps ?))
  (domain-fact (name mps-side-free) (param-values ?mps INPUT))
 =>
  (retract ?f)
)
