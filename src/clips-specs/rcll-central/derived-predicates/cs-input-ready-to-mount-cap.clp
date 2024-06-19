(defrule cs-input-ready-to-mount-cap-positive
" CS is ready to mount if a wp is at input that needs the buffered cap color
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name mps-type) (param-values ?mps CS))
  (domain-fact (name wp-at) (param-values ?wp ?mps INPUT))
  (domain-fact (name cs-buffered) (param-values ?mps ?cap-color))
  (not (domain-fact (name cs-input-ready-to-mount-cap)
    (param-values ?mps ?cap-color)))
  (domain-fact (name next-step) (param-values ?wp CAP))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (domain-fact (name order-cap-color) (param-values ?order ?cap-color))
 =>
  (assert (domain-fact (name cs-input-ready-to-mount-cap)
    (param-values ?mps ?cap-color)
  ))
)

(defrule cs-input-ready-to-mount-cap-negative
" CS is not ready anymore once the input is free again."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name cs-input-ready-to-mount-cap) (param-values ?mps ?))
  (domain-fact (name mps-side-free) (param-values ?mps INPUT))
 =>
  (retract ?f)
)
