(defrule cs-input-ready-to-buffer-cap-positive
" CS is ready to buffer if a wp is at input that has a cap.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name mps-type) (param-values ?mps CS))
  (domain-fact (name wp-at) (param-values ?wp ?mps INPUT))
  (domain-fact (name wp-cap-color)
    (param-values ?order ?cap-color&:(neq ?cap-color CAP_NONE)))
  (not (domain-fact (name cs-input-ready-to-buffer-cap)
    (param-values ?mps ?cap-color)
  ))
 =>
  (assert (domain-fact (name cs-input-ready-to-buffer-cap)
    (param-values ?mps ?cap-color)))
)

(defrule cs-input-ready-to-buffer-cap-negative
" CS is not ready anymore once the input is free again."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name cs-input-ready-to-buffer-cap) (param-values ?mps ?))
  (domain-fact (name mps-side-free) (param-values ?mps INPUT))
 =>
  (retract ?f)
)
