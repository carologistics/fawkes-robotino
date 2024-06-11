(defrule ds-input-ready-to-deliver-positive
" DS is ready to deliver if the delivery window of the wp at the input began."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name wp-at) (param-values ?wp ?mps INPUT))
  (domain-fact (name mps-type) (param-values ?mps DS))
  (not (domain-fact (name ds-input-ready-to-deliver) (param-values ?mps ?wp)))
  (wm-fact (key refbox team-color) (value ?team-color))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order delivery-begin) (type UINT)
           (value ?begin&:(< ?begin (nth$ 1 ?game-time))))
 =>
  (assert (domain-fact (name ds-input-ready-to-deliver) (param-values ?mps ?wp)))
)

(defrule ds-input-ready-to-deliver-negative
" DS is not ready to deliver anymore, if the input is free again."
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name ds-input-ready-to-deliver) (param-values ?mps ?wp))
  (domain-fact (name mps-side-free) (param-values ?mps INPUT))
 =>
  (retract ?f)
)
