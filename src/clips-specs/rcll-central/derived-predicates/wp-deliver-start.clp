(defrule wp-deliver-start-positive
 " If the delivery begin is approaching, the wp is ready to be delivered
   to the DS.
 "
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name order-complexity) (param-values ?order ?com))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (not (domain-fact (name wp-deliver-start) (param-values ?wp)))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order delivery-begin) (type UINT)
    (value ?begin&:(< ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*))))
 =>
  (assert (domain-fact (name wp-deliver-start) (param-values ?wp)))
)

(defrule wp-deliver-start-negative-no-wp
 " Clear the possible delivery indication if the wp is not started anymore.
 "
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name wp-deliver-start) (param-values ?wp))
  (not (wm-fact (key order meta wp-for-order args? wp ?wp $?)))
 =>
  (retract ?f)
)

 (defrule wp-deliver-start-negative-deadline-not-ahead
 " If the delivery begin is in the future, the delivery can not start yet.
   This should not happen in a normal game, but is implemented just for safety
 "
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name wp-deliver-start) (param-values ?wp))
  (wm-fact (key order meta wp-for-order args? wp ?wp ord ?order))
  (wm-fact (key refbox game-time) (values $?game-time))
  (wm-fact (key refbox order ?order delivery-begin) (type UINT)
    (value ?begin&:(>= ?begin (+ (nth$ 1 ?game-time) ?*DELIVER-AHEAD-TIME*))))
 =>
  (retract ?f)
)
