(defrule wp-reachable-positive-holding
" If the robot holds the workpiece, it is reachable.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name at) (param-values ?robot $?))
  (domain-fact (name wp-base-color) (param-values ?wp $?))
  (not (domain-fact (name wp-reachable) (param-values ?robot ?wp)))
  (domain-fact (name holding) (param-values ?robot ?wp))
 =>
  (assert (domain-fact (name wp-reachable) (param-values ?robot ?wp)))
)

(defrule wp-reachable-positive-dispensable
" If the robot holds nothing and the workpiece is in the BS, it is reachable.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name at) (param-values ?robot $?))
  (domain-fact (name wp-base-color) (param-values ?wp BASE_NONE))
  (not (domain-fact (name wp-reachable) (param-values ?robot ?wp)))
  (domain-fact (name can-hold) (param-values ?robot))
 =>
  (assert (domain-fact (name wp-reachable) (param-values ?robot ?wp)))
)

(defrule wp-reachable-positive-usable
" If the robot holds nothing and the workpiece is usable, it is reachable.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name at) (param-values ?robot $?))
  (domain-fact (name wp-base-color) (param-values ?wp $?))
  (not (domain-fact (name wp-reachable) (param-values ?robot ?wp)))
  (domain-fact (name can-hold) (param-values ?robot))
  (domain-fact (name wp-usable) (param-values ?wp))
  (not (domain-fact (name holding) (param-values ~?robot ?wp)))
 =>
  (assert (domain-fact (name wp-reachable) (param-values ?robot ?wp)))
)

(defrule wp-reachable-negative-holding
" If the wp is being held, it is only reachable for that robot.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name wp-reachable) (param-values ?robot ?wp))
  (or
    (domain-fact (name holding) (param-values ?robot ~?wp))
    (domain-fact (name holding) (param-values ~?robot ?wp))
  )
 =>
  (retract ?f)
)

(defrule wp-reachable-negative-not-usable
" If the wp is not inside the base station and not usable, it is unreachable.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name wp-reachable) (param-values ?robot ?wp))
  (domain-fact (name can-hold) (param-values ?robot))
  (domain-fact (name wp-base-color) (param-values ?wp ~BASE_NONE))
  (not (domain-fact (name wp-usable) (param-values ?wp)))
 =>
  (retract ?f)
)
