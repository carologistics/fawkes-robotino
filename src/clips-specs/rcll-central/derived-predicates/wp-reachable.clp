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
  (domain-fact (name mps-type) (param-values ?bs BS))
  (domain-fact (name wp-base-color) (param-values ?wp BASE_NONE))
  (domain-fact (name mps-found) (param-values ?bs))
  (not (domain-fact (name wp-reachable) (param-values ?robot ?wp)))
  (domain-fact (name can-hold) (param-values ?robot))
  (or
    (not (domain-fact (name bs-side-in-use) (param-values ?bs INPUT)))
    (not (domain-fact (name bs-side-in-use) (param-values ?bs OUTPUT)))
  )
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

(defrule wp-reachable-negative-bs-busy
" If the bs has both sides in use and the wp is in the bs, the wp is not reachable.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name wp-reachable) (param-values ?robot ?wp))
  (domain-fact (name wp-base-color) (param-values ?wp BASE_NONE))
  (domain-fact (name bs-side-in-use) (param-values ?bs INPUT))
  (domain-fact (name bs-side-in-use) (param-values ?bs OUTPUT))
  =>
  (retract ?f)
)
