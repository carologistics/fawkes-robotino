(defrule cc-reachable-positive
" True, if the robot either 1) holds a correct cap-carrier or
  2) can pick one up.
  This rule takes care of case 1).
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name at) (param-values ?robot $?))
  (domain-fact (name wp-base-color) (param-values ?wp BASE_CLEAR))
  (domain-fact (name wp-cap-color) (param-values ?wp ?cap-color))
  (not (domain-fact (name cc-reachable) (param-values ?robot ?cap-color)))

  (domain-fact (name can-hold) (param-values ?robot))
  (or
    (domain-fact (name wp-on-shelf) (param-values ?wp $?))
    (domain-fact (name wp-at) (param-values ?wp $?))
  )
 =>
  (assert (domain-fact (name cc-reachable) (param-values ?robot ?cap-color)))
)

(defrule cc-reachable-positive-holding
" This rule takes care of case 2).
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  (domain-fact (name at) (param-values ?robot $?))
  (domain-fact (name wp-base-color) (param-values ?wp BASE_CLEAR))
  (domain-fact (name wp-cap-color) (param-values ?wp ?cap-color))
  (not (domain-fact (name cc-reachable) (param-values ?robot ?cap-color)))

  (domain-fact (name holding) (param-values ?robot ?wp))
 =>
  (assert (domain-fact (name cc-reachable) (param-values ?robot ?cap-color)))
)

(defrule cc-reachable-negative-holding
" This rule forms the negation of case 2).
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name cc-reachable) (param-values ?robot ?cap-color))
  (domain-fact (name holding) (param-values ?robot ?holding-wp))
  (or
    (domain-fact (name wp-base-color) (param-values ?holding-wp ~BASE_CLEAR))
    (domain-fact (name wp-cap-color) (param-values ?holding-wp ~?cap-color))
  )
 =>
  (retract ?f)
)

(defrule cc-reachable-negative-wp-not-reachable
" This rule forms the negation of case 1), only needed in case of a cap-less
  cap-carrier. Those with caps are always available on CS shelves.
"
  (declare (salience ?*SALIENCE-DOMAIN-CHECK*))
  ?f <- (domain-fact (name cc-reachable) (param-values ?robot CAP_NONE))
  (domain-fact (name can-hold) (param-values ?robot))
  (not
    (and
      (domain-fact (name wp-at) (param-values ?wp $?))
      (domain-fact (name wp-cap-color) (param-values ?wp CAP_NONE))
    )
  )
 =>
  (retract ?f)
)
