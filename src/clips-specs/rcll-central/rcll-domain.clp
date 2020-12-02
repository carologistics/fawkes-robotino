;---------------------------------------------------------------------------
;  rcll-domain.clp - Simple domain Of RCLL
;
;  Created:: Mon 10 Jan 2017 13:21:21 CET
;  Copyright  2017  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defrule load-domain
  (executive-init)
  (not (domain-loaded))
=>
  (assert
    (domain-loaded)
    (domain-object-type (name object))
    (domain-object-type (name location))
    (domain-operator (name visit))
    (domain-operator-parameter (operator visit) (type location) (name to))
    (domain-precondition (name visit-precond) (part-of visit) (type negation))
    (domain-atomic-precondition
      (part-of visit-precond) (predicate visited) (param-names to))
    (domain-effect
      (part-of visit) (predicate visited) (param-names to))
  )
  
  (path-load "rcll/refbox.clp")
)
