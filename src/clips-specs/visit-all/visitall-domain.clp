;---------------------------------------------------------------------------
;  visitall-domain.clp - Simple domain to visit some/all locations
;
;  Created: Thu 26 Oct 2017 21:46:58 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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
    (pddl-formula (id visit-precond) (part-of visit) (type negation))
    (pddl-predicate
      (part-of visit-precond) (predicate visited) (param-names to))
    (domain-effect
      (part-of visit) (predicate visited) (param-names to))
  )
)
