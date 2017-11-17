;---------------------------------------------------------------------------
;  domain.clp - Hackathon Domain
;
;  Created: Wed 04 Oct 2017 18:14:57 CEST
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file of cedar
;---------------------------------------------------------------------------

(defrule load-domain
  (executive-init)
  (not (domain-loaded))
=>
  (assert
    (domain-loaded)
    (domain-object-type (name location))
    (domain-object-type (name mps))
    (domain-predicate (name at) (param-names l) (param-types location))
    (domain-operator (name goto))
    ;(domain-operator-parameter (name from) (operator goto) (type location))
    (domain-operator-parameter (name to) (operator goto) (type location))
    (domain-precondition (part-of goto) (type conjunction))
    (domain-effect (part-of goto) (predicate visited) (param-names to))

    (domain-operator (name mps-align))
    (domain-precondition (part-of mps-align) (type conjunction))

    (domain-operator (name conveyor-align))
    (domain-precondition (part-of conveyor-align) (type conjunction))

    (domain-operator (name pick))
    (domain-precondition (part-of pick) (type conjunction))

    (domain-operator (name put))
    (domain-precondition (part-of put) (type conjunction))
    (domain-object-type (name text))
    (domain-operator (name say-hello))
    (domain-precondition (part-of say-hello) (type conjunction))
    (domain-effect
      (part-of say-hello) (predicate said) (param-names hello))
    (domain-operator (name say-goodbye))
    (domain-precondition
      (name say-goodbye-precond) (part-of say-goodbye) (type conjunction))
    (domain-atomic-precondition
      (part-of say-goodbye-precond)
      (predicate said)
      (param-names c)
      (param-constants hello)
    )
    (domain-effect
      (part-of say-goodbye) (predicate said) (param-names goodbye))
  )
)
