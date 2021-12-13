;---------------------------------------------------------------------------
;  domain.clp - Domain configuration
;
;  Created: Thu Dec 2 2021
;  Copyright  2021 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(defrule domain-load
  (executive-init)
  (not (domain-loaded))
=>
  (parse-pddl-domain (path-resolve "xenonite/domain.pddl"))
  ;(assert (skiller-control))
  (assert (domain-loaded))
)

(defrule domain-load-start
  (domain-fact (name self) (param-values ?self&WALL-E))
  (domain-loaded)
  =>
  (assert (wm-fact (key domain initialization) (type STRING) (value "STARTED")))
)


(defrule domain-load-local-facts
  "Load local facts only relevant for this robot."
  (domain-loaded)
  (wm-fact (key domain initialization) (type STRING) (value "STARTED"))
  (domain-fact (name self) (param-values ?self))
  =>
  (printout info "Initializing local domain facts" crlf)
  (assert
    (domain-fact (name robot-can-carry) (param-values ?self))
    (domain-fact (name location-part-of-machine) (param-values MACHINE1-INPUT MACHINE1))
    (domain-fact (name location-part-of-machine) (param-values MACHINE1-OUTPUT MACHINE1))
    (domain-fact (name location-part-of-machine) (param-values MACHINE2-INPUT MACHINE2))
    (domain-fact (name location-part-of-machine) (param-values MACHINE2-OUTPUT MACHINE2))

    (domain-fact (name location-is-spacious) (param-values CONTAINER-DEPOT))
    (domain-fact (name location-is-spacious) (param-values BASE))
    (domain-fact (name location-is-small) (param-values MACHINE1-INPUT))
    (domain-fact (name location-is-small) (param-values MACHINE2-INPUT))
    (domain-fact (name location-is-small) (param-values MACHINE1-OUTPUT))
    (domain-fact (name location-is-small) (param-values MACHINE2-OUTPUT))
    (domain-fact (name location-is-small) (param-values REGOLITH-MINE1))
    (domain-fact (name location-is-small) (param-values REGOLITH-MINE2))
    (domain-fact (name location-is-small) (param-values STORAGE-INPUT))
    (domain-fact (name location-is-mine) (param-values REGOLITH-MINE1))
    (domain-fact (name location-is-mine) (param-values REGOLITH-MINE2))
    (domain-fact (name location-is-machine) (param-values MACHINE1-INPUT))
    (domain-fact (name location-is-machine) (param-values MACHINE1-OUTPUT))
    (domain-fact (name location-is-machine) (param-values MACHINE2-INPUT))
    (domain-fact (name location-is-machine) (param-values MACHINE2-OUTPUT))
    (domain-fact (name location-is-machine-input) (param-values MACHINE1-INPUT))
    (domain-fact (name location-is-machine-output) (param-values MACHINE1-OUTPUT))
    (domain-fact (name location-is-machine-input) (param-values MACHINE2-INPUT))
    (domain-fact (name location-is-machine-output) (param-values MACHINE2-OUTPUT))
    (domain-fact (name machine-for-material) (param-values MACHINE1 REGOLITH))
    (domain-fact (name machine-for-material) (param-values MACHINE2 PROCESSITE))
    (domain-fact (name machine-makes-material) (param-values MACHINE1 PROCESSITE))
    (domain-fact (name machine-makes-material) (param-values MACHINE2 XENONITE))

    (wm-fact (key robot-ready ?self) (type BOOL) (value TRUE))
  )
)

(defrule domain-load-global-facts
  "Load global facts shared by all robots."
  (domain-fact (name self) (param-values ?self&WALL-E))
  ?init <- (wm-fact (key domain initialization) (type STRING) (value "STARTED"))
  (wm-fact (key robot-ready WALL-E) (value TRUE))
  (wm-fact (key robot-ready EVE) (value TRUE))
  (wm-fact (key robot-ready R2D2) (value TRUE))
  (domain-loaded)
  =>
  (printout info "Initializing global domain facts" crlf)

  (assert
    (domain-object (name WALL-E) (type robot))
    (domain-object (name EVE) (type robot))
    (domain-object (name R2D2) (type robot))
    (domain-object (name ARNIE) (type robot))
    (domain-object (name C1) (type container))
    (domain-object (name C2) (type container))
    (domain-object (name C3) (type container))
    ;(domain-object (name C4) (type container))
    (domain-fact (name robot-at) (param-values WALL-E BASE))
    (domain-fact (name robot-at) (param-values EVE BASE))
    (domain-fact (name robot-at) (param-values R2D2 BASE))
    ;(domain-fact (name robot-at) (param-values ARNIE BASE))

    (domain-fact (name container-can-be-filled) (param-values C1))
    (domain-fact (name container-can-be-filled) (param-values C2))
    (domain-fact (name container-can-be-filled) (param-values C3))
    ;(domain-fact (name container-can-be-filled) (param-values C4))
    (domain-fact (name container-at) (param-values C1 CONTAINER-DEPOT))
    (domain-fact (name container-at) (param-values C2 CONTAINER-DEPOT))
    (domain-fact (name container-at) (param-values C3 CONTAINER-DEPOT))
    ;(domain-fact (name container-at) (param-values C4 CONTAINER-DEPOT))
    (domain-fact (name container-for-robot) (param-values C1 WALL-E))
    (domain-fact (name container-for-robot) (param-values C2 EVE))
    (domain-fact (name container-for-robot) (param-values C3 R2D2))
    ;(domain-fact (name container-for-robot) (param-values C4 ARNIE))

    (domain-fact (name machine-in-state) (param-values MACHINE1 IDLE))
    (domain-fact (name machine-in-state) (param-values MACHINE2 IDLE))

    (domain-fact (name location-is-free) (param-values REGOLITH-MINE1))
    (domain-fact (name location-is-free) (param-values REGOLITH-MINE2))
    (domain-fact (name location-is-free) (param-values MACHINE1-INPUT))
    (domain-fact (name location-is-free) (param-values MACHINE1-OUTPUT))
    (domain-fact (name location-is-free) (param-values MACHINE2-INPUT))
    (domain-fact (name location-is-free) (param-values MACHINE2-OUTPUT))
    (domain-fact (name location-is-free) (param-values STORAGE-INPUT))
    (domain-fact (name location-is-free) (param-values CONTAINER-DEPOT))
    (wm-fact (key domain initialized) (type BOOL) (value TRUE))
  )

  (modify ?init (value "FINISHED"))
)

(defrule domain-initialization-complete
  (wm-fact (key domain initialization) (type STRING) (value "FINISHED"))
  =>
  (printout t "Domain initialization complete!" crlf)
  (assert (domain-facts-loaded))
)

