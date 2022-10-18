;---------------------------------------------------------------------------
;  promises.clp - Local promise implementations
;
;  Created: Tue Dec 8 2021
;  Copyright  2021  Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Each application needs a specific implementation of the promise-time translation
(defrule domain-promise-translate-time
    (wm-fact (key refbox game-time) (type UINT) (values ?now ?))
    (not (promise-time (usecs ?now)))
    =>
    (do-for-all-facts ((?p promise-time))
        (retract ?p)
    )
    (assert (promise-time (usecs ?now)))
)

(defrule resource-lock-promise-remove-after-unlock
  (domain-fact (name self) (param-values ?agent))
  ?dp <- (domain-promise (name LOCK-RESOURCES) (promising-agent ?agent) (active TRUE) (param-values $?lock-resources))
  (goal (id ?goal-id) (mode DISPATCHED))
  (plan-action (goal-id ?goal-id) (action-name unlock) (param-values ?value&:(member$ ?value ?lock-resources)) (state FINAL))
  =>
  (modify ?dp (param-values (delete-member$ ?lock-resources ?value)))
)

(defrule resource-lock-promise-remove-after-unlock
  (domain-fact (name self) (param-values ?agent))
  ?dp <- (domain-promise (name LOCATION-LOCK-RESOURCES) (promising-agent ?agent) (active TRUE) (param-values $?lock-resources))
  (goal (id ?goal-id) (mode DISPATCHED))
  (plan-action (goal-id ?goal-id) (action-name location-unlock) (param-values ?location ?side) (state FINAL))
  (test (member$ (create$ ?location ?side) ?lock-resources))
  =>
  (modify ?dp (param-values (delete-member$ ?lock-resources (create$ ?location ?side))))
)

(defrule promise-meta-retract
  ?pm <- (wm-fact (key promise meta args? g ?goal-id $?))
  (not (goal (id ?goal-id)))
  =>
  (retract ?pm)
)
