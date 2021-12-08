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
