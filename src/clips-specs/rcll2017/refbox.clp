;---------------------------------------------------------------------------
;  refbox.clp - Refbox communitcation and actions
;
;  Created: Mon 8 Jan 2017 13:21:21 CET
;  Copyright  2017  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(defrule refbox-init
  "Initialization of refbox related facts."
  (executive-init)
  (domain-loaded)
  =>
  (assert 
    (wm-fact (id "/refbox/team-color") )
    (wm-fact (id "/refbox/points/magenta") (type UINT) (value 0) )
    (wm-fact (id "/refbox/points/cyan") (type UINT) (value 0) )
    (wm-fact (id "/refbox/phase")  (type UNKNOWN) (value PRE_GAME) )
    (wm-fact (id "/refbox/state")  (type UNKNOWN) (value WAIT_START) )
  )  
)

