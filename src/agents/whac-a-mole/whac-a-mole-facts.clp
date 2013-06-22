
;---------------------------------------------------------------------------
;  Whac-a-mole-facts.clp
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;                   Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate navpath
  (slot start (type SYMBOL))
  (slot goal (type SYMBOL))
)

(deffacts startup
  (navpath (start P21) (goal P41))
  (navpath (start P41) (goal P44))
  (navpath (start P44) (goal P14))
  (navpath (start P14) (goal P21))

  (state WAIT_START)
  (phase PRE_GAME)
  (refbox-state WAIT_START)

  (driven-to P44)
)
