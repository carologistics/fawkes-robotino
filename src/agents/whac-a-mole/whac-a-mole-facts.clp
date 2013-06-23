
;---------------------------------------------------------------------------
;  Whac-a-mole-facts.clp
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;                   Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate signal
  (slot type)
  (multislot time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot seq (type INTEGER) (default 1))
  (slot count (type INTEGER) (default 1))
)

(deftemplate active-robot
  (slot name (type SYMBOL) (allowed-values R-1 R-2))
  (multislot last-seen (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
)

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
  (signal (type beacon) (time (create$ 0 0)) (seq 1))

  (driven-to P44)
)
