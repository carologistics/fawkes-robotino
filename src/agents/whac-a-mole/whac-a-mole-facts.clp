
;---------------------------------------------------------------------------
;  Whac-a-mole-facts.clp
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate machine
  (slot name (type STRING))
  (slot x (type FLOAT))
  (slot y (type FLOAT))
  (slot light (type SYMBOL) (allowed-values GREEN ORANGE RED OFF) (default OFF))
)

(deftemplate navpoint
  (slot name (type STRING))
  (slot x (type FLOAT))
  (slot y (type FLOAT))
)

(deftemplate navpath
  (slot start (type STRING))
  (slot goal (type STRING))
)

;(deftemplate defultCoordinate
;  (slot name (type SYMBOL) (allowed-values UNKNOWN))
;  (slot x (type FLOAT))
;  (slot y (type FLOAT))
;)

(deffacts startup
  (machine (name "M10") (x 1.75) (y 4.8))
  (machine (name "M9") (x 1.4) (y 3.95))
  (machine (name "M8") (x 2.1) (y 1.75))
  (machine (name "M7") (x 2.85) (y 5.45))
  (machine (name "M6") (x 2.5) (y 3.95))
  (machine (name "M5") (x 2.85) (y 3.1))
  (machine (name "M4") (x 2.85) (y 1.4))
  (machine (name "M3") (x 3.4) (y 0.6))
  (machine (name "M2") (x 3.5) (y 3.95))
  (machine (name "M1") (x 3.95) (y 1.4))
  (machine (name "R1") (x 0.5) (y 5.2))
  (machine (name "R2") (x 5.2) (y 0.5))
  (machine (name "D1") (x 5.1) (y 3.2))
  (machine (name "D2") (x 5.1) (y 2.85))
  (machine (name "D3") (x 5.1) (y 2.5))
  (machine (name "T") (x 5.2) (y 5.2))
  (navpoint (name "North West") (x 1.0) (y 4.5))
  (navpoint (name "South") (x 2.9) (y 1.1))
  (navpoint (name "South East") (x 4.8) (y 1.1))
  (navpoint (name "North East") (x 4.8) (y 4.5))
  (navpath (start "South") (goal "South East"))
  (navpath (start "South East") (goal "North East"))
  (navpath (start "North East") (goal "North West"))
  (navpath (start "North West") (goal "South"))
  (blocked "Nothing")
  (position 0.0 0.0)
  (lastmovetime 0.0)
  (lastnavpointtime 0.0)
)
