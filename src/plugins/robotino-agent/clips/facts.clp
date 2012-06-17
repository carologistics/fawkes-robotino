
;---------------------------------------------------------------------------
;  facts.clp - Robotino agent decision testing -- facts
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate goto
  (multislot machines (type STRING))
)

(deftemplate machine
  (slot name (type STRING))
  (slot mtype (type SYMBOL)
        (allowed-values UNKNOWN M1 M2 M3 M1_2 M2_3 DELIVER) (default UNKNOWN))
  (slot egc (type SYMBOL) (allowed-values YES NO) (default NO))
  (multislot loaded-with (type SYMBOL) (allowed-symbols S0 S1 S2))
  (slot junk (type INTEGER) (default 0))
  (slot productions (type INTEGER) (default 0))
)

;(deftemplate holding
;  (slot puck (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P) (default NONE))
;)


(deffacts startup
  (holding NONE)
  (machine (name "deliver") (mtype DELIVER))
  (machine (name "m9"))
  (machine (name "m8"))
  (machine (name "m7"))
  (machine (name "m6"))
  (machine (name "m5"))
  (machine (name "m4"))
  (machine (name "m3"))
  (machine (name "m2"))
  (machine (name "m1"))
  (goto (machines))
  (state WAIT_START)
  (holding NONE)
)
