
;---------------------------------------------------------------------------
;  facts.clp - Robotino agent decision testing -- facts
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate goto
  (slot min-prio (type INTEGER) (default 0))
  (multislot machines (type STRING))
)

(deftemplate machine
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot mtype (type SYMBOL) (default UNKNOWN)
        (allowed-values UNKNOWN M1 M2 M3 M1_2 M2_3 DELIVER M1_EXPRESS IGNORED))
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
  (machine (name D1) (mtype DELIVER))
  (machine (name M1))
  (machine (name M2))
  (machine (name M3))
  (machine (name M4))
  (machine (name M5))
  (machine (name M6))
  (machine (name M7))
  (machine (name M8))
  (machine (name M9))
  (machine (name M10))
  (goto (machines))
  (state WAIT_START)
  (holding NONE)
)
