
;---------------------------------------------------------------------------
;  facts.clp - Robotino agent decision testing -- facts
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate machine
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot mtype (type SYMBOL) (default UNKNOWN)
        (allowed-values UNKNOWN T1 T2 T3 T4 T5 DELIVER IGNORED))
  (multislot loaded-with (type SYMBOL) (allowed-symbols S0 S1 S2))
  (slot junk (type INTEGER) (default 0))
  (slot productions (type INTEGER) (default 0))
  (slot output (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P1 P2 P3))
)

;(deftemplate holding
;  (slot puck (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P) (default NONE))
;)

(deftemplate signal
  (slot type)
  (multislot time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot seq (type INTEGER) (default 1))
  (slot count (type INTEGER) (default 1))
)


(deffacts startup
  (holding NONE)
  (machine (name D1) (mtype DELIVER))
  (machine (name D2) (mtype DELIVER))
  (machine (name D3) (mtype DELIVER))
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
  (state WAIT_START)
  (phase PRE_GAME)
  (refbox-state WAIT_START)

  (signal (type beacon) (time (create$ 0 0)) (seq 1))
)
