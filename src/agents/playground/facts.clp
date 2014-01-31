
;---------------------------------------------------------------------------
;  facts.clp - Robotino agent decision testing -- facts
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; EXPLORATION

; PRODUCTION

(deftemplate machine
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot mtype (type SYMBOL) (default UNKNOWN)
        (allowed-values UNKNOWN T1 T2 T3 T4 T5 DELIVER RECYCLE IGNORED))
  (multislot loaded-with (type SYMBOL) (allowed-symbols S0 S1 S2))
  (slot junk (type INTEGER) (default 0))
  (slot productions (type INTEGER) (default 0))
  (slot output (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P1 P2 P3))
  (slot x (type FLOAT))
  (slot y (type FLOAT))
)

(deffacts startup-production
  (last-lights)
  (holding NONE)
  (machine (name D1) (mtype DELIVER))
  (machine (name D2) (mtype DELIVER))
  (machine (name D3) (mtype DELIVER))
  (machine (name M1) (x 3.9) (y 1.25))
  (machine (name M2) (x 3.5) (y 3.9))
  (machine (name M3) (x 3.3) (y 1.0))
  (machine (name M4) (x 2.75) (y 1.2))
  (machine (name M5) (x 2.75) (y 3.25))
  (machine (name M6) (x 2.3) (y 3.85))
  (machine (name M7) (x 3.25) (y 5.05))
  (machine (name M8) (x 2.05) (y 1.65))
  (machine (name M9) (x 1.25) (y 3.9))
  (machine (name M10) (x 1.68) (y 4.5))
  (machine (name R1) (output S0) (mtype RECYCLE) (x 4.8) (y 0.2))
  (machine (name R2) (output S0) (mtype RECYCLE) (x 0.2) (y 4.8))
)
