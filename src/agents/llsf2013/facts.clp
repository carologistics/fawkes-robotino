
;---------------------------------------------------------------------------
;  facts.clp - Robotino agent decision testing -- facts
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; GENERIC

(deftemplate signal
  (slot type)
  (multislot time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot seq (type INTEGER) (default 1))
  (slot count (type INTEGER) (default 1))
)

(deftemplate active-robot
  (slot name (type SYMBOL) (allowed-values R1 R2))
  (multislot last-seen (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
)


; EXPLORATION

(deftemplate machine-exploration
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2) (default M1))
  (slot x (type FLOAT))
  (slot y (type FLOAT))
  (slot light (type SYMBOL) (allowed-values GREEN ORANGE RED OFF) (default OFF))
  (slot next (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2) (default M1))
  (slot look-pos (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 ExpM1 ExpM2 ExpM3 ExpM4 ExpM5 ExpM6 ExpM7 ExpM8 ExpM9 ExpM10 D1 D2 D3 TST R1 R2) (default M1))
)

(deftemplate machine-light
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2) (default M1))
  (slot red (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot yellow (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot green (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
)

(deftemplate machine-type
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10) (default M1))
  (slot type (type SYMBOL) (allowed-values T1 T2 T3 T4 T5) (default T1))
)

(deftemplate matching-type-light
  (slot type (type SYMBOL) (allowed-values T1 T2 T3 T4 T5 DELIVER TEST RECYCLE) (default TEST))
  (slot red (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot yellow (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot green (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
)

(deffacts startup-exploration
  (active-robot (name R1))
  (active-robot (name R2))

  (machine-exploration (name M10) (x 2.18) (y 4.74) (next M9) (look-pos M10))
  (machine-exploration (name M9) (x 1.38) (y 3.42) (next M8) (look-pos M9))
  (machine-exploration (name M8) (x 1.38) (y 2.18) (next M3) (look-pos M8))
  (machine-exploration (name M7) (x 2.5) (y 4.5) (next M6) (look-pos M7))
  (machine-exploration (name M6) (x 3.1) (y 4.42) (next M10) (look-pos M6))
  (machine-exploration (name M5) (x 2.3) (y 3.1) (next M2) (look-pos M5))
  (machine-exploration (name M4) (x 3.1) (y 2.13) (next M1) (look-pos M4))
  (machine-exploration (name M3) (x 3.1) (y 1.06) (next M4) (look-pos M3))
  (machine-exploration (name M2) (x 4.42) (y 3.62) (next M7) (look-pos M2))
  (machine-exploration (name M1) (x 3.62) (y 1.18) (next M5) (look-pos M1))
)


; PRODUCTION

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

(deffacts startup-production
  (last-lights)
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
