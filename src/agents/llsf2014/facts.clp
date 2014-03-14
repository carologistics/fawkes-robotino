
;---------------------------------------------------------------------------
;  facts.clp - Robotino agent decision testing -- facts
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; GENERIC
(deftemplate active-robot
  (slot name (type SYMBOL) (allowed-values R-1 R-2))
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
  (machine-exploration (name M10) (x 2.18) (y 4.74) (next M9) (look-pos ExpM10))
  (machine-exploration (name M9) (x 1.38) (y 3.42) (next M8) (look-pos ExpM9))
  (machine-exploration (name M8) (x 1.38) (y 2.18) (next M3) (look-pos ExpM8))
  (machine-exploration (name M7) (x 2.5) (y 4.5) (next M6) (look-pos ExpM7))
  (machine-exploration (name M6) (x 3.1) (y 4.42) (next M10) (look-pos ExpM6))
  (machine-exploration (name M5) (x 2.3) (y 3.1) (next M2) (look-pos ExpM5))
  (machine-exploration (name M4) (x 3.1) (y 2.13) (next M1) (look-pos ExpM4))
  (machine-exploration (name M3) (x 3.1) (y 1.06) (next M4) (look-pos ExpM3))
  (machine-exploration (name M2) (x 4.42) (y 3.62) (next M7) (look-pos ExpM2))
  (machine-exploration (name M1) (x 3.62) (y 1.18) (next M5) (look-pos ExpM1))
)


; PRODUCTION

(deftemplate machine
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot mtype (type SYMBOL) (default UNKNOWN)
        (allowed-values UNKNOWN T1 T2 T3 T4 T5 DELIVER RECYCLE IGNORED))
  (multislot loaded-with (type SYMBOL) (allowed-symbols S0 S1 S2))
  (multislot incoming (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO))
  (slot junk (type INTEGER) (default 0))
  (slot productions (type INTEGER) (default 0))
  (slot output (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P1 P2 P3))
  (slot x (type FLOAT))
  (slot y (type FLOAT))
)

;common template for a task
(deftemplate task
  (slot name (type SYMBOL) (allowed-values load-with-S0 load-with-S1 pick-and-load pick-and-deliver recycle-and-load-with-S0 recycle-and-load-with-T1))
  (multislot args (type SYMBOL)) ;in chronological order
  (slot state (type SYMBOL) (allowed-values ordered running finished) (default ordered))
)

;common template for a proposed task
(deftemplate proposed-task
  (slot name (type SYMBOL) (allowed-values load-with-S0 load-with-S1 pick-and-load pick-and-deliver recycle-and-load-with-S0 recycle-and-load-with-T1))
  (multislot args (type SYMBOL)) ;in chronological order
  (slot state (type SYMBOL) (allowed-values proposed asked rejected) (default proposed))
)

(deftemplate needed-task-lock
  (slot action (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO))
  (slot place (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 R1 R2))
  (slot resource (type SYMBOL))
)

(deftemplate worldmodel-change
  (slot machine (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot change (type SYMBOL) (allowed-values ADD_LOADED_WITH REMOVE_LOADED_WITH ADD_INCOMING REMOVE_INCOMING SET_NUM_CO))
  (slot value (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO NOTHING) (default NOTHING))
  (slot amount (type INTEGER) (default 0))
  (slot already-applied (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
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
  (state WAIT_START)
  (phase PRE_GAME)
  (refbox-state WAIT_START)
  (game-time (create$ 0 0))
  (game-duration (* 15 60))
  
  (timer (name beacon) (time (create$ 0 0)) (seq 1))
  (timer (name send-worldmodel-sync) (time (create$ 0 0)) (seq 1))
)
