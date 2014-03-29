
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

(deftemplate pose
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
)

; EXPLORATION

(deftemplate machine-exploration
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 M11 M12 M13 M14 M15 M16 M17 M18 M19 M20 M21 M22 M23 M24))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (slot light (type SYMBOL) (allowed-values GREEN ORANGE RED OFF) (default OFF))
  (slot look-pos (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 M11 M12 M13 M14 M15 M16 M17 M18 M19 M20 M21 M22 M23 M24 ExpM1 ExpM2 ExpM3 ExpM4 ExpM5 ExpM6 ExpM7 ExpM8 ExpM9 ExpM10 ExpM11 ExpM12 ExpM13 ExpM14 ExpM15 ExpM16 ExpM17 ExpM18 ExpM19 ExpM20 ExpM21 ExpM22 ExpM23 ExpM24 D1 D2 D3 TST R1 R2) (default M1))
  (slot ownership (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot recognized (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot next (type SYMBOL)) ;TODO delete next
)

(deftemplate machine-type
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 M11 M12 M13 M14 M15 M16 M17 M18 M19 M20 M21 M22 M23 M24) (default M1))
  (slot type (type SYMBOL) (allowed-values T1 T2 T3 T4 T5) (default T1))
)

(deftemplate matching-type-light
  (slot type (type SYMBOL) (allowed-values T1 T2 T3 T4 T5 DELIVER TEST RECYCLE) (default TEST))
  (slot red (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot yellow (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
  (slot green (type SYMBOL) (allowed-values ON OFF BLINKING) (default OFF))
)

(deftemplate exp-row
  (slot name (type SYMBOL) (allowed-values HIGH MID LOW))
  (multislot row (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 M11 M12 M13 M14 M15 M16 M17 M18 M19 M20 M21 M22 M23 M24))
)

(deffacts startup-exploration
  (machine-exploration (name M24) (look-pos ExpM24))
  (machine-exploration (name M23) (look-pos ExpM23))
  (machine-exploration (name M22) (look-pos ExpM22))
  (machine-exploration (name M21) (look-pos ExpM21))
  (machine-exploration (name M20) (look-pos ExpM20))
  (machine-exploration (name M19) (look-pos ExpM19))
  (machine-exploration (name M18) (look-pos ExpM18))
  (machine-exploration (name M17) (look-pos ExpM17))
  (machine-exploration (name M16) (look-pos ExpM16))
  (machine-exploration (name M15) (look-pos ExpM15))
  (machine-exploration (name M14) (look-pos ExpM14))
  (machine-exploration (name M13) (look-pos ExpM13))
  (machine-exploration (name M12) (look-pos ExpM12))
  (machine-exploration (name M11) (look-pos ExpM11))
  (machine-exploration (name M10) (look-pos ExpM10))
  (machine-exploration (name M9) (look-pos ExpM9))
  (machine-exploration (name M8) (look-pos ExpM8))
  (machine-exploration (name M7) (look-pos ExpM7))
  (machine-exploration (name M6) (look-pos ExpM6))
  (machine-exploration (name M5) (look-pos ExpM5))
  (machine-exploration (name M4) (look-pos ExpM4))
  (machine-exploration (name M3) (look-pos ExpM3))
  (machine-exploration (name M2) (look-pos ExpM2))
  (machine-exploration (name M1) (look-pos ExpM1))
)


; PRODUCTION

(deftemplate machine
  (slot name (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot mtype (type SYMBOL) (default UNKNOWN)
        (allowed-values UNKNOWN T1 T2 T3 T4 T5 DELIVER RECYCLE IGNORED))
  (multislot loaded-with (type SYMBOL) (allowed-symbols S0 S1 S2))
  (multislot incoming (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO))
  (slot produced-puck (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P1 P2 P3) (default NONE))
  (slot junk (type INTEGER) (default 0))
  (slot productions (type INTEGER) (default 0))
  (slot output (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P1 P2 P3))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (multislot final-prod-time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot ownership (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot produce-blocked (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot recycle-blocked (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot doubtful-worldmodel (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
)

;common template for a task
(deftemplate task
  (slot name (type SYMBOL) (allowed-values load-with-S0 load-with-S1 pick-and-load pick-and-deliver recycle))
  (multislot args (type SYMBOL)) ;in chronological order
  (slot state (type SYMBOL) (allowed-values ordered running finished failed) (default ordered))
  (slot priority (type INTEGER) (default 0))
)

;common template for a proposed task
(deftemplate proposed-task
  (slot name (type SYMBOL) (allowed-values load-with-S0 load-with-S1 pick-and-load pick-and-deliver recycle))
  (multislot args (type SYMBOL)) ;in chronological order
  (slot state (type SYMBOL) (allowed-values proposed asked rejected) (default proposed))
  (slot priority (type INTEGER) (default 0))
)

(deftemplate needed-task-lock
  (slot action (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO))
  (slot place (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 R1 R2))
  (slot resource (type SYMBOL))
)

(deftemplate worldmodel-change
  (slot machine (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 D1 D2 D3 TST R1 R2))
  (slot change (type SYMBOL) (allowed-values ADD_LOADED_WITH REMOVE_LOADED_WITH ADD_INCOMING REMOVE_INCOMING SET_NUM_CO SET_PROD_FINISHED_TIME REMOVE_PRODUCED SET_PRODUCE_BLOCKED SET_RECYCLE_BLOCKED SET_DOUBTFUL_WORLDMODEL))
  (slot value (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO NOTHING S0 S1 S2) (default NOTHING))
  (slot amount (type INTEGER) (default 0))
  (slot already-applied (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (multislot last-sent (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot id (type INTEGER) (default 0)) ;random id
)

(deftemplate wait-for-lock
  (slot res (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values new get use finished) (default new))
  (slot priority (type INTEGER))
)

(deffacts startup-production
  (last-lights)
  (holding NONE)
  (machine (name D1) (mtype DELIVER) (ownership TRUE))
  (machine (name D2) (mtype DELIVER) (ownership TRUE))
  (machine (name D3) (mtype DELIVER) (ownership TRUE))
  (machine (name M1) (ownership TRUE))
  (machine (name M2) (ownership TRUE))
  (machine (name M3) (ownership TRUE))
  (machine (name M4) (ownership TRUE))
  (machine (name M5) (ownership TRUE))
  (machine (name M6) (ownership TRUE))
  (machine (name M7) (ownership TRUE))
  (machine (name M8) (ownership TRUE))
  (machine (name M9) (ownership TRUE))
  (machine (name M10) (ownership TRUE))
  (machine (name R1) (output S0) (mtype RECYCLE) (ownership TRUE))
  (machine (name R2) (output S0) (mtype RECYCLE) (ownership TRUE))
  (state WAIT_START)
  (phase PRE_GAME)
  (refbox-state WAIT_START)
  (game-time (create$ 0 0))
  (game-duration (* 15 60))
  
  (timer (name beacon) (time (create$ 0 0)) (seq 1))
  (timer (name send-worldmodel-sync) (time (create$ 0 0)) (seq 1))

  (already-received-wm-changes (create$))

  (pose (x 0.0) (y 0.0))
  (puck-in-gripper FALSE)
  
  ;TODO: get following data from the refbox
  (own-half right)
  (input-storage Ins1)
)
