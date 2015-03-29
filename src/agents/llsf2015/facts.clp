;---------------------------------------------------------------------------
;  facts.clp - Robotino agent decision testing -- facts
;
;  Created: Sat Jun 16 12:37:44 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; GENERIC
(deftemplate active-robot
  (slot name (type SYMBOL) (allowed-values R-1 R-2 R-3))
  (multislot last-seen (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
)

(deftemplate pose
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
)

; EXPLORATION

(deftemplate zone-exploration
  (slot name (type SYMBOL) (allowed-values Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12
					   Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24))
  (slot team (type SYMBOL) (allowed-symbols nil CYAN MAGENTA))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (slot light (type SYMBOL) (allowed-values GREEN ORANGE RED OFF) (default OFF))
  ; list of positions where to search for tags
  (multislot look-pos (type SYMBOL) (default (create$)))
  ; index of the next lookpos to use
  (slot current-look-pos (type INTEGER) (default 1))
  (slot recognized (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot next (type SYMBOL)) ;TODO delete next
  (slot machine (type SYMBOL) (allowed-values CBS CCS1 CCS2 CRS1 CRS2 CDS MBS MCS1 MCS2 MRS1 MRS2 MDS))
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
  (zone-exploration (name Z24) (look-pos (create$ ExpZ24_1 ExpZ24_2)))
  (zone-exploration (name Z23) (look-pos (create$ ExpZ23_1 ExpZ23_2)))
  (zone-exploration (name Z22) (look-pos (create$ ExpZ22_1 ExpZ22_2)))
  (zone-exploration (name Z21) (look-pos (create$ ExpZ21_1 ExpZ21_2)))
  (zone-exploration (name Z20) (look-pos (create$ ExpZ20_1 ExpZ20_2)))
  (zone-exploration (name Z19) (look-pos (create$ ExpZ19_1 ExpZ19_2)))
  (zone-exploration (name Z18) (look-pos (create$ ExpZ18_1 ExpZ18_2)))
  (zone-exploration (name Z17) (look-pos (create$ ExpZ17_1 ExpZ17_2)))
  (zone-exploration (name Z16) (look-pos (create$ ExpZ16_1 ExpZ16_2)))
  (zone-exploration (name Z15) (look-pos (create$ ExpZ15_1 ExpZ15_2)))
  (zone-exploration (name Z14) (look-pos (create$ ExpZ14_1 ExpZ14_2)))
  (zone-exploration (name Z13) (look-pos (create$ ExpZ13_1 ExpZ13_2)))
  (zone-exploration (name Z12) (look-pos (create$ ExpZ12_1 ExpZ12_2)))
  (zone-exploration (name Z11) (look-pos (create$ ExpZ11_1 ExpZ11_2)))
  (zone-exploration (name Z10) (look-pos (create$ ExpZ10_1 ExpZ10_2)))
  (zone-exploration (name Z9) (look-pos (create$ ExpZ9_1 ExpZ9_2)))
  (zone-exploration (name Z8) (look-pos (create$ ExpZ8_1 ExpZ8_2)))
  (zone-exploration (name Z7) (look-pos (create$ ExpZ7_1 ExpZ7_2)))
  (zone-exploration (name Z6) (look-pos (create$ ExpZ6_1 ExpZ6_2)))
  (zone-exploration (name Z5) (look-pos (create$ ExpZ5_1 ExpZ5_2)))
  (zone-exploration (name Z4) (look-pos (create$ ExpZ4_1 ExpZ4_2)))
  (zone-exploration (name Z3) (look-pos (create$ ExpZ3_1 ExpZ3_2)))
  (zone-exploration (name Z2) (look-pos (create$ ExpZ2_1 ExpZ2_2)))
  (zone-exploration (name Z1) (look-pos (create$ ExpZ1_1 ExpZ1_2)))
)


; PRODUCTION

(deftemplate machine
  (slot name (type SYMBOL) (allowed-values CBS CCS1 CCS2 CRS1 CRS2 CDS MBS MCS1 MCS2 MRS1 MRS2 MDS))
  (slot team (type SYMBOL) (allowed-symbols nil CYAN MAGENTA))
  (slot mtype (type SYMBOL) (default UNKNOWN)
        (allowed-values UNKNOWN T1 T2 T3 T4 T5 DELIVER RECYCLE IGNORED))
  (multislot loaded-with (type SYMBOL) (allowed-symbols S0 S1 S2))
  (multislot incoming (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO))
  (multislot incoming-agent (type SYMBOL)) ;the agent bringing/getting the thing specified in incoming
  (slot produced-puck (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P1 P2 P3) (default NONE))
  (slot fails (type INTEGER) (default 0))
  (slot junk (type INTEGER) (default 0))
  (slot productions (type INTEGER) (default 0))
  (slot output (type SYMBOL) (allowed-symbols NONE S0 S1 S2 P1 P2 P3))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (multislot final-prod-time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot produce-blocked (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot recycle-blocked (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot doubtful-worldmodel (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot priority (type INTEGER) (default 0))
  (multislot out-of-order-until (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
)

(deftemplate tag-matching
  (slot machine (type SYMBOL) (allowed-values CBS CCS1 CCS2 CRS1 CRS2 CDS MBS MCS1 MCS2 MRS1 MRS2 MDS))
  (slot side (type SYMBOL) (allowed-values INPUT OUTPUT))
  (slot tag-id (type INTEGER))
  (slot team (type SYMBOL) (allowed-symbols CYAN MAGENTA))
)

(deftemplate order
  (slot id (type INTEGER))
  (slot product (type SYMBOL) (allowed-symbols P1 P2 P3))
  (slot quantity-requested (type INTEGER))
  (slot quantity-delivered (type INTEGER))
  (slot begin (type INTEGER))
  (slot end (type INTEGER))
  (slot in-production (type INTEGER) (default 0))
  (slot in-delivery (type INTEGER) (default 0))
)

; Common template for an abstract task which consists of a sequence of steps
(deftemplate task
  (slot id (type INTEGER))
  (slot name (type SYMBOL) (allowed-values produce-p3-and-deliver load-with-S0 load-with-S1 load-with-S2 pick-and-load pick-and-deliver recycle deliver recycle-holding just-in-time-P3 pick-and-store get-stored-and-deliver store produce-with-S0))
  (slot state (type SYMBOL) (allowed-values proposed asked rejected ordered running finished failed) (default proposed))
  (slot priority (type INTEGER) (default 0))
  ;a task consists of multiple steps
  (slot current-step (type INTEGER) (default 0))
  (multislot steps (type INTEGER)) ;in chronological order refers to the ids of the steps
)

; Template for a step
; A step is the building block of a task and usually corresponds to an elementary action
; that has to be performed (skill-call)
; The id has to be in the step sequence of the task
; The arguments of a specific step are optional and used when required
(deftemplate step
  (slot id (type INTEGER))
  (slot name (type SYMBOL) (allowed-values get-s0 produce-at-some-t1 load-machine produce-at deliver recycle get-consumed store get-from-storage))
  (slot state (type SYMBOL) (allowed-values inactive wait-for-activation running finished failed) (default inactive))
  ;optional arguments of a step
  (slot task-priority (type INTEGER))
  (slot machine (type SYMBOL))
  (slot product-type (type SYMBOL))
)

; Needed locks for a task which guarantee that no other robot tries to accomplish the same goal by doing some task
; e.g. bring-intermediate-product to machine-x so no other robot also brings such a intermediate-product to the machine
; task-id has to correspont to the task
(deftemplate needed-task-lock
  (slot task-id (type INTEGER))
  (slot action (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO BRING_P1 BRING_P2 BRING_P3 STORE_PUCK GET_STORED_PUCK))
  (slot place (type SYMBOL) (allowed-values M1 M2 M3 M4 M5 M6 M7 M8 M9 M10 M11 M12 M13 M14 M15 M16 M17 M18 M19 M20 M21 M22 M23 M24 R1 R2 DELIVER))
  (slot resource (type SYMBOL) (default NONE))
)

(deftemplate worldmodel-change
  (slot machine (type SYMBOL) (default NONE)); or puck-storage
  (slot order (type INTEGER) (default 0))
  (slot change (type SYMBOL) (allowed-values ADD_LOADED_WITH REMOVE_LOADED_WITH ADD_INCOMING REMOVE_INCOMING SET_NUM_CO SET_PROD_FINISHED_TIME REMOVE_PRODUCED SET_PRODUCE_BLOCKED RESET_PRODUCE_BLOCKED SET_RECYCLE_BLOCKED SET_DOUBTFUL_WORLDMODEL SET_IN_DELIVERY SET_OUT_OF_ORDER_UNTIL STORE_PUCK GET_STORED_PUCK))
  (slot value (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO NOTHING S0 S1 S2 P1 P2 P3) (default NOTHING))
  (slot amount (type INTEGER) (default 0))
  (slot already-applied (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (multislot last-sent (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot id (type INTEGER) (default 0)) ;random id
  (slot agent (type SYMBOL) (default DEFAULT))
)

(deftemplate wait-for-lock
  (slot res (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values new get use finished) (default new))
  (slot priority (type INTEGER) (default 0))
)

(deftemplate skill-to-execute
  (slot skill (type SYMBOL))
  (multislot args (type SYMBOL) (default (create$)))
  (slot state (type SYMBOL) (allowed-symbols wait-for-lock running final failed) (default wait-for-lock))
  (slot target (type SYMBOL))
)

(deftemplate puck-storage
  (slot name (type SYMBOL))
  (slot puck (type SYMBOL) (allowed-symbols NONE P1 P2 P3) (default NONE))
  (slot team (type SYMBOL) (allowed-symbols CYAN MAGENTA))
  (multislot incoming (type SYMBOL) (allowed-symbols STORE_PUCK GET_STORED_PUCK))
  (multislot incoming-agent (type SYMBOL))
)

(deffacts startup-facts
  (team-color nil)
  (points-magenta 0)
  (points-cyan 0)
  (last-lights)
  (holding NONE)

  (machine (name CBS) (team CYAN))
  (machine (name CCS1) (team CYAN))
  (machine (name CCS2) (team CYAN))
  (machine (name CRS1) (team CYAN))
  (machine (name CRS2) (team CYAN))
  (machine (name CDS) (team CYAN))
  (machine (name MBS) (team MAGENTA))
  (machine (name MCS1) (team MAGENTA))
  (machine (name MCS2) (team MAGENTA))
  (machine (name MRS1) (team MAGENTA))
  (machine (name MRS2) (team MAGENTA))
  (machine (name MDS) (team MAGENTA))

  (tag-matching (machine CBS) (side INPUT) (team CYAN) (tag-id 65))
  (tag-matching (machine CCS1) (side INPUT) (team CYAN) (tag-id 1))
  (tag-matching (machine CCS2) (side INPUT) (team CYAN) (tag-id 17))
  (tag-matching (machine CRS1) (side INPUT) (team CYAN) (tag-id 33))
  (tag-matching (machine CRS2) (side INPUT) (team CYAN) (tag-id 177))
  (tag-matching (machine CDS) (side INPUT) (team CYAN) (tag-id 81))
  (tag-matching (machine CBS) (side OUTPUT) (team CYAN) (tag-id 66))
  (tag-matching (machine CCS1) (side OUTPUT) (team CYAN) (tag-id 2))
  (tag-matching (machine CCS2) (side OUTPUT) (team CYAN) (tag-id 18))
  (tag-matching (machine CRS1) (side OUTPUT) (team CYAN) (tag-id 34))
  (tag-matching (machine CRS2) (side OUTPUT) (team CYAN) (tag-id 178))
  (tag-matching (machine CDS) (side OUTPUT) (team CYAN) (tag-id 82))

  (tag-matching (machine MBS) (side INPUT) (team MAGENTA) (tag-id 161))
  (tag-matching (machine MCS1) (side INPUT) (team MAGENTA) (tag-id 97))
  (tag-matching (machine MCS2) (side INPUT) (team MAGENTA) (tag-id 113))
  (tag-matching (machine MRS1) (side INPUT) (team MAGENTA) (tag-id 129))
  (tag-matching (machine MRS2) (side INPUT) (team MAGENTA) (tag-id 145))
  (tag-matching (machine MDS) (side INPUT) (team MAGENTA) (tag-id 49))
  (tag-matching (machine MBS) (side OUTPUT) (team MAGENTA) (tag-id 162))
  (tag-matching (machine MCS1) (side OUTPUT) (team MAGENTA) (tag-id 98))
  (tag-matching (machine MCS2) (side OUTPUT) (team MAGENTA) (tag-id 114))
  (tag-matching (machine MRS1) (side OUTPUT) (team MAGENTA) (tag-id 130))
  (tag-matching (machine MRS2) (side OUTPUT) (team MAGENTA) (tag-id 146))
  (tag-matching (machine MDS) (side OUTPUT) (team MAGENTA) (tag-id 50))

  (state WAIT_START)
  (phase PRE_GAME)
  (refbox-state WAIT_START)
  (game-time (create$ 0 0))
  (game-duration (* 15 60))
  
  (timer (name beacon) (time (create$ 0 0)) (seq 1))
  (timer (name exploration-finished) (time (create$ 0 0)) (seq 1))
  (timer (name send-worldmodel-sync) (time (create$ 0 0)) (seq 1))

  (already-received-wm-changes (create$))

  (pose (x 0.0) (y 0.0))
  (puck-in-gripper FALSE)
  
  ; Input storage per team color
  (input-storage CYAN Ins1 0 0)
  (input-storage MAGENTA Ins2 0 0)
  (secondary-storage CYAN Ins1Sec 0 0)
  (secondary-storage MAGENTA Ins2Sec 0 0)
  (deliver CYAN deliver1 0 0)
  (deliver MAGENTA deliver2 0 0)
)
