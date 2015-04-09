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
  ; id to refer it in other facts
  (slot id (type INTEGER) (default 0))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (slot ori (type FLOAT) (default 0.0))
)

; EXPLORATION

(deftemplate zone-exploration
  (slot name (type SYMBOL) (allowed-values Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12
					   Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24))
  (slot team (type SYMBOL) (allowed-symbols nil CYAN MAGENTA))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (slot light (type SYMBOL) (allowed-values GREEN ORANGE RED OFF) (default OFF))
  ; list of positions where to search for tags (ids to pose facts)
  (multislot look-pos (type INTEGER) (default (create$)))
  ; index of the next lookpos to use
  (slot current-look-pos (type INTEGER) (default 1))
  (slot recognized (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot still-to-explore (type SYMBOL) (allowed-symbols TRUE FALSE) (default TRUE))
  (slot next (type SYMBOL)) ;TODO delete next
  (slot machine (type SYMBOL) (allowed-values CBS CCS1 CCS2 CRS1 CRS2 CDS MBS MCS1 MCS2 MRS1 MRS2 MDS))
)

(deftemplate exploration-result
  (slot machine (type SYMBOL) (allowed-symbols CBS CCS1 CCS2 CRS1 CRS2 CDS MBS MCS1 MCS2 MRS1 MRS2 MDS))
  (slot zone (type SYMBOL) (allowed-symbols Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12
					   Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24))
  (slot red (type SYMBOL) (allowed-symbols ON OFF BLINKING))
  (slot yellow (type SYMBOL) (allowed-symbols ON OFF BLINKING))
  (slot green (type SYMBOL) (allowed-symbols ON OFF BLINKING))
)

(deftemplate exp-row
  (slot name (type SYMBOL) (allowed-values HIGH MID LOW))
  (multislot row (type SYMBOL) (allowed-symbols Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12 Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24))
)

(deffacts startup-exploration
  (zone-exploration (name Z24))
  (zone-exploration (name Z23))
  (zone-exploration (name Z22))
  (zone-exploration (name Z21))
  (zone-exploration (name Z20))
  (zone-exploration (name Z19))
  (zone-exploration (name Z18))
  (zone-exploration (name Z17))
  (zone-exploration (name Z16))
  (zone-exploration (name Z15))
  (zone-exploration (name Z14))
  (zone-exploration (name Z13))
  (zone-exploration (name Z12))
  (zone-exploration (name Z11))
  (zone-exploration (name Z10))
  (zone-exploration (name Z9))
  (zone-exploration (name Z8))
  (zone-exploration (name Z7))
  (zone-exploration (name Z6))
  (zone-exploration (name Z5))
  (zone-exploration (name Z4))
  (zone-exploration (name Z3))
  (zone-exploration (name Z2))
  (zone-exploration (name Z1))
)


; PRODUCTION

(deftemplate machine
  (slot name (type SYMBOL) (allowed-values CBS CCS1 CCS2 CRS1 CRS2 CDS MBS MCS1 MCS2 MRS1 MRS2 MDS))
  (slot team (type SYMBOL) (allowed-symbols nil CYAN MAGENTA))
  (slot mtype (type SYMBOL) (allowed-values BS DS RS CS))
  (multislot incoming (type SYMBOL) (allowed-symbols BRING_S0 BRING_S1 BRING_S2 PICK_PROD PICK_CO))
  (multislot incoming-agent (type SYMBOL) (default (create$))) ;the agent bringing/getting the thing specified in incoming
  ;id of the loaded-puck
  (slot loaded-id (type INTEGER) (default 0))
  ;id of the produced-puck
  (slot produced-id (type INTEGER) (default 0))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (multislot final-prod-time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  ; (slot priority (type INTEGER) (default 0))
  (multislot out-of-order-until (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
)

; (deftemplate base-station 
;   (slot name (type SYMBOL) (allowed-symbols CBS MBS))
; )

; (deftemplate delivery-station 
;   (slot name (type SYMBOL) (allowed-symbols CDS MDS))
; )

(deftemplate cap-station 
  (slot name (type SYMBOL) (allowed-symbols CCS1 CCS2 MCS1 MCS2))
  (slot cap-loaded (type SYMBOL) (allowed-symbols NONE GREY BLACK) (default NONE))
  ;the team has to fill one CS with black and the other with grey caps (config)
  (slot assigned-cap-color (type SYMBOL) (allowed-symbols NONE GREY BLACK) (default NONE))
)

(deftemplate ring-station
  (slot name (type SYMBOL) (allowed-symbols CRS1 CRS2 MRS1 MRS2))
  (multislot available-colors (type SYMBOL) (allowed-symbols BLUE GREEN YELLOW ORANGE))
  (slot selected-color (type SYMBOL) (allowed-symbols NONE BLUE GREEN YELLOW ORANGE)
	(default NONE))
  (slot bases-needed (type INTEGER) (allowed-values 0 1 2) (default 0))
)


(deftemplate tag-matching
  (slot machine (type SYMBOL) (allowed-values CBS CCS1 CCS2 CRS1 CRS2 CDS MBS MCS1 MCS2 MRS1 MRS2 MDS))
  (slot side (type SYMBOL) (allowed-values INPUT OUTPUT))
  (slot tag-id (type INTEGER))
  (slot team (type SYMBOL) (allowed-symbols CYAN MAGENTA))
)

(deftemplate product
  ;id to link it in other facts
  (slot id (type INTEGER))
  (multislot rings (type SYMBOL) (allowed-symbols BLUE GREEN YELLOW ORANGE)
	     (default (create$ )))
  (slot cap (type SYMBOL) (allowed-symbols NONE GREY BLACK) (default NONE))
  ; is the base from a cap-station and therefore unusable
  (slot base-usable (type SYMBOL) (allowed-symbols TRUE FALSE) (default TRUE))
)

(deftemplate order
  (slot id (type INTEGER))
  ;id of product fact how the product should look like
  (slot product-id (type INTEGER))
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
  (slot name (type SYMBOL) (allowed-symbols fill-cap))
  (slot state (type SYMBOL) (allowed-symbols proposed asked rejected ordered running finished failed) (default proposed))
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
  (slot name (type SYMBOL) (allowed-symbols get-from-shelf insert get-output))
  (slot state (type SYMBOL) (allowed-symbols inactive wait-for-activation running finished failed) (default inactive))
  ;optional arguments of a step
  (slot task-priority (type INTEGER))
  (slot machine (type SYMBOL))
  (slot product-type (type SYMBOL))
  (slot shelf-slot (type SYMBOL) (allowed-symbols LEFT MIDDLE RIGHT))
)

; Needed locks for a task which guarantee that no other robot tries to accomplish the same goal by doing some task
; e.g. bring-intermediate-product to machine-x so no other robot also brings such a intermediate-product to the machine
; task-id has to correspont to the task
(deftemplate needed-task-lock
  (slot task-id (type INTEGER))
  (slot action (type SYMBOL))
  (slot place (type SYMBOL))
  (slot resource (type SYMBOL) (default NONE))
)

(deftemplate worldmodel-change
  (slot machine (type SYMBOL) (default NONE)); or puck-storage
  (slot order (type INTEGER) (default 0))
  (slot change (type SYMBOL))
  (slot value (type SYMBOL) (default NOTHING))
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

  (machine (name CBS) (team CYAN) (mtype BS))
  (machine (name CCS1) (team CYAN) (mtype CS))
  (cap-station (name CCS1))
  (machine (name CCS2) (team CYAN) (mtype CS))
  (cap-station (name CCS2))
  (machine (name CRS1) (team CYAN) (mtype RS))
  (ring-station (name CRS1))
  (machine (name CRS2) (team CYAN) (mtype RS))
  (ring-station (name CRS2))
  (machine (name CDS) (team CYAN) (mtype DS))

  (machine (name MBS) (team MAGENTA) (mtype BS))
  (machine (name MCS1) (team MAGENTA) (mtype CS))
  (cap-station (name MCS1))
  (machine (name MCS2) (team MAGENTA) (mtype CS))
  (cap-station (name MCS2))
  (machine (name MRS1) (team MAGENTA) (mtype RS))
  (ring-station (name MRS1))
  (machine (name MRS2) (team MAGENTA) (mtype RS))
  (ring-station (name MRS2))
  (machine (name MDS) (team MAGENTA) (mtype DS))

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
