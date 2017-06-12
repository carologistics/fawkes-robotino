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
  (slot name (type STRING) (default ""))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (slot ori (type FLOAT) (default 0.0))
)

; EXPLORATION

(deftemplate zone-exploration
  (slot name (type SYMBOL) (allowed-values Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12
					   Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24))
  (slot machine (type SYMBOL) (allowed-symbols C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS UNKNOWN) (default UNKNOWN))
  (slot team (type SYMBOL) (allowed-symbols nil CYAN MAGENTA))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  ; list of positions where to search for tags (ids to pose facts)
  (multislot look-pos (type INTEGER) (default (create$)))
  ; index of the next lookpos to use
  (slot current-look-pos (type INTEGER) (default 1))
  (slot recognized (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot still-to-explore (type SYMBOL) (allowed-symbols TRUE FALSE) (default TRUE))
  (slot next (type SYMBOL)) ;TODO delete next

  ; for exploration-catch-up in produciton
  (multislot incoming (type SYMBOL) (default (create$)))  
  (multislot incoming-agent (type SYMBOL) (default (create$)))
  (slot times-searched (type INTEGER) (default 0))
  (slot sync-id (type INTEGER) (default 0))
)

(deftemplate exp-matching
  (slot red (type SYMBOL) (allowed-symbols ON OFF))
  (slot yellow (type SYMBOL) (allowed-symbols ON OFF))
  (slot green (type SYMBOL) (allowed-symbols ON OFF))
  (slot mtype (type STRING))
  (slot found (type SYMBOL) (allowed-symbols TRUE MAYBE FALSE) (default FALSE))
  (slot machine (type SYMBOL) (allowed-symbols C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS
                                               M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS NONE) (default NONE))
  (slot sync-id (type INTEGER) (default 0))
)

(deftemplate exploration-result
  (slot machine (type SYMBOL) (allowed-symbols C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS))
  (slot zone (type SYMBOL) (allowed-symbols Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12
					   Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24))
  (slot mtype (type STRING) (default ""))
)

(deftemplate exp-row
  (slot name (type SYMBOL) (allowed-values HIGH MID LOW))
  (multislot row (type SYMBOL) (allowed-symbols Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12 Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24))
)

(deftemplate found-tag
  (slot name (type SYMBOL))
  (slot side (type SYMBOL) (allowed-values INPUT OUTPUT))
  (slot frame (type STRING))
  (multislot trans (type FLOAT) (cardinality 3 3))
  (multislot rot (type FLOAT) (cardinality 4 4))
  (slot sync-id (type INTEGER) (default 0))
)

(deftemplate navgraph-added-for-mps
  (slot name (type SYMBOL))
)

(deftemplate last-navgraph-compute-msg 
  (slot id (type INTEGER))
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
  (exp-nearing-tag FALSE)
)


; PRODUCTION

(deftemplate machine
  (slot name (type SYMBOL) (allowed-values C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS))
  (slot team (type SYMBOL) (allowed-symbols nil CYAN MAGENTA))
  (slot mtype (type SYMBOL) (allowed-values BS DS RS CS))
  (multislot incoming (type SYMBOL))
  (multislot incoming-agent (type SYMBOL) (default (create$))) ;the agent bringing/getting the thing specified in incoming
  ;id of the loaded-puck
  (slot loaded-id (type INTEGER) (default 0))
  ;id of the produced-puck
  (slot produced-id (type INTEGER) (default 0))
  (slot x (type FLOAT) (default 0.0))
  (slot y (type FLOAT) (default 0.0))
  (multislot final-prod-time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot state (type SYMBOL) (allowed-values IDLE BROKEN PREPARED PROCESSING
					    PROCESSED READY-AT-OUTPUT WAIT-IDLE DOWN))
  (slot sync-id (type INTEGER) (default 0))
)

(deftemplate base-station
  (slot name (type SYMBOL) (allowed-symbols C-BS M-BS))
  (slot active-side (type SYMBOL) (allowed-symbols INPUT OUTPUT) (default INPUT))
  (slot fail-side (type SYMBOL) (allowed-symbols INPUT OUTPUT NONE) (default NONE))
  (slot sync-id (type INTEGER) (default 0))
)

; (deftemplate delivery-station 
;   (slot name (type SYMBOL) (allowed-symbols C-DS M-DS))
; )

(deftemplate cap-station 
  (slot name (type SYMBOL) (allowed-symbols C-CS1 C-CS2 M-CS1 M-CS2))
  (slot cap-loaded (type SYMBOL) (allowed-symbols NONE GREY BLACK) (default NONE))
  ;the team has to fill one CS with black and the other with grey caps (config)
  (slot assigned-cap-color (type SYMBOL) (allowed-symbols NONE GREY BLACK) (default NONE))
  (slot caps-on-shelf (type INTEGER) (default 3))
  (slot sync-id (type INTEGER) (default 0))
)

(deftemplate ring-station
  (slot name (type SYMBOL) (allowed-symbols C-RS1 C-RS2 M-RS1 M-RS2))
  (multislot availablecolors (type SYMBOL) (allowed-symbols BLUE GREEN YELLOW ORANGE))
  (slot selected-color (type SYMBOL) (allowed-symbols NONE BLUE GREEN YELLOW ORANGE)
	(default NONE))
  (slot bases-loaded (type INTEGER) (allowed-values 0 1 2 3) (default 0))
  (slot sync-id (type INTEGER) (default 0))
)


(deftemplate tag-matching
  (slot machine (type SYMBOL) (allowed-values C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS))
  (slot side (type SYMBOL) (allowed-values INPUT OUTPUT))
  (slot tag-id (type INTEGER))
  (slot team (type SYMBOL) (allowed-symbols CYAN MAGENTA))
)

(deftemplate product
  ;id to link it in other facts
  (slot id (type INTEGER))
  (slot product-id (type INTEGER) (default 0))
  (multislot rings (type SYMBOL) (allowed-symbols BLUE GREEN YELLOW ORANGE)
	     (default (create$ )))
  (slot cap (type SYMBOL) (allowed-symbols NONE GREY BLACK) (default NONE))
  (slot base (type SYMBOL) (allowed-symbols BLACK SILVER RED UNKNOWN) (default UNKNOWN))
  ; is the base from a cap-station and therefore unusable
  ; (slot base-usable (type SYMBOL) (allowed-symbols TRUE FALSE) (default TRUE))
  (slot sync-id (type INTEGER) (default 0))
)

(deftemplate ring
  (slot color (type SYMBOL) (allowed-values BLUE GREEN ORANGE YELLOW))
  (slot req-bases (type INTEGER) (default 0))
)

(deftemplate order
  (slot id (type INTEGER))
  ;id of product fact how the product should look like
  (slot product-id (type INTEGER))
  (slot complexity (type SYMBOL) (allowed-symbols C0 C1 C2 C3))
  (slot delivery-gate (type INTEGER))
  (slot quantity-requested (type INTEGER))
  (slot quantity-delivered (type INTEGER) (default 0))
  (slot begin (type INTEGER))
  (slot end (type INTEGER))
  (slot in-production (type INTEGER) (default 0))
  (slot in-delivery (type INTEGER) (default 0))
  (slot sync-id (type INTEGER) (default 0))
)

; Template for a planned task in form of an STN representation
(deftemplate stn-action
  (slot id (type INTEGER))
  (slot name (type SYMBOL) (allowed-symbols move-to-position-empty move-to-position-holding pick-cc-from-shelf load-cs pick-wp-from-cs prod-at-cs pick-wp-from-bs pick-wp-from-rs add-base-to-slide-one add-base-to-slide-two add-base-to-slide-three reset-rs-slide add-ring-one add-ring-two add-ring-three lock-position unlock-position lock-change discard-wp deliver-c0 deliver-c1 deliver-c2 deliver-c3))
  (slot state (type SYMBOL) (allowed-symbols pending running finished))
  (slot duration (type INTEGER))
  (multislot cond-actions (type INTEGER))
  (multislot opts (type SYMBOL))
  (slot active-robot (type SYMBOL) (allowed-symbols R-1 R-2 R-3 R-4 R-5 R-6 NONE) (default NONE))
  (slot sync-id (type INTEGER) (default 0))
)

(deftemplate stn-sync
  (slot state (type SYMBOL) (allowed-symbols generated))
  (slot sync-id (type INTEGER) (default 0))
)

; Common template for an abstract task which consists of a sequence of steps
(deftemplate task
  (slot id (type INTEGER))
  (slot name (type SYMBOL) (allowed-symbols move-to-position-empty move-to-position-holding pick-cc-from-shelf load-cs pick-wp-from-cs discard-wp pick-wp-from-bs prod-at-cs deliver-c0 fill-cap produce-c0 produce-cx add-first-ring add-additional-ring deliver fill-rs discard-unknown exploration-catch-up clear-bs clear-cs clear-rs))
  (slot state (type SYMBOL) (allowed-symbols planned proposed asked rejected ordered running finished failed)
        (default proposed))
  (slot stn-action (type INTEGER))
  (slot priority (type INTEGER) (default 0))
  ;a task consists of multiple steps
  (multislot predecessor (type INTEGER))
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
  (slot name (type SYMBOL) (allowed-symbols get-from-shelf insert get-output get-base find-tag instruct-mps discard
                                            drive-to wait-for-rs wait-for-output acquire-lock release-lock))
  (slot state (type SYMBOL) (allowed-symbols inactive wait-for-activation running finished failed) (default inactive))
  ;optional arguments of a step
  (slot task-priority (type INTEGER))
  (slot place (type SYMBOL))
  (slot machine (type SYMBOL))
  (slot zone (type SYMBOL))
  (slot product-type (type SYMBOL))
  (slot machine-feature (type SYMBOL) (allowed-symbols CONVEYOR SHELF SLIDE))
  (slot base (type SYMBOL) (allowed-symbols BLACK SILVER RED))
  (slot ring (type SYMBOL) (allowed-symbols BLUE GREEN YELLOW ORANGE))
  (slot cs-operation (type SYMBOL) (allowed-symbols MOUNT_CAP RETRIEVE_CAP))
  (slot gate (type INTEGER) (allowed-values 1 2 3))
  (slot product-id (type INTEGER))
  (slot already-at-mps (type SYMBOL) (allowed-symbols TRUE FALSE) (default FALSE))
  (slot side (type SYMBOL) (allowed-symbols INPUT OUTPUT) (default OUTPUT))
  (slot lock (type SYMBOL) (default NONE))
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
  (slot key (type INTEGER) (default 0)) ; composed of sync-id of the fact to synchronize and the slot to change
  (slot value (default nil))
  (multislot last-sent (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot id (type INTEGER) (default 0)) ;random id
  (slot agent (type SYMBOL) (default DEFAULT))
)

(deftemplate wm-sync-info
  (multislot synced-templates (type SYMBOL) (default (create$)))
)

(deftemplate wait-for-lock
  (slot res (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values new get use finished) (default new))
  (slot priority (type INTEGER) (default 0))
  (slot place (type SYMBOL) (default NOT-SET))
)

(deftemplate skill-to-execute
  (slot skill (type SYMBOL))
  (multislot args (type SYMBOL) (default (create$)))
  (slot state (type SYMBOL) (allowed-symbols wait-for-lock running final failed) (default wait-for-lock))
  (slot target (type SYMBOL))
)

(deftemplate zone-waitpoint
  (slot name (type SYMBOL))
  (slot x (type FLOAT))
  (slot y (type FLOAT))
)

(deftemplate place-waitpoint-assignment
  (slot place (type SYMBOL))
  (slot waitpoint (type SYMBOL))
)

(deftemplate puck-storage
  (slot name (type SYMBOL))
  (slot puck (type INTEGER) (default 0))
  (slot team (type SYMBOL) (allowed-symbols CYAN MAGENTA))
  (multislot incoming (type SYMBOL) (allowed-symbols STORE_PUCK GET_STORED_PUCK))
  (multislot incoming-agent (type SYMBOL))
)

(deftemplate last-lights
  (slot green (type SYMBOL) (allowed-symbols ON OFF BLINKING UNKNOWN) (default UNKNOWN))
  (slot yellow (type SYMBOL) (allowed-symbols ON OFF BLINKING UNKNOWN) (default UNKNOWN))
  (slot red (type SYMBOL) (allowed-symbols ON OFF BLINKING UNKNOWN) (default UNKNOWN))
)

(deftemplate last-zoneinfo
  (slot search-state (type SYMBOL) (allowed-symbols YES NO MAYBE UNKNOWN) (default UNKNOWN))
  (slot tag-id (type INTEGER) (default -1))
)

(deftemplate mps-instruction
  (slot machine (type SYMBOL) (allowed-symbols C-BS C-CS1 C-CS2 C-RS1 C-RS2 C-DS M-BS M-CS1 M-CS2 M-RS1 M-RS2 M-DS))
  (multislot timer (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  ; lock that the robot needs before sending the instruction
  ; (NONE for no needed lock)
  (slot lock (type SYMBOL) (default NONE))
  ;times sent
  (slot seq (type INTEGER) (default 0))
  ;optional fields for different instruction-types:
  (slot base-color (type SYMBOL) (allowed-symbols BLACK SILVER RED))
  (slot ring-color (type SYMBOL) (allowed-symbols BLUE GREEN YELLOW ORANGE))
  (slot gate (type INTEGER) (allowed-values 1 2 3))
  (slot cs-operation (type SYMBOL) (allowed-symbols MOUNT_CAP RETRIEVE_CAP))
  (slot side (type SYMBOL) (allowed-symbols INPUT OUTPUT))
)

(deftemplate exp-current-zone
  (slot name (type SYMBOL))
)

(deffacts startup-facts
  (team-color nil)
  (points MAGENTA 0)
  (points CYAN 0)
  (last-lights)
  (holding NONE)

  (machine (name C-BS) (team CYAN) (mtype BS))
  (base-station (name C-BS))
  (machine (name C-CS1) (team CYAN) (mtype CS))
  (cap-station (name C-CS1))
  (machine (name C-CS2) (team CYAN) (mtype CS))
  (cap-station (name C-CS2))
  (machine (name C-RS1) (team CYAN) (mtype RS))
  (ring-station (name C-RS1))
  (machine (name C-RS2) (team CYAN) (mtype RS))
  (ring-station (name C-RS2))
  (machine (name C-DS) (team CYAN) (mtype DS))

  (machine (name M-BS) (team MAGENTA) (mtype BS))
  (base-station (name M-BS))
  (machine (name M-CS1) (team MAGENTA) (mtype CS))
  (cap-station (name M-CS1))
  (machine (name M-CS2) (team MAGENTA) (mtype CS))
  (cap-station (name M-CS2))
  (machine (name M-RS1) (team MAGENTA) (mtype RS))
  (ring-station (name M-RS1))
  (machine (name M-RS2) (team MAGENTA) (mtype RS))
  (ring-station (name M-RS2))
  (machine (name M-DS) (team MAGENTA) (mtype DS))

  (tag-matching (machine C-BS) (side INPUT) (team CYAN) (tag-id 65))
  (tag-matching (machine C-CS1) (side INPUT) (team CYAN) (tag-id 1))
  (tag-matching (machine C-CS2) (side INPUT) (team CYAN) (tag-id 17))
  (tag-matching (machine C-RS1) (side INPUT) (team CYAN) (tag-id 33))
  (tag-matching (machine C-RS2) (side INPUT) (team CYAN) (tag-id 177))
  (tag-matching (machine C-DS) (side INPUT) (team CYAN) (tag-id 81))
  (tag-matching (machine C-BS) (side OUTPUT) (team CYAN) (tag-id 66))
  (tag-matching (machine C-CS1) (side OUTPUT) (team CYAN) (tag-id 2))
  (tag-matching (machine C-CS2) (side OUTPUT) (team CYAN) (tag-id 18))
  (tag-matching (machine C-RS1) (side OUTPUT) (team CYAN) (tag-id 34))
  (tag-matching (machine C-RS2) (side OUTPUT) (team CYAN) (tag-id 178))
  (tag-matching (machine C-DS) (side OUTPUT) (team CYAN) (tag-id 82))

  (tag-matching (machine M-BS) (side INPUT) (team MAGENTA) (tag-id 161))
  (tag-matching (machine M-CS1) (side INPUT) (team MAGENTA) (tag-id 97))
  (tag-matching (machine M-CS2) (side INPUT) (team MAGENTA) (tag-id 113))
  (tag-matching (machine M-RS1) (side INPUT) (team MAGENTA) (tag-id 129))
  (tag-matching (machine M-RS2) (side INPUT) (team MAGENTA) (tag-id 145))
  (tag-matching (machine M-DS) (side INPUT) (team MAGENTA) (tag-id 49))
  (tag-matching (machine M-BS) (side OUTPUT) (team MAGENTA) (tag-id 162))
  (tag-matching (machine M-CS1) (side OUTPUT) (team MAGENTA) (tag-id 98))
  (tag-matching (machine M-CS2) (side OUTPUT) (team MAGENTA) (tag-id 114))
  (tag-matching (machine M-RS1) (side OUTPUT) (team MAGENTA) (tag-id 130))
  (tag-matching (machine M-RS2) (side OUTPUT) (team MAGENTA) (tag-id 146))
  (tag-matching (machine M-DS) (side OUTPUT) (team MAGENTA) (tag-id 50))

  (state WAIT_START)
  (phase PRE_GAME)
  (refbox-state WAIT_START)
  (game-time (create$ 0 0))
  (game-duration (* 15 60))
  
  (timer (name beacon) (time (create$ 0 0)) (seq 1))
  (timer (name exploration-finished) (time (create$ 0 0)) (seq 1))
  (timer (name send-worldmodel-sync) (time (create$ 0 0)) (seq 1))
  (timer (name send-tag-poses-sync) (time (create$ 0 0)) (seq 1))

  (already-received-wm-changes (create$))

  (pose (x 0.0) (y 0.0))
  (puck-in-gripper FALSE)

  (team-robot R-1)
  (team-robot R-2)
  (team-robot R-3)
  
  ; Input storage per team color
  (input-storage CYAN Ins1 0 0)
  (input-storage MAGENTA Ins2 0 0)
  (secondary-storage CYAN Ins1Sec 0 0)
  (secondary-storage MAGENTA Ins2Sec 0 0)
  (wait-point WAIT1)
  (deliver CYAN deliver1 0 0)
  (deliver MAGENTA deliver2 0 0)

  (wm-sync-info (synced-templates (create$ machine zone-exploration cap-station ring-station product order found-tag base-station exp-matching)))
  ; zone-exploration, machine, cap-station, product, ring station

  (last-zoneinfo)

)
