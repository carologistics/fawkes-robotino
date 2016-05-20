
;---------------------------------------------------------------------------
;  globals.clp - LLSF2015 CLIPS Agent global CLIPS variables
;
;  Created: Thu Apr 25 10:37:43 2013 (GO2013 Magdeburg)
;  Copyright  2013  Tim Niemueller [www.niemueller.de]
;  Licensed under BSD license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ; network sending periods; seconds
  ?*BEACON-PERIOD* = 1.0
  ?*MASTER-ANNOUNCE-PERIOD* = 0.5
  ?*LOCK-PERIOD* = 0.5
  ?*LOCK-STATUS-SEND-PERIOD* = 1.0
  ?*WORLDMODEL-SYNC-PERIOD* = 1.0
  ?*WORLDMODEL-CHANGE-SEND-PERIOD* = 0.5
  ?*MPS_INSTRUCTION-PERIOD* = 0.5

  ?*MIN-TIMES-TO-SEND-MPS-INSTRUCTIONS* = 4

  ?*TEAM-NAME*    = "?"
  ?*ROBOT-NAME*   = "?"
  ?*ROBOT-NUMBER* = 0

  ; Time before the slave becomes the master
  ;(if there is a master the timeout gets larger)
  ?*CURRENT-MASTER-TIMEOUT* = 5.0
  ?*INITIAL-MASTER-TIMEOUT* = 5.0
  ?*MASTER-TIMEOUT-ROBOT-OFFSET* = 5.0
  ?*ROBOT-TIMEOUT* = 10.0
  ?*RELEASE-DISTANCE* = 0.5

  ?*FAILS-TO-BLOCK* = 2

  ?*LOCK-ANNOUNCE-RESTART-PERIOD* = 0.25
  ?*LOCK-ANNOUNCE-RESTART-REPETITIONS* = 8
  ?*LOCK-ANNOUNCE-RESTART-WAIT-BEFORE-FINISH* = 3.0

  ;initial values for skill-durations:
  ?*SKILL-DURATION-GET-PRODUCED* = 10
  ?*SKILL-DURATION-GET-STORED-PUCK* = 10
  ?*SKILL-DURATION-DELIVER* = 10

  ;how long do we want to produce P3 without leaving the machine
  ?*TIME-P3-PRODUCTION-WITHOUT-LEAVING* = 150

  ;maximum distance from the delivery gates to allow leaving a T5 machine while producing
  ?*MAX-T5-LEAVE-DISTANCE* = 6.0

  ;After the change of a decision based on a new worldmodel the remove msg might have arrived before the add message. Timeout to wait until there is a field to remove:
  ?*DELAYED-WORLDMODEL-CHANGE-TIMEOUT* = 10

  ;Timeout before removing rejected tasks when no task is found
  ?*TIMEOUT-REMOVE-REJECTED-WHILE-WAITING* = 25
  
  ?*PI* = 3.141592653589
  ?*2PI* = 6.2831853
  ?*PI-HALF* = 1.5707963

  ; some measures of the game
  ?*ZONE-HEIGHT* = 1.5
  ?*ZONE-WIDTH* = 2.0

  ; synchronization ids
  ?*LAST-INITIAL-SYNC-ID* = 5
  ?*USING-INITIAL-SYNC-IDS* = TRUE
  ?*SYNC-ID-ASSERT* = 1
  ?*SYNC-ID-RETRACT* = 2

  ?*MESSAGE-OVERHEAD-SIZE* = 4
  ?*MAX-MESSAGE-SIZE* = 900 ; bytes

  ?*PRODUCE-C0-AHEAD-TIME* = 90
  ?*PRODUCE-C0-LATEST-TIME* = 30
  ?*PRODUCE-CX-AHEAD-TIME* = 90
  ?*PRODUCE-CX-LATEST-TIME* = 30
  ?*DELIVER-AHEAD-TIME* = 60
  ?*DELIVER-LATEST-TIME* = 10
  ?*DELIVER-ABORT-TIMEOUT* = 30
)

(defrule globals-config-team-name
  (confval (path "/clips-agent/rcll2016/team-name") (type STRING) (value ?team-name))
  =>
  (bind ?*TEAM-NAME* ?team-name)
)

(defrule globals-config-robot-name
  (confval (path "/clips-agent/rcll2016/robot-name") (type STRING) (value ?robot-name))
  =>
  (bind ?*ROBOT-NAME* ?robot-name)
)

(defrule globals-config-robot-number
  (confval (path "/clips-agent/rcll2016/robot-number") (type UINT) (value ?robot-number))
  =>
  (bind ?*ROBOT-NUMBER* ?robot-number)
)

(defrule globals-config-timeouts
  (confval (path "/clips-agent/rcll2016/initial-master-timeout") (type FLOAT) (value ?initial))
  (confval (path "/clips-agent/rcll2016/robot-timeout") (type FLOAT) (value ?robot-timeout))
  (confval (path "/clips-agent/rcll2016/robot-number") (type UINT) (value ?robot-number))
  =>
  (bind ?*ROBOT-TIMEOUT* ?robot-timeout)
  (bind ?*INITIAL-MASTER-TIMEOUT* ?initial)
  (bind ?*MASTER-TIMEOUT-ROBOT-OFFSET* (* ?robot-number 3))
)

(defrule globals-config-release-distance
  (confval (path "/clips-agent/rcll2016/release-distance") (type FLOAT) (value ?d))
  =>
  (bind ?*RELEASE-DISTANCE* ?d)
)

(defrule globals-config-estimated-skill-durations
  (confval (path "/clips-agent/rcll2016/estimated-skill-duration/get-produced") (type UINT) (value ?d-get-produced))
  (confval (path "/clips-agent/rcll2016/estimated-skill-duration/get-stored-puck") (type UINT) (value ?d-get-stored-puck))
  (confval (path "/clips-agent/rcll2016/estimated-skill-duration/deliver") (type UINT) (value ?d-deliver))
  =>
  (bind ?*SKILL-DURATION-GET-PRODUCED* ?d-get-produced)
  (bind ?*SKILL-DURATION-GET-STORED-PUCK* ?d-get-stored-puck)
  (bind ?*SKILL-DURATION-DELIVER* ?d-deliver)
)
