
;---------------------------------------------------------------------------
;  globals.clp - LLSF2014 CLIPS Agent global CLIPS variables
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

  ?*TEAM-NAME*    = "?"
  ?*ROBOT-NAME*   = "?"
  ?*ROBOT-NUMBER* = 0

  ; Time before the slave becomes the master
  ;(if there is a master the timeout gets larger)
  ?*CURRENT-MASTER-TIMEOUT* = 2.0
  ?*INITIAL-MASTER-TIMEOUT* = 2.0
  ?*ROBOT-TIMEOUT* = 10.0
  ?*RELEASE-DISTANCE* = 0.5

  ?*FAILS-TO-BLOCK* = 2

  ?*LOCK-ANNOUNCE-RESTART-PERIOD* = 0.25
  ?*LOCK-ANNOUNCE-RESTART-REPETITIONS* = 8
  ?*LOCK-ANNOUNCE-RESTART-WAIT-BEFORE-FINISH* = 3.0
)

(defrule globals-config-team-name
  (confval (path "/clips-agent/llsf2014/team-name") (type STRING) (value ?team-name))
  =>
  (bind ?*TEAM-NAME* ?team-name)
)

(defrule globals-config-robot-name
  (confval (path "/clips-agent/llsf2014/robot-name") (type STRING) (value ?robot-name))
  =>
  (bind ?*ROBOT-NAME* ?robot-name)
)

(defrule globals-config-robot-number
  (confval (path "/clips-agent/llsf2014/robot-number") (type UINT) (value ?robot-number))
  =>
  (bind ?*ROBOT-NUMBER* ?robot-number)
)

(defrule globals-config-timeouts
  (confval (path "/clips-agent/llsf2014/initial-master-timeout") (type FLOAT) (value ?initial))
  (confval (path "/clips-agent/llsf2014/robot-timeout") (type FLOAT) (value ?robot-timeout))
  =>
  (bind ?*ROBOT-TIMEOUT* ?robot-timeout)
  (bind ?*INITIAL-MASTER-TIMEOUT* ?initial)
)

(defrule globals-config-release-distance
  (confval (path "/clips-agent/llsf2014/release-distance") (type FLOAT) (value ?d))
  =>
  (bind ?*RELEASE-DISTANCE* ?d)
)
