
;---------------------------------------------------------------------------
;  globals.clp - LLSF2013 CLIPS Agent global CLIPS variables
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
  ?*WORLDMODEL-SYNC-PERIOD* = 0.5
  ?*WORLDMODEL-CHANGE-SEND-PERIOD* = 0.5

  ?*TEAM-NAME*    = "?"
  ?*ROBOT-NAME*   = "?"
  ?*ROBOT-NUMBER* = 0

  ; Time before the slave becomes the master
  ;(if there is a master the timeout gets larger)
  ?*CURRENT-MASTER-TIMEOUT* = 2.0
  ?*INITIAL-MASTER-TIMEOUT* = 2.0
  ?*ROBOT-TIMEOUT* = 6.0
  ?*RELEASE-DISTANCE* = 0.5
)

(defrule globals-config-team-name
  (confval (path "/clips-agent/llsf2013/team-name") (type STRING) (value ?team-name))
  =>
  (bind ?*TEAM-NAME* ?team-name)
)

(defrule globals-config-robot-name
  (confval (path "/clips-agent/llsf2013/robot-name") (type STRING) (value ?robot-name))
  =>
  (bind ?*ROBOT-NAME* ?robot-name)
)

(defrule globals-config-robot-number
  (confval (path "/clips-agent/llsf2013/robot-number") (type UINT) (value ?robot-number))
  =>
  (bind ?*ROBOT-NUMBER* ?robot-number)
)

(defrule globals-config-timeouts
  (confval (path "/clips-agent/llsf2013/initial-master-timeout") (type FLOAT) (value ?initial))
  (confval (path "/clips-agent/llsf2013/robot-timeout") (type FLOAT) (value ?robot-timeout))
  =>
  (bind ?*ROBOT-TIMEOUT* ?robot-timeout)
  (bind ?*INITIAL-MASTER-TIMEOUT* ?initial)
)

(defrule globals-config-timeouts
  (confval (path "/clips-agent/llsf2013/release-distance") (type FLOAT) (value ?d))
  =>
  (bind ?*RELEASE-DISTANCE* ?d)
)
