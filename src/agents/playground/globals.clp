
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