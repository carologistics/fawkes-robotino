
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
  ?*LOCK-PERIOD* = 0.2

  ?*TEAM-NAME*  = "?"
  ?*ROBOT-NAME* = "?"
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
