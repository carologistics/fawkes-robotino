
;---------------------------------------------------------------------------
;  priorities.clp - Robotino agent decision testing -- priorities
;
;  Created: Sat Jun 16 20:06:24 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ; evaluation priorities
  ?*PRIORITY-HIGH*    =  500
  ?*PRIORITY-SIM*     =  300
  ?*PRIORITY-WM*      =  200
  ?*PRIORITY-LOW*     = -100
  ?*PRIORITY-SKILL*   = -150
  ?*PRIORITY-CLEANUP* = -400
  ?*PRIORITY-LAST*    = -500

  ; production order priorities
  ?*PRIORITY-DELIVER* =   100
  ?*PRIORITY-P*       =   90
  ?*PRIORITY-S2*      =   80
  ?*PRIORITY-S1*      =   70
  ?*PRIORITY-T5*      =    0
  ?*PRIORITY-T4*      =    0
  ?*PRIORITY-T3*      =   20
  ?*PRIORITY-T2*      =   10
  ?*PRIORITY-T1*      =    5

  ; get s0 from insertion or recycling
  ?*PRIORITY-GET-S0-INS*     =    0
  ?*PRIORITY-GET-S0-RECYCLE* =  100

  ; locking priorities
  ?*PRIORITY-LOCK-SEND*    = -100 ;so they get only send if not used by the agent itself
)
