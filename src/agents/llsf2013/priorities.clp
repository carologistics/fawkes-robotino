
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
  ?*PRIORITY-WM-MID*  =  195
  ?*PRIORITY-WM-LOW*  =  190
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

  ; locking priorities
  ?*PRIORITY-LOCK-HIGH* = 280
  ?*PRIORITY-LOCK-CLEAN* = 250
  ?*PRIORITY-LOCK-LOW* = 230
  ?*PRIORITY-LOCK-SEND* = -5
)
