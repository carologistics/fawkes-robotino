
;---------------------------------------------------------------------------
;  priorities.clp - Robotino agent decision testing -- priorities
;
;  Created: Sat Jun 16 20:06:24 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;                   Frederik Zwilling
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
  ?*PRIORITY-LOAD-T3_T4-WITH-S2* = 80
  ?*PRIORITY-LOAD-T3_T4-WITH-S1* = 83
  ?*PRIORITY-LOAD-T3_T4-WITH-S0* = 86
  ?*PRIORITY-LOAD-T2-WITH-S1* = 73
  ?*PRIORITY-LOAD-T2-WITH-S0* = 70
  ?*PRIORITY-LOAD-T5-WITH-S0* = 68
  

  ; old production order priorities
  ?*PRIORITY-DELIVER* =   100
  ?*PRIORITY-P*       =   90
  ?*PRIORITY-S2*      =   80
  ?*PRIORITY-S1*      =   70
  ?*PRIORITY-T5*      =    0
  ?*PRIORITY-T4*      =    0
  ?*PRIORITY-T3*      =   20
  ?*PRIORITY-T2*      =   10
  ?*PRIORITY-T1*      =    5

  ; tast execution order priorities
  ; (if the first subtasks are already complete, don't do them again)
  ?*PRIORITY-SUBTASK-1* = 10
  ?*PRIORITY-SUBTASK-2* = 20
  ?*PRIORITY-SUBTASK-3* = 30
  ?*PRIORITY-SUBTASK-4* = 40
  ?*PRIORITY-SUBTASK-5* = 50
  ?*PRIORITY-SUBTASK-6* = 60

  ; get s0 from insertion or recycling
  ?*PRIORITY-GET-S0-INS*     =    0
  ?*PRIORITY-GET-S0-RECYCLE* =  100

  ;Locking priorities
  ?*PRIORITY-LOCKING-HIGH* = 200
  ?*PRIORITY-LOCKING-LOW* = 10

  ; locking priorities
  ?*PRIORITY-LOCK-HIGH* = 280
  ?*PRIORITY-LOCK-CLEAN* = 250
  ?*PRIORITY-LOCK-LOW* = 230
  ?*PRIORITY-LOCK-SEND* = -5
  ?*PRIORITY-LOCK_USAGE* = 110
)
