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
  ?*PRIORITY-JUST-IN-TIME-P3* = 105
  ?*PRIORITY-DELIVER-STORED* = 102
  ?*PRIORITY-DELIVER-P3* = 100
  ?*PRIORITY-DELIVER-P1P2* = 100
  ?*PRIORITY-STORE-PRODUCED* = 95
  ?*PRIORITY-RECYCLE* = 90
  ?*PRIORITY-LOAD-T5-WITH-S0* = 80
  ?*PRIORITY-LOAD-T3_T4-WITH-S2* = 73
  ?*PRIORITY-CONTINUE-T3_T4-WITH-S1* = 70
  ?*PRIORITY-START-T3_T4-WITH-S0* = 75
  ?*PRIORITY-PREPARE-T3_T4-WITH-S1* = 63
  ?*PRIORITY-PREPARE-T3_T4-WITH-S0* = 64
  ?*PRIORITY-LOAD-T3_T4-WITH-S1* = 55
  ?*PRIORITY-LOAD-T3_T4-WITH-S0* = 50
  ?*PRIORITY-LOAD-T2-WITH-S1* = 60
  ?*PRIORITY-START-T2-WITH-S0* = 65
  ?*PRIORITY-LOAD-T2-WITH-S0* = 55
  ?*PRIORITY-DELIVER-HOLDING* = 5
  ?*PRIORITY-LOAD-HOLDING-S2* = 5
  

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
