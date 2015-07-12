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
  ?*PRIORITY-PREFILL-CS* = 50
  ?*PRIORITY-PREFILL-RS* = 40
  ?*PRIORITY-ADD-FIRST-RING* = 80
  ?*PRIORITY-PRODUCE-C0* = 90
  ?*PRIORITY-FIND-MISSING-MPS* = 110
  ?*PRIORITY-PRODUCE-CX* = 95
  ?*PRIORITY-FIND-MISSING-MPS* = 40
  ?*PRIORITY-DELIVER* = 100
  ?*PRIORITY-DISCARD-UNKNOWN* = 10
  ?*PRIORITY-NOTHING-TO-DO* = -1

  ; tast execution order priorities
  ; (if the first subtasks are already complete, don't do them again)
  ?*PRIORITY-STEP-START* = 10
  ?*PRIORITY-STEP-FAILED* = 20
  ?*PRIORITY-STEP-FINISH* = 30

  ;Locking priorities
  ?*PRIORITY-LOCKING-HIGH* = 200
  ?*PRIORITY-LOCKING-LOW* = 10

  ; locking priorities
  ?*PRIORITY-LOCK-HIGH* = 280
  ?*PRIORITY-LOCK-CLEAN* = 250
  ?*PRIORITY-LOCK-LOW* = 230
  ?*PRIORITY-LOCK-SEND* = -5
  ?*PRIORITY-LOCK_USAGE* = 110

  ; priorities for skill execution
  ?*PRIORITY-SKILL-DEFAULT* = 0
  ?*PRIORITY-SKILL-SPECIAL-CASE* = 5

  ; exploration
  ?*PRIORITY-EXP-PARTLY-EXPLORED* = 5
  ?*PRIORITY-EXP-NEAREST* = 0
)
