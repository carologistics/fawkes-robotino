
;---------------------------------------------------------------------------
;  priorities.clp - Robotino agent decision testing -- priorities
;
;  Created: Sat Jun 16 20:06:24 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*PRIORITY_HIGH*    =  500
  ?*PRIORITY_WM*      =  100
  ?*PRIORITY_WM_EXT*  =   90
  ?*PRIORITY_WM_DEF*  =   80
  ?*PRIORITY_S2*      =   20
  ?*PRIORITY_P*       =   30
  ?*PRIORITY_LOW*     = -100
  ?*PRIORITY_SKILL*   = -150
  ?*PRIORITY_CLEANUP* = -400
  ?*PRIORITY_LAST*    = -500

  ?*GOTOPRIO_DELIVER* = 50
  ?*GOTOPRIO_M3*      = 40
  ?*GOTOPRIO_M1_2*    = 35
  ?*GOTOPRIO_M2_3*    = 30
  ?*GOTOPRIO_M2*      = 20
  ?*GOTOPRIO_M1*      = 10
  ?*GOTOPRIO_UNK*     =  0
)
