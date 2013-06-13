
;---------------------------------------------------------------------------
;  utils.clp - Robotino agent decision testing -- utility functions
;
;  Created: Sun Jun 17 12:19:34 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; Note that this function only works if the roll and pitch are zero, i.e.
; that a pointing vector is in the x-y plane.
(deffunction yaw-from-quaternion (?q)
  (return (* 2 (acos (nth$ 4 ?q)) (if (> (nth$ 3 ?q) 0) then 1 else -1)))
)

