;---------------------------------------------------------------------------
;  utils.clp - Robotino agent decision testing -- utility functions
;
;  Created: Sun Jun 17 12:19:34 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;             2013  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffunction distance (?x ?y ?x2 ?y2)
  "Returns the distance of two points in the x,y-plane."
  (return (float (sqrt (float(+ (* (- ?x ?x2) (- ?x ?x2)) (* (- ?y ?y2) (- ?y ?y2)))))))
)

; Note that this function only works if the roll and pitch are zero, i.e.
; that a pointing vector is in the x-y plane.
(deffunction yaw-from-quaternion (?q)
  (return (* 2 (acos (nth$ 4 ?q)) (if (> (nth$ 3 ?q) 0) then 1 else -1)))
)

(deffunction quaternion-from-yaw (?yaw)
  (return (create$ 0 0
		   (sin (/ ?yaw 2))
		   (cos (/ ?yaw 2))))
)