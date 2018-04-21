;---------------------------------------------------------------------------
;  util.clp - utilities needed for executive rcll agent
;
;  Created: Tue 19 Apr 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.

(defglobal
  ?*PI* = 3.141592653589
  ?*2PI* = 6.2831853
  ?*PI-HALF* = 1.5707963
  )

(deffunction clips-name (?zone)
  (return
    (sym-cat (sub-string 1 1 ?zone) "-" (sub-string 3 99 ?zone))
  )`
)

(deffunction deg-to-rad (?deg)
  (bind ?bigrad (* (/ ?deg 360) ?*2PI*))
  (if (> ?bigrad ?*PI*) then
    (return (* -1 (- ?*2PI* ?bigrad)))
  else
    (return ?bigrad)
  )
)

(deffunction zone-center (?zn)
  (bind ?x (eval (sub-string 4 4 ?zn)))
  (bind ?y (eval (sub-string 5 5 ?zn)))
  (if (eq (sub-string 1 1 ?zn) "M") then
    (bind ?x (* -1 ?x))
    (bind ?sgn -1)
  else
    (bind ?sgn 1)
  )
  (return (create$ (- ?x (* ?sgn 0.5)) (- ?y 0.5)))
)


(deffunction zone-coords (?zn)
  (bind ?x (eval (sub-string 4 4 ?zn)))
  (bind ?y (eval (sub-string 5 5 ?zn)))
  (if (eq (sub-string 1 1 ?zn) "M") then
    (bind ?x (* -1 ?x))
  )
  (return (create$ ?x ?y))
)

(deffunction tag-offset (?zone ?yaw ?width)
  (bind ?c (zone-center ?zone))
  (bind ?x (nth$ 1 ?c))
  (bind ?y (nth$ 2 ?c))
  (bind ?x (+ ?x (* (cos ?yaw) ?width)))
  (bind ?y (+ ?y (* (sin ?yaw) ?width)))
  (return (create$ ?x ?y 0.48))
)
