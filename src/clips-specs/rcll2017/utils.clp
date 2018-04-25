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


(deffunction navgraph-add-all-new-tags ()
  "send all new tags to the navgraph generator"
  (bind ?any-tag-to-add FALSE)

  (delayed-do-for-all-facts ((?ft-n wm-fact))
      (wm-key-prefix ?ft-n:key (create$ game found-tag name))
    (bind ?mps (wm-key-arg ?ft-n:key m))

      (do-for-fact ((?wm-fact wm-fact)) (eq ?wm-fact:key (create$ game found-tag side args? m ?mps))
        (bind ?side ?wm-fact:value))
      (do-for-fact ((?wm-fact wm-fact)) (eq ?wm-fact:key (create$ game found-tag frame args? m ?mps))
        (bind ?frame ?wm-fact:value))
      (do-for-fact ((?wm-fact wm-fact)) (eq ?wm-fact:key (create$ game found-tag trans args? m ?mps))
        (bind ?trans ?wm-fact:values))
      (do-for-fact ((?wm-fact wm-fact)) (eq ?wm-fact:key (create$ game found-tag rot args? m ?mps))
        (bind ?rot ?wm-fact:values))
      (do-for-fact ((?wm-fact wm-fact)) (eq ?wm-fact:key (create$ game found-tag zone args? m ?mps))
        (bind ?zone ?wm-fact:value))

      (bind ?any-tag-to-add TRUE)
      ; report tag position to navgraph generator
      (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "UpdateStationByTagMessage"))
      (blackboard-set-msg-field ?msg "name" (str-cat ?mps))
      (blackboard-set-msg-field ?msg "side" ?side)
      (blackboard-set-msg-field ?msg "frame" ?frame)
      (blackboard-set-msg-multifield ?msg "tag_translation" ?trans)
      (blackboard-set-msg-multifield ?msg "tag_rotation" ?rot)
      (blackboard-set-msg-multifield ?msg "zone_coords" (zone-coords ?zone))
      (blackboard-send-msg ?msg)
      (printout t "Send UpdateStationByTagMessage: id " (str-cat ?mps)
          " side " ?side
          " frame " ?frame
          " trans " ?trans
          " rot " ?rot
          " zone " ?zone
          crlf)
    ;   ; (assert (navgraph-added-for-mps (name ?ft-mps:value)))
    ; )
  )
  (if ?any-tag-to-add
    then
    ; send compute message so we can drive to the output
    (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "ComputeMessage"))
    (bind ?compute-msg-id (blackboard-send-msg ?msg))

    ; save the last compute-msg-id to know when it was processed
    ; (delayed-do-for-all-facts ((?lncm last-navgraph-compute-msg)) TRUE
    ;   (retract ?lncm)
    ; )
    ; (assert (last-navgraph-compute-msg (id ?compute-msg-id)))
    else
    (printout t "There are no tags to add" crlf)
  )
)

(deffunction random-id ()
  "Return a random task id"
  (return (random 0 1000000000))
)

(deffunction get-param-by-arg (?params ?arg)
	"Extract the argument named in ?arg.
   @param ?params the paramter list
   @param ?arg name of argument to extract

   @return returns nil if value is not specified, or a single or
           multi-field value depending on the argument value."
	(bind ?l (member$ ?arg ?params))
	(bind ?L (length$ ?params))
	(if ?l then
		(while (<= (+ ?l 1) ?L) do
			(if (eq (nth$ ?l ?params) ?arg) then
				(return (nth$ (+ ?l 1) ?params))
			)
		)
	)
	(return nil)
)
