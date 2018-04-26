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

(deffunction clips-name (?zone)
  (return
    (sym-cat (sub-string 1 1 ?zone) "-" (sub-string 3 99 ?zone))
  )
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

  (bind ?msg (blackboard-create-msg
    "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps"
    "GenerateWaitZonesMessage"
  ))
  (blackboard-send-msg ?msg)

  (if ?any-tag-to-add
    then
    ; send compute message so we can drive to the output
    (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "ComputeMessage"))
    (bind ?compute-msg-id (blackboard-send-msg ?msg))
    (printout t "Sent compute" crlf)

    ; save the last compute-msg-id to know when it was processed
    ; (delayed-do-for-all-facts ((?lncm last-navgraph-compute-msg)) TRUE
    ;   (retract ?lncm)
    ; )
    ; (assert (last-navgraph-compute-msg (id ?compute-msg-id)))
    else
    (printout t "There are no tags to add" crlf)
  )
)

;---------------Exploration-Phase functions--------------------------
(deffunction distance (?x ?y ?x2 ?y2)
  "Returns the distance of two points in the x,y-plane."
  (return (float (sqrt (float(+ (* (- ?x ?x2) (- ?x ?x2)) (* (- ?y ?y2) (- ?y ?y2)))))))
)

(deffunction protobuf-name (?zone)
  (return
    (str-cat (sub-string 1 1 ?zone) "_" (sub-string 3 99 ?zone))
  )
)

(deffunction transform-safe (?to-frame ?from-frame ?timestamp ?trans ?rot)
  (if (tf-can-transform ?to-frame ?from-frame ?timestamp) then
    (bind ?rv (tf-transform-pose ?to-frame ?from-frame ?timestamp ?trans ?rot))
  else
    (if (tf-can-transform ?to-frame ?from-frame (create$ 0 0)) then
      (bind ?rv (tf-transform-pose ?to-frame ?from-frame (create$ 0 0) ?trans ?rot))
    else
      (return FALSE)
    )
  )
  (if (= (length$ ?rv) 7) then
    (return ?rv)
  else
    (return FALSE)
  )
)

(deffunction compensate-movement (?factor ?v-odom ?p ?timestamp)
  (if (eq ?p FALSE) then
    (return FALSE)
  )
  (if (= 2 (length$ ?v-odom)) then
    (bind ?v-odom (create$ (nth$ 1 ?v-odom) (nth$ 2 ?v-odom) 0))
  )
  (bind ?p-map (transform-safe "map" "base_link" ?timestamp (create$ 0 0 0) (create$ 0 0 0 1)))
  (if (<> 7 (length$ ?p-map)) then
    (return FALSE)
  )
  (bind ?pv-map (transform-safe "map" "base_link" ?timestamp ?v-odom (create$ 0 0 0 1)))
  (if (<> 7 (length$ ?pv-map)) then
    (return FALSE)
  )
  (bind ?v-map (create$
    (- (nth$ 1 ?pv-map) (nth$ 1 ?p-map))
    (- (nth$ 2 ?pv-map) (nth$ 2 ?p-map))
  ))
  (bind ?rv (create$
    (+ (nth$ 1 ?p) (* ?factor (nth$ 1 ?v-map)))
    (+ (nth$ 2 ?p) (* ?factor (nth$ 2 ?v-map)))
  ))
  (return ?rv)
)

(deffunction distance-mf (?p1 ?p2)
  (return (distance (nth$ 1 ?p1) (nth$ 2 ?p1) (nth$ 1 ?p2) (nth$ 2 ?p2)))
)

(deffunction round-down (?x)
  (bind ?round (round ?x))
  (if (< ?x ?round) then
    (return (- ?round 1))
  )
  (return ?round)
)

(deffunction round-up (?x)
  (bind ?round (round ?x))
  (if (> ?x ?round) then
    (return (+ ?round 1))
  )
  (return ?round)
)
(deffunction get-zone (?margin $?vector)
  "Return the zone name for a given map coordinate $?vector if its
   distance from the zone borders is greater or equal than ?margin."
  (if (eq ?vector FALSE) then
    (return FALSE)
  )
  (bind ?x (nth$ 1 ?vector))
  (bind ?y (nth$ 2 ?vector))
  (if (not (and (numberp ?x) (numberp ?y))) then
    (return FALSE)
  )

  (if (<= ?y 0) then
    ; y <= 0 is outside the playing field
    (return FALSE)
  else
    (bind ?yr (round-up ?y))
  )

  (if (or (< (- ?x ?margin) (round-down ?x))
          (> (+ ?x ?margin) (round-up ?x))
          (< (- ?y ?margin) (round-down ?y))
          (> (+ ?y ?margin) (round-up ?y))
      ) then
    (return FALSE)
  )

  (if (< ?x 0) then
    (bind ?rv M-Z)
    (bind ?x (* ?x -1))
  else
    (bind ?rv C-Z)
  )
  (bind ?xr (round-up ?x))

  (return (sym-cat ?rv ?xr ?yr))
)
(deffunction utils-get-2d-center (?x1 ?y1 ?x2 ?y2)
  (return (create$ (/ (+ ?x1 ?x2) 2) (/ (+ ?y1 ?y2) 2)))
)
(deffunction laser-line-center-map (?ep1 ?ep2 ?frame ?timestamp)
  (bind ?c (utils-get-2d-center (nth$ 1 ?ep1) (nth$ 2 ?ep1) (nth$ 1 ?ep2) (nth$ 2 ?ep2)))
  (bind ?c3 (nth$ 1 ?c) (nth$ 2 ?c) 0)
  (return (transform-safe "map" ?frame ?timestamp ?c3 (create$ 0 0 0 1)))
)

(deffunction mirror-name (?zn)
  (bind ?team (sub-string 1 1 ?zn))
  (bind ?zone (sub-string 3 99 ?zn))
  (if (eq ?team "M") then
    (return (sym-cat "C-" ?zone))
  else
    (return (sym-cat "M-" ?zone))
  )
)

(deffunction want-mirrored-rotation (?mtype ?zone)
"According to the RCLL2017 rulebook, this is when a machine is mirrored"
  (bind ?zn (str-cat ?zone))
  (bind ?x (eval (sub-string 4 4 ?zn)))
  (bind ?y (eval (sub-string 5 5 ?zn)))

  (return (or (member$ ?mtype (create$ BS DS SS))
              (not (or (eq ?x 7) ; left or right
                       (eq ?y 8) ; top wall
                       (eq ?y 1) ; bottom wall
                       (and (member$ ?x (create$ 5 6 7)); insertion
                            (eq ?y 2)
                       )
                   )
              )
  ))
)

(deffunction mirror-orientation (?mtype ?zone ?ori)
  (bind ?zn (str-cat ?zone))
  (bind ?t (sub-string 1 1 ?zn))
  (if (want-mirrored-rotation ?mtype ?zone)
   then
    (if (eq ?t "C")
     then
      (do-for-fact ((?mo domain-fact)) (and (eq (nth$ 1 ?mo:param-values) ?ori) (eq ?mo:name mirror-orientation))
        (bind ?m-ori (nth$ 2 ?mo:param-values))
      )
     else
      (do-for-fact ((?mo domain-fact)) (and (eq (nth$ 2 ?mo:param-values) ?ori) (eq ?mo:name mirror-orientation))
        (bind ?m-ori (nth$ 1 ?mo:param-values))
      )
    )
    (return ?m-ori)
   else
    (bind ?x (eval (sub-string 4 4 ?zn)))
    (bind ?y (eval (sub-string 5 5 ?zn)))

    (if (eq ?y 8) then
      (return 180)
    )
    (if (or (eq ?y 1) (eq ?y 2)) then
      (return 0)
    )
    (if (and (eq ?x 7) (eq ?t "M")) then  ; this is the other way around, because I compare with the team color of the originalting machine
      (return 90)
    )
    (if (and (eq ?x 7) (eq ?t "C")) then
      (return 270)
    )
    (printout error "error in rotation of machines, checked all possible cases, but nothing cateched" crlf)
    (return ?ori)
  )
)

(deffunction mirror-team (?team)
  (if (eq (sym-cat ?team) CYAN) then
    (return MAGENTA)
  else
    (return CYAN)
  )
)
