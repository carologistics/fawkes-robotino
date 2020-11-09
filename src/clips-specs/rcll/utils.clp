;---------------------------------------------------------------------------
;  util.clp - utilities needed for executive rcll agent
;
;  Created: Tue 19 Apr 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>, Daniel Habering
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

  ?*POINTS-MOUNT-RING-CC0* = 5
  ?*POINTS-MOUNT-RING-CC1* = 10
  ?*POINTS-MOUNT-RING-CC2* = 20
  ?*POINTS-MOUNT-LAST-RING-C1* = 10
  ?*POINTS-MOUNT-LAST-RING-C2* = 30
  ?*POINTS-MOUNT-LAST-RING-C3* = 80
  ?*POINTS-MOUNT-CAP* = 10
  ?*POINTS-DELIVER* = 20
  ?*POINTS-COMPETITIVE* = 10

  ?*TIME-MOUNT-RING* = 60
  ?*TIME-MOUNT-CAP* = 90
  ?*TIME-DELIVER* = 120
  ?*TIME-GET-BASE* = 30
  ?*TIME-RETRIEVE-CAP* = 60
  ?*TIME-FILL-RS* = 20

; Maximum distance between two points on the field
  ?*MAX-DISTANCE* = 16.124

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
  "Replaces the 2nd character of any string with '-' and returns it as symbol
  @param ?zone name of the zone

  @return returns the input as string with the second character replaced by a '-'
  "
  (return
    (sym-cat (sub-string 1 1 ?zone) "-" (sub-string 3 99 ?zone))
  )
)


(deffunction deg-to-rad (?deg)
  "Converts an angle in degree to radiant
  @param ?deg angle in degree

  @return angle in radiant
  "
  (bind ?bigrad (* (/ ?deg 360) ?*2PI*))
  (if (> ?bigrad ?*PI*) then
    (return (* -1 (- ?*2PI* ?bigrad)))
  else
    (return ?bigrad)
  )
)


(deffunction zone-center (?zn)
  "Calculates the coordinates of the center of a zone
  @param ?zn name of the zone (eg. M-Z34)

  @return coordinates of the zone as multifield (eg (3.5 4.5))
  "
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
  "Extracts the coordinates of the zone
  @param ?zn id of the zone (eg M-Z34)

  @return coordinates of the zone as multifield (eg (3 4))
  "
  (bind ?x (eval (sub-string 4 4 ?zn)))
  (bind ?y (eval (sub-string 5 5 ?zn)))
  (if (eq (sub-string 1 1 ?zn) "M") then
    (bind ?x (* -1 ?x))
  )
  (return (create$ ?x ?y))
)


(deffunction tag-offset (?zone ?yaw ?width)
  "Calculates the offset of the position of a tag inside a zone
  @param ?zone id of the zone (eg M-Z33)
  @param ?yaw rotation of the machine in this zone
  @param ?width width of the tag

  @return coordinates of the tag as multifield
  "
  (bind ?c (zone-center ?zone))
  (bind ?x (nth$ 1 ?c))
  (bind ?y (nth$ 2 ?c))
  (bind ?x (+ ?x (* (cos ?yaw) ?width)))
  (bind ?y (+ ?y (* (sin ?yaw) ?width)))
  (return (create$ ?x ?y 0.48))
)


(deffunction navgraph-add-all-new-tags ()
  "Send all new tags to the navgraph generator"
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

(deffunction wm-fact-to-navgraph-node (?key)
	"Get the name of the navgraph node given a wm-fact key, where the machine has
	 the arg 'm' and the side the arg 'side'.
	 @param ?key key of the wm-fact
	 @return The name of the navgraph node."
	(bind ?suffix "")
	(switch (wm-key-arg ?key side)
		(case INPUT then (bind ?suffix "-I"))
		(case OUTPUT then (bind ?suffix "-O"))
	)
	(return (str-cat (wm-key-arg ?key m) ?suffix))
)


;---------------Exploration-Phase functions--------------------------

(deffunction distance (?x ?y ?x2 ?y2)
  "Returns the distance of two points in the x,y-plane.
  @param ?x ?y coordinates of one point
  @param ?x2 ?y2 coordinates of the other point

  @return euclidean distance of the two points
  "
  (return (float (sqrt (float(+ (* (- ?x ?x2) (- ?x ?x2)) (* (- ?y ?y2) (- ?y ?y2)))))))
)


(deffunction protobuf-name (?zone)
  "Replaces the 2nd character of a zone name with '_'
  @param ?zone id of the zone (eg M-Z22)

  @return the input string with 2nd character replaced with '_'
  "
  (return
    (str-cat (sub-string 1 1 ?zone) "_" (sub-string 3 99 ?zone))
  )
)


(deffunction transform-safe (?to-frame ?from-frame ?timestamp ?trans ?rot)
  " Transforms a position and  rotation of one frame into the other
  @param ?to-frame target frame to which the ouput should be relative to
  @param ?from-frame origin frame to which the input coordinates are relative to
  @param ?timestamp timestamp of the input coordinates
  @param ?trans coordinates that should be transformed
  @param ?rot rotation in deg that should be transformed

  @return coordinates and rotation relative to the target frame if possible. FALSE otherwise
  "
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


(deffunction get-mps-type-from-name (?mps)
  " Extracts the type of mps out of the name
  @param ?mps name of the mps (eg M-CS1)

  @return type of the mps as symbol (eg CS)
  "
  (bind ?type (sym-cat (sub-string 3 4 (str-cat ?mps))))
  (return ?type)
)


(deffunction compensate-movement (?factor ?v-odom ?p ?timestamp)
  " Uses measured velocity to relocate a given point according to a given factor
  @param ?factor used to adapt the amount of compensation
  @param ?v-odom measured translation and rotation velocity
  @param ?p point to be adapted
  @param ?timestamp timestamp of the measurement

  @return point that got compensated by the error probable introduced through the moving
  "
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
  "Calculates distance between two points"
  (return (distance (nth$ 1 ?p1) (nth$ 2 ?p1) (nth$ 1 ?p2) (nth$ 2 ?p2)))
)


(deffunction round-down (?x)
  "Rounds a given numer down to the next natural number
  @param ?x number to round

  @return next lowest natural number
  "
  (bind ?round (round ?x))
  (if (< ?x ?round) then
    (return (- ?round 1))
  )
  (return ?round)
)


(deffunction round-up (?x)
  "Rounds a given number up to the next natural number
  @param ?x number to round

  @return next highest natural number
  "
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


(deffunction get-2d-center (?x1 ?y1 ?x2 ?y2)
  "Calculates the point in the middle of two given points
  @param ?x1 ?y1 coordinates of the first point
  @param ?x2 ?y2 coordinates of the second point

  @return the point exactly in the middle of the two points
  "
  (return (create$ (/ (+ ?x1 ?x2) 2) (/ (+ ?y1 ?y2) 2)))
)


(deffunction laser-line-center-map (?ep1 ?ep2 ?frame ?timestamp)
  (bind ?c (get-2d-center (nth$ 1 ?ep1) (nth$ 2 ?ep1) (nth$ 1 ?ep2) (nth$ 2 ?ep2)))
  (bind ?c3 (nth$ 1 ?c) (nth$ 2 ?c) 0)
  (return (transform-safe "map" ?frame ?timestamp ?c3 (create$ 0 0 0 1)))
)


(deffunction mirror-name (?zn)
  "Gets the name of a zone or mps on the other half on the field
  @param ?zn name of a zone or machine (eg M-CS1 or M-Z22)

  @return name of the corresponding zone on the other half of the field
  "
  (bind ?team (sub-string 1 1 ?zn))
  (bind ?zone (sub-string 3 99 ?zn))
  (if (eq ?team "M") then
    (return (sym-cat "C-" ?zone))
  else
    (return (sym-cat "M-" ?zone))
  )
)


(deffunction want-mirrored-rotation (?mtype ?zone)
  "Checks if a machine has to be mirrored according to the rulebook
  @param ?mtype type of the machine
  @param ?zone zone the machine stands in

  @return TRUE if the rotation of the machine has to be mirrored
  "
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
          )
  )
)


(deffunction mirror-orientation (?mtype ?zone ?ori)
  "Calculates the rotation of a mps on the other half of the field
  @param ?mtype type of the mps
  @param ?zone zone in which the mps is located
  @param ?ori orientation of the mps in that zone

  @return orientation of the given mps on the other half of the field
  "
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
  "Returns the opposite team
  @param ?team the one team

  @return the other team
  "
  (if (eq (sym-cat ?team) CYAN) then
    (return MAGENTA)
  else
    (return CYAN)
  )
)


(deffunction navigator-set-speed (?max-velocity ?max-rotation)
  "Uses the NavigatorInterface to set the max velocity and speed"
  (bind ?msg (blackboard-create-msg "NavigatorInterface::Navigator" "SetMaxVelocityMessage"))
  (blackboard-set-msg-field ?msg "max_velocity" ?max-velocity)
  (blackboard-send-msg ?msg)
  (bind ?msg (blackboard-create-msg "NavigatorInterface::Navigator" "SetMaxRotationMessage"))
  (blackboard-set-msg-field ?msg "max_rotation" ?max-rotation)
  (blackboard-send-msg ?msg)
)


(deffunction zone-is-blocked (?mps-zone ?orientation ?zone ?mps)
  (bind ?type (eval (sub-string 3 4 ?mps)))
  (bind ?x (eval (sub-string 4 4 ?zone)))
  (bind ?mps-x (eval (sub-string 4 4 ?mps-zone)))
  (bind ?y (eval (sub-string 5 5 ?zone)))
  (bind ?mps-y (eval (sub-string 5 5 ?mps-zone)))
  (bind ?side (sub-string 1 1 ?mps-zone))
  (if (eq ?side (sub-string 1 1 ?zone)) then
	  (return FALSE)
  )
  (if (eq (str-compare ?side "C") 0) then
    (if (eq ?orientation 270) then
      (if (and (eq ?y (- ?mps-y 1)) (eq ?x ?mps-x)) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 90) then
      (if (and (eq ?y (+ ?mps-y 1)) (eq ?x ?mps-x)) then
        (return TRUE)
      )
    )

    (if (eq ?orientation 0) then
      (if (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y)) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 180) then
      (if (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y)) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 315) then
      (if (or (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y))
              (and (eq ?x (+ ?mps-x 1)) (eq ?y (- ?mps-y 1)))
              (and (eq ?x ?mps-x) (eq ?y (- ?mps-y 1)))) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 45) then
      (if (or (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y))
              (and (eq ?x (+ ?mps-x 1)) (eq ?y (+ ?mps-y 1)))
              (and (eq ?x ?mps-x) (eq ?y (+ ?mps-y 1)))) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 135) then
      (if (or (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y))
              (and (eq ?x (- ?mps-x 1)) (eq ?y (+ ?mps-y 1)))
              (and (eq ?x ?mps-x) (eq ?y (+ ?mps-y 1)))) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 225) then
      (if (or (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y))
              (and (eq ?x (- ?mps-x 1)) (eq ?y (- ?mps-y 1)))
              (and (eq ?x ?mps-x) (eq ?y (- ?mps-y 1)))) then
        (return TRUE)
      )
    )
    " Block output side as well "
    (if (or (eq (str-compare ?type "BS") 0)
            (eq (str-compare ?type "CS") 0)
            (eq (str-compare ?type "RS") 0)
            (eq (str-compare ?type "SS") 0)) then
      (if (eq ?orientation 270) then
        (if (and (eq ?y (+ ?mps-y 1)) (eq ?x ?mps-x)) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 90) then
        (if (and (eq ?y (- ?mps-y 1)) (eq ?x ?mps-x)) then
          (return TRUE)
        )
      )

      (if (eq ?orientation 0) then
        (if (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y)) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 180) then
        (if (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y)) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 315) then
        (if (or (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y))
                (and (eq ?x (- ?mps-x 1)) (eq ?y (- ?mps-y 1)))
                (and (eq ?x ?mps-x) (eq ?y (+ ?mps-y 1)))) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 45) then
        (if (or (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y))
                (and (eq ?x (- ?mps-x 1)) (eq ?y (+ ?mps-y 1)))
                (and (eq ?x ?mps-x) (eq ?y (- ?mps-y 1)))) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 135) then
        (if (or (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y))
                (and (eq ?x (+ ?mps-x 1)) (eq ?y (+ ?mps-y 1)))
                (and (eq ?x ?mps-x) (eq ?y (- ?mps-y 1)))) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 225) then
        (if (or (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y))
                (and (eq ?x (+ ?mps-x 1)) (eq ?y (- ?mps-y 1)))
                (and (eq ?x ?mps-x) (eq ?y (+ ?mps-y 1)))) then
          (return TRUE)
        )
      )

    )

  else
    (if (eq ?orientation 270) then
      (if (and (eq ?y (- ?mps-y 1)) (eq ?x ?mps-x)) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 90) then
      (if (and (eq ?y (+ ?mps-y 1)) (eq ?x ?mps-x)) then
        (return TRUE)
      )
    )

    (if (eq ?orientation 0) then
      (if (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y)) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 180) then
      (if (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y)) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 315) then
      (if (or (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y))
              (and (eq ?x (- ?mps-x 1)) (eq ?y (- ?mps-y 1)))
              (and (eq ?x ?mps-x) (eq ?y (- ?mps-y 1)))) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 45) then
      (if (or (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y))
              (and (eq ?x (- ?mps-x 1)) (eq ?y (+ ?mps-y 1)))
              (and (eq ?x ?mps-x) (eq ?y (+ ?mps-y 1)))) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 135) then
      (if (or (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y))
              (and (eq ?x (+ ?mps-x 1)) (eq ?y (+ ?mps-y 1)))
              (and (eq ?x ?mps-x) (eq ?y (+ ?mps-y 1)))) then
        (return TRUE)
      )
    )
    (if (eq ?orientation 225) then
      (if (or (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y))
              (and (eq ?x (+ ?mps-x 1)) (eq ?y (- ?mps-y 1)))
              (and (eq ?x ?mps-x) (eq ?y (- ?mps-y 1)))) then
        (return TRUE)
      )
    )
    (if (or (eq (str-compare ?type "BS") 0)
            (eq (str-compare ?type "CS") 0)
            (eq (str-compare ?type "RS") 0)) then

      (if (eq ?orientation 270) then
        (if (and (eq ?y (+ ?mps-y 1)) (eq ?x ?mps-x)) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 90) then
        (if (and (eq ?y (- ?mps-y 1)) (eq ?x ?mps-x)) then
          (return TRUE)
        )
      )

      (if (eq ?orientation 0) then
        (if (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y)) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 180) then
        (if (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y)) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 315) then
        (if (or (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y))
                (and (eq ?x (+ ?mps-x 1)) (eq ?y (- ?mps-y 1)))
                (and (eq ?x ?mps-x) (eq ?y (+ ?mps-y 1)))) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 45) then
        (if (or (and (eq ?x (+ ?mps-x 1)) (eq ?y ?mps-y))
                (and (eq ?x (+ ?mps-x 1)) (eq ?y (+ ?mps-y 1)))
                (and (eq ?x ?mps-x) (eq ?y (- ?mps-y 1)))) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 135) then
        (if (or (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y))
                (and (eq ?x (- ?mps-x 1)) (eq ?y (+ ?mps-y 1)))
                (and (eq ?x ?mps-x) (eq ?y (- ?mps-y 1)))) then
          (return TRUE)
        )
      )
      (if (eq ?orientation 225) then
        (if (or (and (eq ?x (- ?mps-x 1)) (eq ?y ?mps-y))
                (and (eq ?x (- ?mps-x 1)) (eq ?y (- ?mps-y 1)))
                (and (eq ?x ?mps-x) (eq ?y (+ ?mps-y 1)))) then
          (return TRUE)
        )
      )

    )
  )
  (return FALSE)
)


(deffunction wait-pos (?mps ?side)
" @param ?mps  machine name
  @param ?side machine side

  @return Symbol of the waiting position corresponding to the given mps side
"
	(bind ?prefix (sym-cat WAIT- ?mps))
  (switch ?side
    (case INPUT then (return (sym-cat ?prefix - I)))
    (case OUTPUT then (return (sym-cat ?prefix - O)))
		(default
      (printout error "wait-pos input " ?side " is not a valid side
                       (allowed values: INPUT,OUTPUT)" crlf)
      (return (sym-cat ?prefix - I)))
  )
)


(deffunction sym-to-int (?sym)
" @param ?sym domain representation of a number (ZERO|...|SEVEN)

  @return Integer value described in ?sym, or 0 if ?sym is not recognized
"
  (if (eq ?sym ZERO) then (return 0))
  (if (eq ?sym ONE) then (return 1))
  (if (eq ?sym TWO) then (return 2))
  (if (eq ?sym THREE) then (return 3))
  (if (eq ?sym FOUR) then (return 4))
  (if (eq ?sym FIVE) then (return 5))
  (if (eq ?sym SIX) then (return 6))
  (if (eq ?sym SEVEN) then (return 7))
  (printout error "sym-to-int input " ?sym " is not a valid domain number
                   (allowed values: ZERO,...,SEVEN)" crlf)
  (return 0)
)

(deffunction int-to-sym (?int)
" @param ?int Number as Integer (0-7)

  @return Domain representation of number (ZERO|ONE|TWO|THREE)
"
  (switch ?int
    (case 0 then
      (return ZERO))
    (case 1 then
      (return ONE))
    (case 2 then
      (return TWO))
    (case 3 then
      (return THREE))
    (case 4 then
      (return FOUR))
    (case 5 then
      (return FIVE))
    (case 6 then
      (return SIX))
    (case 7 then
      (return SEVEN))
  )
  (return nil)
)

(deffunction bool-to-int (?bool)
" @param ?bool boolean

  @return Integer 1 if ?bool is true, else 0
"
  (if (eq ?bool TRUE) then (return 1) else (return 0))
)


(deffunction last-ring-points (?com)
" @param ?com complexity of an order

  @return returns points awarded for mounting the last ring of an order with
          complexity ?com
"
  (if (eq ?com C0) then (return 0))
  (if (eq ?com C1) then (return ?*POINTS-MOUNT-LAST-RING-C1*))
  (if (eq ?com C2) then (return ?*POINTS-MOUNT-LAST-RING-C2*))
  (if (eq ?com C3) then (return ?*POINTS-MOUNT-LAST-RING-C3*))
  (printout error "last-ring-points input " ?com " is not a valid complexity
                   (allowed values: C0,C1,C2,C3)" crlf)
  (return 0)
)


(deffunction ring-req-points (?req)
" @param ?req number of additional bases needed for a ring

  @return points awarded for mounting a ring needing ?req additinal bases
"
  (if (eq ?req ZERO) then (return ?*POINTS-MOUNT-RING-CC0*))
  (if (eq ?req ONE) then (return ?*POINTS-MOUNT-RING-CC1*))
  (if (eq ?req TWO) then (return ?*POINTS-MOUNT-RING-CC2*))
  (printout error "ring-req-points input " ?req " is not a valid ring spec
                   (allowed values: ZERO,ONE,TWO)" crlf)
  (return 0)
)

(deffunction values-from-name-value-list ($?args)
" @param $?args a list containing name value pairs. E.g. (m C-CS1 s IDLE)

  @return The values of the argument list as list. E.g (C-CS1 IDLE)
"
  (bind ?values (create$))
  (foreach ?a ?args
    (if (eq 0 (mod ?a-index 2)) then
      (bind ?values (create$ ?values ?a))
    )
  )
  (return ?values)
)

(deffunction order-steps-index (?step)
" @param ?step production step, from the agent view describing the progress
               after a production goal is successfully executed.

  @return index of ?step in the list of points-steps and estimated-time-steps
"
  (if (eq ?step RING1) then (return 1))
  (if (eq ?step RING2) then (return 2))
  (if (eq ?step RING3) then (return 3))
  (if (eq ?step CAP) then (return 4))
  (if (eq ?step DELIVER) then (return 5))
  (printout error "order-steps-index input " ?step " is not a valid step
                   (allowed values: RING1,RING2,RING3,CAP,DELIVER)" crlf)
)


(deffunction delivery-points (?qr ?qd-us ?qd-them ?competitive ?curr-time ?deadline)
" @param ?qr quantities-requested of the order in question
  @param ?qd-us quantities-delivered of our team
  @param ?qd-them quantities-delivered of the opposing team
  @param ?competitive bool indicating whether competitive point changes should
                      be applied
  @param ?curr-time current game time in seconds
  @param ?deadline deadline of the order

  @return delivery points (apply competitive rules if needed, 0 points if
          the requested quantities are already delivered, reduced points after)
"
  (if (not (> ?qr ?qd-us))
    then
      (return 0))
  (bind ?delivery-points 20)
  (if (> ?curr-time ?deadline)
    then
      (bind ?delivery-points 5)
  )
  (if ?competitive
    then
      (if (> ?qd-them ?qd-us)
        then
          (return (max 0 (- ?delivery-points ?*POINTS-COMPETITIVE*)))
        else
          (return (+ ?delivery-points ?*POINTS-COMPETITIVE*)))
    else
      (return ?delivery-points)
  )
)


(deffunction greedy-knapsack (?goods ?initial-weight ?max-weight)
" @params ?goods list of weights to pack in knapsack
          ?initial-weight weight of the empty knapsack
          ?max-weight upper bound on total weight that fits in the knapsack

  @return number of items that can be greedily packed into the knapsack
"
  (bind ?curr-weight ?initial-weight)
  (bind ?counter 0)
  (progn$ (?item ?goods)
    (if (<= (+ ?curr-weight ?item) ?max-weight)
      then
        (bind ?curr-weight (+ ?curr-weight ?item))
        (bind ?counter (+ ?counter 1))
      else
        (return ?counter)
    )
  )
  (return ?counter)
)


(deffunction estimate-achievable-points
             (?pointlist ?achieved-points ?timelist ?curr-time ?deadline ?next-step)
" @params ?pointlist list of points for all production steps
          ?achieved-points points already scored in previous production steps
          ?timelist list of time-estimates for all production steps
          ?curr-time current game time in seconds
          ?deadline deadline (in seconds) for the order that gets produced
          ?next-step next step that scores points

  @return Amount of points the product yields, assuming tasks only score points
          if they are finished within the deadline
"
  (bind ?curr-step (order-steps-index ?next-step))
  (if (<= ?curr-step (length ?pointlist))
    then
      (bind ?remaining-timelist (subseq$ ?timelist
                                         ?curr-step
                                         (length ?timelist)))
      (bind ?doable-steps (greedy-knapsack ?remaining-timelist
                                           ?curr-time
                                           ?deadline))
      (bind ?achievable-pointlist (subseq$ ?pointlist
                                           ?curr-step
                                           (+ ?curr-step (- ?doable-steps 1))))
      (bind ?achievable-points ?achieved-points)
      (progn$ (?points ?achievable-pointlist)
            (bind ?achievable-points (+ ?achievable-points ?points))
      )
      (return ?achievable-points)
    else
      (return ?achieved-points)
  )
)

(deffunction node-distance (?node)
" @param ?node Name of a navgraph node

  @return Euclidean distance between robots position and node
          -1 if any of the two positions can not be found.
"
  (if (not (do-for-fact ((?nn navgraph-node)) (eq ?nn:name (str-cat ?node))
        (bind ?xn (nth$ 1 ?nn:pos))
        (bind ?yn (nth$ 2 ?nn:pos))
      ))
    then
      (return -1)
  )
  (if (not (do-for-fact ((?pos Position3DInterface)) (eq ?pos:id "Pose")
        (bind ?xp (nth$ 1 ?pos:translation))
        (bind ?yp (nth$ 2 ?pos:translation))
      ))
    then
      (return -1)
  )
  (return (distance ?xn ?yn ?xp ?yp))
)

(deffunction goal-distance-prio (?dist)
" @param The distance between the robot and a target position of a goal

  @return A value between 0 and 1 based on the distance
"
  (return (- 1 (/ ?dist ?*MAX-DISTANCE*)))
)
