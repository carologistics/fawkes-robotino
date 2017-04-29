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

(deffunction distance-mf (?p1 ?p2)
  (return (distance (nth$ 1 ?p1) (nth$ 2 ?p1) (nth$ 1 ?p2) (nth$ 2 ?p2)))
)

(deffunction is-working ($?out-of-order)
  "Check if a machine is not out of order"
  (return (eq (nth$ 1 ?out-of-order) 0))
)

(deffunction random-id ()
  "Return a random task id"
  (return (random 0 1000000000))
)

(deffunction get-input (?mps)
  "Return the navgraph point of the input side of the given mps"
  (return (str-cat ?mps "-I"))
)

(deffunction get-output (?mps)
  "Return the navgraph point of the output side of the given mps"
  (return (str-cat ?mps "-O"))
)

(deffunction get-light-signal-side (?mps)
  "Return the navgraph point of the side where the light-signal is mounted"
  (if (any-factp ((?machine machine)) (and (eq ?machine:name ?mps)
					   (eq ?machine:mtype DS)))
    then
    ; for DS
    (return (get-input ?mps))
    else
    ; for all other mps
    (return (get-output ?mps))
  )
)

(deffunction iface-get-idx (?iface-id)
  "Return the index of BB interface names of the form /LaserLines/1 or /LaserLines/1/moving_avg"
  (bind ?id (sub-string 2 1024 ?iface-id))
  (bind ?id (sub-string (+ 1 (str-index "/" ?id)) 1024 ?id))
  (if (str-index "/" ?id) then
    (bind ?id (sub-string 1 (- (str-index "/" ?id) 1) ?id))
  )
  (return (string-to-field ?id))
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


(deffunction create-multifield-with-length-and-entry (?length ?entry)
  "Creates a Multifield with ?length times the entry ?entry"
  (bind $?res (create$ ))
  (loop-for-count (?i 1 24) do
    (bind $?res (insert$ ?res 1 ?entry))
  )
  (return ?res)
)

(deffunction create-multifield-with-length (?length)
  "Creates a Multifield with length ?length"
  (return (create-multifield-with-length-and-entry ?length RANDOM-SYMBOL))
)

(deffunction navgraph-add-all-new-tags ()
  "send all new tags to the navgraph generator"
  (bind ?any-tag-to-add FALSE)
  (delayed-do-for-all-facts ((?ft found-tag)) (not (any-factp ((?added-f navgraph-added-for-mps)) (eq ?ft:name ?added-f:name)))
    (bind ?any-tag-to-add TRUE)
    ; report tag position to navgraph generator
    (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "UpdateStationByTagMessage"))
    (blackboard-set-msg-field ?msg "name" (str-cat ?ft:name))
    (blackboard-set-msg-field ?msg "side" ?ft:side)
    (blackboard-set-msg-field ?msg "frame" ?ft:frame)
    (blackboard-set-msg-multifield ?msg "tag_translation" ?ft:trans)
    (blackboard-set-msg-multifield ?msg "tag_rotation" ?ft:rot)
    (blackboard-send-msg ?msg)
    (printout t "Send UpdateStationByTagMessage: id " (str-cat ?ft:name) 
	      " side " ?ft:side
	      " frame " ?ft:frame
	      " trans " ?ft:trans
	      " rot " ?ft:rot  crlf)
   
    (assert (navgraph-added-for-mps (name ?ft:name)))
  )
  (if ?any-tag-to-add
    then
    ; send compute message so we can drive to the output
    (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "ComputeMessage"))
    (bind ?compute-msg-id (blackboard-send-msg ?msg))

    ; save the last compute-msg-id to know when it was processed
    (do-for-all-facts ((?lncm last-navgraph-compute-msg)) TRUE
      (retract ?lncm)
    )
    (assert (last-navgraph-compute-msg (id ?compute-msg-id)))
    else
    (printout t "There are no tags to add" crlf)
  )
)

(deffunction navigator-set-speed (?max-velocity ?max-rotation)
  (bind ?msg (blackboard-create-msg "NavigatorInterface::Navigator" "SetMaxVelocityMessage"))
  (blackboard-set-msg-field ?msg "max_velocity" ?max-velocity)
  (blackboard-send-msg ?msg)
  (bind ?msg (blackboard-create-msg "NavigatorInterface::Navigator" "SetMaxRotationMessage"))
  (blackboard-set-msg-field ?msg "max_rotation" ?max-rotation)
  (blackboard-send-msg ?msg)
)

(deffunction utils-remove-prefix (?string ?prefix)
  "Removes a prefix from a string or symbol by its length"
  (bind ?res (sub-string (+ 1 (str-length (str-cat ?prefix))) 
			 (str-length (str-cat ?string))
			 (str-cat ?string)))
  (if (eq (type ?string) SYMBOL) then
    (return (sym-cat ?res))
    else
    (return ?res)
  )
)

(deffunction utils-get-2d-center (?x1 ?y1 ?x2 ?y2)
  (return (create$ (/ (+ ?x1 ?x2) 2) (/ (+ ?y1 ?y2) 2)))
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

(deffunction string-from-multifield ($?mf)
  (bind ?res "")
  (progn$ (?f ?mf)
    (bind ?res (str-cat ?res ?f ","))
  )
)

(deffunction utils-get-zone-edges (?zone)
  "returns the edges of a zone"
  ; compute cyan edges first and mirrow them later if neccessary
  (bind ?zone (eval (sub-string 2 (str-length (str-cat ?zone)) (str-cat ?zone))))
  (bind ?zone-cyan ?zone)
  (if (> ?zone-cyan 12) then
    (bind ?zone-cyan (- ?zone 12))
  )
  (bind ?zone-x (round-down (/ (- ?zone-cyan 1) 4)))
  (bind ?zone-y (mod (- ?zone-cyan 1) 4))
  
  (bind ?y-min (* ?zone-y ?*ZONE-HEIGHT*))
  (bind ?y-max (* (+ ?zone-y 1) ?*ZONE-HEIGHT*))

  (bind ?x-min (* ?zone-x ?*ZONE-WIDTH*))
  (bind ?x-max (* (+ ?zone-x 1) ?*ZONE-WIDTH*))
  
  (if (> ?zone 12) then
    (bind ?x-min (- 0 ?x-min))
    (bind ?x-max (- 0 ?x-max))
  )
  (return (create$ ?x-min ?x-max ?y-min ?y-max))
)

(deffunction utils-get-tags-str-still-to-explore (?team)
  ; get string with all tags we still need to explore
  (bind ?search-tags "{")
  (delayed-do-for-all-facts ((?tag tag-matching)) (and (eq ?tag:team ?team)
                                                       (not (any-factp ((?found found-tag)) (eq ?found:name ?tag:machine))))
    (bind ?search-tags (str-cat ?search-tags ?tag:tag-id ","))
  )
  (bind ?search-tags (str-cat ?search-tags "}"))
  (return ?search-tags)
)

(deffunction escape-if-string (?value)
  ; this function ads \" \" around a string to prevent transforming the string value
  ; into a Symbol when using the eval function
  (if (eq STRING (type ?value)) then
    (return (str-cat "\"" ?value "\""))
  )
  (return ?value)
)

(deffunction dyn-mod (?f $?arg) ;arg contains slot and value pairs
  ; dynamic modify function that takes the fact-adress, the slot name and the new value
  ; the modificytion is done by creating a new assert string and removing the old fact
  ; returns the adress of the modified fact, ?f is no longer usable
  (if (neq (mod (length$ ?arg) 2) 0) then
    (printout error "dyn-mod needs to be called with slot-value pairs" crlf)
    (return)
  )
  (bind ?slots-to-change (create$))
  (bind ?values-to-set (create$))
  (progn$ (?field ?arg)
    (if (eq (mod ?field-index 2) 1)
      then
      (bind ?slots-to-change (insert$ ?slots-to-change 1 ?field))
      else
      (bind ?values-to-set (insert$ ?values-to-set 1 ?field))
    )
  )
  ; dont modify if set to current value
  (bind ?different-slots (create$))
  (bind ?different-values (create$))
  (progn$ (?slot ?slots-to-change)
    (if (neq (fact-slot-value ?f ?slot) (nth$ ?slot-index ?values-to-set)) then
      (bind ?different-slots (insert$ ?different-slots 1 ?slot))
      (bind ?different-values (insert$ ?different-values 1 (nth$ ?slot-index ?values-to-set)))
    )
  )
  (bind ?slots-to-change ?different-slots)
  (bind ?values-to-set ?different-values)
  ; return if there is nothing to modify
  (if (eq 0 (length$ ?slots-to-change)) then
    (return ?f)
  )
  (bind ?acom (str-cat "(assert (" (fact-relation ?f) " "))
  (progn$ (?slot (fact-slot-names ?f))
    (if (not (member$ ?slot ?slots-to-change))
      then
      (if (deftemplate-slot-multip (fact-relation ?f) ?slot)
        then ;copy multifield
        (bind ?acom (str-cat ?acom "(" ?slot " (create$ " 
                             (implode$ (fact-slot-value ?f ?slot))
                             "))"))
        else ;copy singlefield
        (bind ?acom (str-cat ?acom "(" ?slot " " 
                             (escape-if-string (fact-slot-value ?f ?slot)) ")"))
      )
      else
      (bind ?acom (str-cat ?acom "(" ?slot " "
                           (escape-if-string (nth$ (member$ ?slot ?slots-to-change) ?values-to-set))
                           ")"))
    )
  )
  (bind ?acom (str-cat ?acom "))"))
  ;(printout t ?acom crlf)
  (retract ?f)
  (bind ?res (eval ?acom))
  (if (not ?res) then
    (printout error "Assert failed : " ?acom crlf)
  )
  (return ?res)
)

(deffunction dyn-add-to-multifield (?f ?slot ?value)
  ; dynamic modify function that takes the fact-adress, the slot name and
  ; the new value that should be added to the multifield in the slot
  ; the modificytion is done by creating a new assert string and removing the old fact
  ; returns the adress of the modified fact, ?f is no longer usable
  (if (not (deftemplate-slot-multip (fact-relation ?f) ?slot))
    then
    (printout error "Slot " ?slot " is no multifield!" crlf)
  )
  
  (bind ?acom (str-cat "(assert (" (fact-relation ?f) " "))
  (progn$ (?cur-slot (fact-slot-names ?f))
    (if (neq ?slot ?cur-slot)
      then
      (if (deftemplate-slot-multip (fact-relation ?f) ?cur-slot)
        then ;coply multifield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " (create$ " 
                             (implode$ (fact-slot-value ?f ?cur-slot))
                             "))"))
        else ;copy singlefield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " "
                             (escape-if-string (fact-slot-value ?f ?cur-slot)) ")"))
      )
      else
      (bind ?acom (str-cat ?acom "(" ?slot " "
                           "(create$ "
                           (implode$ (fact-slot-value ?f ?cur-slot))
                           " " (escape-if-string ?value) ")"
                           ")"))
    )
  )
  (bind ?acom (str-cat ?acom "))"))
  ;(printout t ?acom crlf)
  (retract ?f)
  (bind ?res (eval ?acom))
  (if (not ?res) then
    (printout error "Assert failed : " ?acom crlf)
  )
  (return ?res)
)

(deffunction dyn-remove-from-multifield (?f ?slot ?value)
  ; dynamic modify function that takes the fact-adress, the slot name and
  ; the value that should be removed from the multifield in the slot
  ; the modificytion is done by creating a new assert string and removing the old fact
  ; returns the adress of the modified fact, ?f is no longer usable
  (if (not (deftemplate-slot-multip (fact-relation ?f) ?slot))
    then
    (printout error "Slot " ?slot " is no multifield!" crlf)
  )
  (bind ?acom (str-cat "(assert (" (fact-relation ?f) " "))
  (progn$ (?cur-slot (fact-slot-names ?f))
    (if (neq ?slot ?cur-slot)
      then
      (if (deftemplate-slot-multip (fact-relation ?f) ?cur-slot)
        then ;coply multifield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " (create$ "
                             (implode$ (fact-slot-value ?f ?cur-slot))
                             "))"))
        else ;copy singlefield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " "
                             (escape-if-string (fact-slot-value ?f ?cur-slot)) ")"))
      )
      else
      (bind ?acom (str-cat ?acom "(" ?slot " "
                           "(create$ "
                           (implode$ (delete-member$ (fact-slot-value ?f ?cur-slot) ?value))
                           ")"
                           ")"))
    )
  )
  (bind ?acom (str-cat ?acom "))"))
  ;(printout t ?acom crlf)
  (retract ?f)
  (bind ?res (eval ?acom))
  (if (not ?res) then
    (printout error "Assert failed : " ?acom crlf)
  )
  (return ?res)
)

(deffunction dyn-override-multifield (?f ?slot $?multifield)
  ; dynamic modify function that takes the fact-adress, the slot name and
  ; the new list
  ; the modificytion is done by creating a new assert string and removing the old fact
  ; returns the adress of the modified fact, ?f is no longer usable
  (if (not (deftemplate-slot-multip (fact-relation ?f) ?slot))
    then
    (printout error "Slot " ?slot " is no multifield!" crlf)
  )
  ; skip overriding if there is no change
  (if (eq (string-from-multifield ?multifield)
          (string-from-multifield (fact-slot-value ?f ?slot))) then
    (return ?f)
  )
  (bind ?acom (str-cat "(assert (" (fact-relation ?f) " "))
  (progn$ (?cur-slot (fact-slot-names ?f))
    (if (neq ?slot ?cur-slot)
      then
      (if (deftemplate-slot-multip (fact-relation ?f) ?cur-slot)
        then ;coply multifield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " (create$ " 
                             (implode$ (fact-slot-value ?f ?cur-slot))
                             "))"))
        else ;copy singlefield
        (bind ?acom (str-cat ?acom "(" ?cur-slot " "
                             (escape-if-string (fact-slot-value ?f ?cur-slot)) ")"))
      )
      else
      (bind ?acom (str-cat ?acom "(" ?slot " "
                           "(create$ " (implode$ ?multifield) ")"
                           ")"))
    )
  )
  (bind ?acom (str-cat ?acom "))"))
  ;(printout t ?acom crlf)
  (retract ?f)
  (bind ?res (eval ?acom))
  (if (not ?res) then
    (printout error "Assert failed : " ?acom crlf)
  )
  (return ?res)
)

(deffunction dyn-assert (?fact)
  ; assert a fact from a string
  (bind ?assertstr (str-cat "(assert " ?fact ")"))
  (bind ?res (eval ?assertstr))
  (if (not ?res) then
    (printout error "Assert failed : " ?fact crlf)
  )
  (return ?res)
)

(deffunction laser-line-get-center (?iface-id ?timestamp)
  "Return the center of a laser line in map coordinates"
  (bind ?idx (iface-get-idx ?iface-id))
  (bind ?frame1 (str-cat "laser_line_" ?idx "_e1"))
  (bind ?frame2 (str-cat "laser_line_" ?idx "_e2"))
  (bind ?ep2-from-ep1 (transform-safe ?frame1 ?frame2 ?timestamp
                                      (create$ 0 0 0) (create$ 0 0 0 1)))
  (if (not ?ep2-from-ep1) then
    (return FALSE)
  )

  (bind ?mid-from-ep1
    (/ (nth$ 1 ?ep2-from-ep1) 2)
    (/ (nth$ 2 ?ep2-from-ep1) 2)
    (/ (nth$ 3 ?ep2-from-ep1) 2)
  )
  (bind ?mid-map (transform-safe "map" ?frame1 ?timestamp ?mid-from-ep1 (create$ 0 0 0 1)))
  (return ?mid-map)
)


(deffunction zone-center (?zn)
  (bind ?x (eval (sub-string 4 4 ?zn)))
  (bind ?y (eval (sub-string 5 5 ?zn)))
  (if (eq (sub-string 1 1 ?zn) "M") then
    (bind ?x (* -1 ?x))
  )
  (return (create$ (- ?x 0.5) (- ?y 0.5)))
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


(deffunction mirror-trans ($?trans)
  (return (create$
    (- 0 (nth$ 1 ?trans))
    (nth$ 2 ?trans)
    (nth$ 3 ?trans)
  ))
)


(deffunction at-wall (?x ?y)
  (bind ?xy (str-cat ?x ?y))
  (return
    (or
      (eq ?x 7) (eq ?x -7) (eq ?y 8) (eq ?y 1)
      (member$ ?xy (create$ "-72" "-62" "62" "72"))
    )
  )
)


(deffunction protobuf-name (?zone)
  (return
    (str-cat (sub-string 1 1 ?zone) "_" (sub-string 3 99 ?zone))
  )
)


(deffunction want-mirrored-rotation (?mtype ?zone)
"According to the RCLL2017 rulebook, this is when a machine is mirrored"
  (bind ?zn (str-cat ?zone))
  (bind ?x (eval (sub-string 4 4 ?zn)))
  (if (eq (sub-string 1 1 ?zn) "M") then
    (bind ?x (* -1 ?x))
  )
  (bind ?y (eval (sub-string 5 5 ?zn)))

  (return (or (member$ ?mtype (create$ BS DS SS))
              (not (at-wall ?x ?y))
              (and (!= ?y 8) (!= ?y 1))
  ))
)


(deffunction mirror-rot (?mtype ?zone $?rot)
"Mirror rotation according to rules, $?rot is a quaternion"
  (if (want-mirrored-rotation ?mtype ?zone) then
    (bind ?yaw (tf-yaw-from-quat ?rot))
    (bind ?yaw-mirror (+ (- 0 (- ?yaw ?*PI-HALF*)) ?*PI-HALF*))
    (if (> ?yaw-mirror ?*PI*) then
      (bind ?yaw-mirror (- ?yaw-mirror ?*2PI*)))
    (if (< ?yaw-mirror (- 0 ?*PI*)) then
      (bind ?yaw-mirror (+ ?yaw-mirror ?*2PI*)))
    (return (tf-quat-from-yaw ?yaw-mirror))
  else
    (return ?rot)
  )
)


(deffunction mirror-team (?team)
  (if (eq (sym-cat ?team) CYAN) then
    (return MAGENTA)
  else
    (return CYAN)
  )
)


(deffunction mirror-orientation (?mtype ?zone ?ori)
  (if (want-mirrored-rotation ?mtype ?zone) then
    (if (eq (sub-string 1 1 ?zone) "C") then
      (do-for-fact ((?mo mirror-orientation)) (eq ?mo:cyan ?ori)
        (bind ?m-ori ?mo:magenta)
      )
    else
      (do-for-fact ((?mo mirror-orientation)) (eq ?mo:magenta ?ori)
        (bind ?m-ori ?mo:cyan)
      )
    )
    (return ?m-ori)
  else
    (return ?ori)
  )
)


