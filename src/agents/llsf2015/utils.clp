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
  (delayed-do-for-all-facts ((?ft found-tag)) (not ?ft:already-added)
    (bind ?any-tag-to-add TRUE)
    ; report tag position to navgraph generator
    (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "UpdateStationByTagMessage"))
    (blackboard-set-msg-field ?msg "id" (str-cat ?ft:name))
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
   
    (modify ?ft (already-added TRUE))
  )
  (if ?any-tag-to-add
    then
    ; send message which zones are still to explore
    (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "SetExplorationZonesMessage"))
    (bind $?zones-still-to-explore-multifield (create-multifield-with-length 24))
    (do-for-all-facts ((?zone-to-explore zone-exploration)) TRUE
    ;get index of zone (e.g. Z15 -> 15)
      (bind ?zone-name (str-cat ?zone-to-explore:name))
      (bind ?zone-index (eval (sub-string 2 (str-length ?zone-name) ?zone-name)))
      (bind ?zones-still-to-explore-multifield
	    (replace$ ?zones-still-to-explore-multifield ?zone-index ?zone-index (sym-cat ?zone-to-explore:still-to-explore)))
    )
    (blackboard-set-msg-multifield ?msg "zones" ?zones-still-to-explore-multifield)
    (blackboard-send-msg ?msg)

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