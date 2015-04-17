;---------------------------------------------------------------------------
;  navgraph.clp - using the navgraph-feature
;
;  Created: Sat Mar 22 13:46:30 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


(defrule navgraph-get-exp-coordinates
  "Extract position and orientation of the explorations points from the navgraph"
  (declare (salience ?*PRIORITY-WM*))
  ?nn <- (navgraph-node (name ?name&:(str-index "Exp" (str-cat ?name)))
			(pos $?pos) (properties $?props))
  (not (pose (name ?name)))
  =>
  ;check if the node has the orientation property for any zone
  (delayed-do-for-all-facts ((?ze zone-exploration)) (navgraph-has-property ?props (str-cat "orientation_" ?ze:name))
    ; add pose to the zone
    (bind ?pose-id (random-id))
    (assert (pose (id ?pose-id) (name ?name)
		  (x (nth$ 1 ?pos)) (y (nth$ 2 ?pos))
		  (ori (navgraph-property-as-float ?props (str-cat "orientation_" ?ze:name)))))
    (modify ?ze (look-pos (insert$ ?ze:look-pos 1 ?pose-id)))
  )
  ; we don't need this fact again
  (retract ?nn)
)

(defrule navgraph-set-exploration-zone-coordinates
  "Set the coordinates of the exploration zones according to the assigned exploration points"
  (declare (salience ?*PRIORITY-WM*))
  ?ze <- (zone-exploration (x 0.0) (y 0.0) (look-pos $?lp&:(> (length$ ?lp) 0)))
  =>
  ;find the corresponding position fact
  (do-for-fact ((?pose pose)) (eq ?pose:id (nth$ 1 ?lp))
    (modify ?ze (x ?pose:x) (y ?pose:y))
  )
)

(defrule navgraph-remove-navgraph-facts ;to reduce amount of facts in clips-webview
  (declare (salience ?*PRIORITY-WM-LOW*))
  (navgraph-node)
  =>
  (navgraph-cleanup)
)

(defrule navgraph-add-new-tag-exploration
  "We got a tag from another bot (all tags from us are processed in exploration.clp with higher priority). Add the tag directly. I hope this doesn't break anything when the robot is driving"
  (declare (salience ?*PRIORITY-LOW*))
  ?ft <- (found-tag (name ?machine) (side ?side) (frame ?frame)
		    (trans $?trans) (rot $?rot) (already-added FALSE))
  (tag-matching (tag-id ?tag) (machine ?machine) (side ?side))
  ; TODO: check in which zone the machine is located to mark that the tag in this zone was found
  =>
  (printout t "Add Tag Nr." ?tag " (" ?machine " " ?side 
	    ") we got from another bot to Navgraph-generation"  crlf)
  (printout warn "TODO: check which zone contains the machine, so we don't try to find a tag there again"  crlf)
  
  (navgraph-add-all-new-tags)
)