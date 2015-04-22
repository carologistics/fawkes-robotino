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

(defrule navgraph-remove-navgraph-nodes ;to reduce amount of facts in clips-webview
  (declare (salience ?*PRIORITY-WM-LOW*))
  ?nn <- (navgraph-node (name ?name&~:(or (eq "C-" (sub-string 1 2 (str-cat ?name)))
                                   (eq "M-" (sub-string 1 2 (str-cat ?name))))))
  =>
  (retract ?nn)
)

(defrule navgraph-remove-navgraph-edges ;to reduce amount of facts in clips-webview
  (declare (salience ?*PRIORITY-WM-LOW*))
  ?ne <- (navgraph-edge)
  =>
  (retract ?ne)
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

(defrule navgraph-get-zone-of-found-tag
  "For new found tags that were added to the navgraph, we want to check the zone where the machine is standing for the exploration report"
  (declare (salience ?*PRIORITY-WM*))
  (machine-to-find-zone-of ?mps)
  (found-tag (name ?mps) (side ?side) (frame ?frame)
             (trans $?trans) (rot $?rot))
  ; ?lncm <- (last-navgraph-compute-msg (id ?compute-msg-id))
  ; ?ngg-if <- (NavGraphWithMPSGeneratorInterface (id "/navgraph-generator-mps") (msgid ?compute-msg-id) (final TRUE))
  (navgraph-node (name ?input&:(eq ?input (str-cat ?mps "-I")))
                 (pos $?pos-i))
  (navgraph-node (name ?output&:(eq ?output (str-cat ?mps "-O")))
                 (pos $?pos-o))
  (team-color ?team-color)
  (goalmachine ?zone-intended)
  =>
  ; Find zone by center of the mps
  (bind ?center (utils-get-2d-center (nth$ 1 ?pos-i) (nth$ 2 ?pos-i) 
                                     (nth$ 1 ?pos-o) (nth$ 2 ?pos-o)))
  (bind ?zone-y (round-down (/ (nth$ 2 ?center) ?*ZONE-HEIGHT*)))
  (bind ?cyan-x (nth$ 1 ?center))
  (if (< ?cyan-x 0) then
    (bind ?cyan-x (- 0 ?cyan-x))
  )
  (bind ?zone-cyan-x (round-down (/ ?cyan-x ?*ZONE-WIDTH*)))
  (bind ?zone (+ 1 ?zone-y (* 4 ?zone-cyan-x)))
  (if (< (nth$ 1 ?center) 0) then
    (bind ?zone (+ ?zone 12))
  )
  
  (printout t "mps " ?mps " is in zone " ?zone crlf)

  (do-for-fact ((?ze zone-exploration)) (eq ?ze:name (sym-cat Z ?zone))
    (if (eq ?ze:team ?team-color) then
      (if (neq ?zone ?zone-intended) then
        (printout t "That is behind the zone I currently explore (" 
                  ?zone-intended ")" crlf)
      )
      (assert (worldmodel-change (machine ?ze:name) (change ZONE_STILL_TO_EXPLORE)
                                 (value FALSE))
              (worldmodel-change (machine ?ze:name) (change ZONE_MACHINE_IDENTIFIED)
                                 (value ?mps))
      )
      else
      (printout error "mps " ?mps " in wrong zone detected. Setting it to intended zone " ?zone-intended crlf)
      
      (assert (worldmodel-change (machine ?zone-intended) (change ZONE_STILL_TO_EXPLORE)
                                 (value FALSE))
              (worldmodel-change (machine ?zone-intended) (change ZONE_MACHINE_IDENTIFIED)
                                 (value ?mps))
      )
    )
  )
)