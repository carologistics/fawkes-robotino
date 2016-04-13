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
                                          (eq "M-" (sub-string 1 2 (str-cat ?name)))
                                          (eq "WAIT" (sub-string 1 4 (str-cat ?name))))))
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
		    (trans $?trans) (rot $?rot))
  (tag-matching (tag-id ?tag) (machine ?machine) (side ?side))
  (not (navgraph-added-for-mps (name ?machine)))
  =>
  (printout t "Add Tag Nr." ?tag " (" ?machine " " ?side 
	    ") we got from another bot to Navgraph-generation"  crlf)
  (printout warn "TODO: check which zone contains the machine, so we don't try to find a tag there again"  crlf)
  
  (navgraph-add-all-new-tags)
)

(defrule navgraph-get-zone-of-found-tag
  "For new found tags that were added to the navgraph, we want to check the zone where the machine is standing for the exploration report"
  (declare (salience ?*PRIORITY-WM*))
  (found-tag (name ?mps) (side ?side) (frame ?frame)
             (trans $?trans) (rot $?rot))
  (navgraph-node (name ?input&:(eq ?input (str-cat ?mps "-I")))
                 (pos $?pos-i))
  (navgraph-node (name ?output&:(eq ?output (str-cat ?mps "-O")))
                 (pos $?pos-o))
  (team-color ?team-color)
  (goalmachine ?zone-intended)
  (not (zone-exploration (machine ?mps)))
  (phase ?phase)
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

  ;if we are currently in the exploration phase check which zone we expect the machine to be in
  (bind ?zone-intended NONE)
  (do-for-fact ((?cur-zone-fact exp-current-zone)) TRUE
    (bind ?zone-intended ?cur-zone-fact:name)
  )
  (printout t "mps " ?mps " is in zone " ?zone crlf)

  (bind ?zone-color-right FALSE)
  (do-for-fact ((?ze zone-exploration)) (eq ?ze:name (sym-cat Z ?zone))
    (if (or (eq ?ze:team ?team-color) (eq ?phase PRODUCTION)) then
      (if (and (neq (sym-cat Z ?zone) ?zone-intended) (neq ?zone-intended NONE)) then
        (printout t "That is behind the zone I currently explore (" 
                  ?zone-intended ")" crlf)
      )
      (synced-modify ?ze still-to-explore FALSE
                     recognized TRUE
                     machine ?mps)
      (bind ?zone-color-right TRUE)
    )
  )
  (if (and (not ?zone-color-right) (eq ?phase EXPLORATION))
    then
    (printout error "mps " ?mps " in wrong zone detected. Setting it to intended zone " ?zone-intended crlf)
    (do-for-fact ((?ze zone-exploration)) (eq ?ze:name ?zone-intended)
      (synced-modify ?ze still-to-explore FALSE
                     recognized TRUE
                     machine ?mps)
    )
  )
)

(defrule navgraph-generate-waiting-positions
  "To generate waiting positions in the empty zones we have to send the SetWaitZones message to the navgraph-generation interface"
  (phase PRODUCTION)
  (forall (machine (name ?mps))
          (found-tag (name ?mps))
          (zone-exploration (times-searched ?ts) (machine ?found&:(or (eq ?mps ?found)
                                                                      (and (eq ?found UNKNOWN) (> ?ts 2))))))
  (not (added-waiting-positions))
  (not (requested-waiting-positions))
  =>
  (printout error "Adding waiting positions in empty zones" crlf)
  (printout error "Adding waiting positions in empty zones" crlf)
  (printout error "Adding waiting positions in empty zones" crlf)
  (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "SetWaitZonesMessage"))
  (bind $?zones-to-wait (create-multifield-with-length-and-entry 24 FALSE))

  (do-for-all-facts ((?zone zone-exploration)) (eq ?zone:machine UNKNOWN)
    (bind ?zone-name (str-cat ?zone:name))
    (bind ?zone-index (eval (sub-string 2 (str-length ?zone-name) ?zone-name)))
    (bind ?zones-to-wait
          (replace$ ?zones-to-wait ?zone-index ?zone-index TRUE))
  )
  (printout t "Adding wait-zones in: " ?zones-to-wait crlf)
  (blackboard-set-msg-multifield ?msg "zones" ?zones-to-wait)
  (blackboard-send-msg ?msg)
  (printout t "Sent wait zones" crlf)

  (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "ComputeMessage"))
  (bind ?compute-msg-id (blackboard-send-msg ?msg))
  (printout t "Sent compute" crlf)
  (assert (requested-waiting-positions))
  ; save the last compute-msg-id to know when it was processed
  (do-for-all-facts ((?lncm last-navgraph-compute-msg)) TRUE
    (retract ?lncm)
  )
  (assert (last-navgraph-compute-msg (id ?compute-msg-id)))
)

(defrule navgraph-compute-wait-positions-finished
  (requested-waiting-positions)
  (last-navgraph-compute-msg (id ?compute-msg-id))
  ?ngg-if <- (NavGraphWithMPSGeneratorInterface (id "/navgraph-generator-mps") (msgid ?compute-msg-id) (final TRUE))
  =>
  (assert (added-waiting-positions))
  (printout t "Navgraph generation of waiting-points finished. Getting waitpoints." crlf)
  (retract ?ngg-if)
  (do-for-all-facts ((?waitzone navgraph-node)) (eq "WAIT-Z" (sub-string 1 6 ?waitzone:name))
    (assert (zone-waitpoint (name (str-cat "WAIT-" (sym-cat (sub-string 6 (str-length ?waitzone:name) ?waitzone:name))))
                            (x (nth$ 1 ?waitzone:pos)) (y (nth$ 2 ?waitzone:pos))))
  )
)


(defrule navgraph-assign-places-to-waitpoints
  "Assing places to waitpoints so we can drive there when the place is locked."
  (added-waiting-positions)
  (navgraph-node (name ?p&:(or (eq "-I" (sub-string (- (str-length ?p) 1) (str-length ?p) ?p))
                               (eq "-O" (sub-string (- (str-length ?p) 1) (str-length ?p) ?p))))                               
                 (pos $?pos))
  (wait-point ?wait-point)
  =>
  ;find nearest waiting point
  (bind ?nearest-wp ?wait-point);default
  (bind ?nearest-dist 10000)
  (delayed-do-for-all-facts ((?wpt zone-waitpoint)) TRUE
    (bind ?dist (distance (nth$ 1 ?pos) (nth$ 2 ?pos) ?wpt:x ?wpt:y))
    (if (< ?dist ?nearest-dist) then
      (bind ?nearest-dist ?dist)
      (bind ?nearest-wp ?wpt:name)
    )
  )
  (assert (place-waitpoint-assignment (place (sym-cat ?p)) (waitpoint ?nearest-wp)))
)

(defrule navgraph-remove-exp-graph-when-not-needed-anymore
  "When the explration is finished and we have found all machines, we can remove left exploration points"
  (phase PRODUCTION)
  (not (exp-graph-removed))
  (forall (machine (name ?mps))
          (found-tag (name ?mps)))
  =>
  (printout t "Removing all exploration points" crlf)
  (assert (exp-graph-removed))

  (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "SetExplorationZonesMessage"))
  (blackboard-set-msg-multifield ?msg "zones" (create-multifield-with-length-and-entry 24 FALSE))
  (printout t "Setting Zones still to explore: " (create-multifield-with-length-and-entry 24 FALSE) crlf)
  (blackboard-send-msg ?msg)
  
  (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "ComputeMessage"))
  (bind ?compute-msg-id (blackboard-send-msg ?msg))
    
  ; save the last compute-msg-id to know when it was processed
  (do-for-all-facts ((?lncm last-navgraph-compute-msg)) TRUE
                    (retract ?lncm)
                    )
  (assert (last-navgraph-compute-msg (id ?compute-msg-id)))
)
