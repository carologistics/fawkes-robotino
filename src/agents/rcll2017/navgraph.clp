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

(defrule navgraph-remove-navgraph-nodes ;to reduce amount of facts in clips-webview
  (declare (salience ?*PRIORITY-WM-LOW*))
  ?nn <- (navgraph-node (name ?name&~:(or (eq "C-" (sub-string 1 2 (str-cat ?name)))
                                          (eq "M-" (sub-string 1 2 (str-cat ?name)))
                                          (eq "WAIT" (sub-string 1 4 (str-cat ?name)))
                                          (eq "exp" (sub-string 1 3 (str-cat ?name)))
  )))
  =>
  (retract ?nn)
)

(defrule navgraph-remove-navgraph-edges ;to reduce amount of facts in clips-webview
  (declare (salience ?*PRIORITY-WM-LOW*))
  ?ne <- (navgraph-edge)
  =>
  (retract ?ne)
)


(defrule navgraph-generate-waiting-positions
  "To generate waiting positions in the empty zones we have to send the SetWaitZones message to the navgraph-generation interface"
  (phase PRODUCTION)
  (forall (machine (name ?mps))
          (zone-exploration (times-searched ?ts) (machine ?found&:(eq ?mps ?found))))
  (not (added-waiting-positions))
  (not (requested-waiting-positions))
  =>
  (printout t "Adding waiting positions in empty zones" crlf)
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

(defrule navgraph-sim-add-found-tags-when-added-by-ground-truth
  "When the gazsim-navgraph-generator plugin is used to add the navgraph points to test production without exploration, we have to add the found-tags"
  (phase PRODUCTION)
  (not (sim-was-in-exploration))
  (machine (name ?mps))
  (not (found-tag (name ?mps)))
  (navgraph-node (name ?mps-o-str&:(eq ?mps-o-str (str-cat ?mps "-O"))))
  =>
  (assert (found-tag (name ?mps) (side OUTPUT))
          (navgraph-added-for-mps (name ?mps)))
)
