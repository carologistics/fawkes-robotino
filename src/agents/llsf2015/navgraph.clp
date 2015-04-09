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
  =>
  ;check if the node has the orientation property for any zone
  (delayed-do-for-all-facts ((?ze zone-exploration)) (navgraph-has-property ?props (str-cat "orientation_" ?ze:name))
    ; add pose to the zone
    (bind ?pose-id (random-id))
    (assert (pose (id ?pose-id) (x (nth$ 1 ?pos)) (y (nth$ 2 ?pos))
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
