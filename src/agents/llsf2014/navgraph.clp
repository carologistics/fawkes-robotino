;---------------------------------------------------------------------------
;  navgraph.clp - using the navgraph-feature
;
;  Created: Sat Mar 22 13:46:30 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


(defrule wm-set-machine-coordinates
  (declare (salience ?*PRIORITY-WM*))
  ?mf <- (machine (name ?name) (x 0.0) (y 0.0))
  (navgraph-node (name ?sname&:(eq ?sname (str-cat ?name))) (pos $?pos))
  =>
  (modify ?mf (x (nth$ 1 ?pos)) (y (nth$ 2 ?pos)))
)

(defrule wm-set-machine-exp-coordinates
  (declare (salience ?*PRIORITY-WM*))
  ?mf <- (machine-exploration (look-pos ?name) (x 0.0) (y 0.0))
  (navgraph-node (name ?sname&:(eq ?sname (str-cat ?name))) (pos $?pos))
  =>
  (modify ?mf (x (nth$ 1 ?pos)) (y (nth$ 2 ?pos)))
)

(defrule wm-remove-navgraph-facts ;to reduce amount of facts in clips-webview
  (declare (salience ?*PRIORITY-WM-LOW*))
  (navgraph-node)
  =>
  (navgraph-cleanup)
)

