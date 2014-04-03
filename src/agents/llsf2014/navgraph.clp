;---------------------------------------------------------------------------
;  navgraph.clp - using the navgraph-feature
;
;  Created: Sat Mar 22 13:46:30 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


(defrule navgraph-set-machine-coordinates
  (declare (salience ?*PRIORITY-WM*))
  ?mf <- (machine (name ?name) (x 0.0) (y 0.0))
  (navgraph-node (name ?sname&:(eq ?sname (str-cat ?name))) (pos $?pos))
  =>
  (modify ?mf (x (nth$ 1 ?pos)) (y (nth$ 2 ?pos)))
)

(defrule navgraph-set-machine-exp-coordinates
  (declare (salience ?*PRIORITY-WM*))
  ?mf <- (machine-exploration (look-pos ?name) (x 0.0) (y 0.0))
  (navgraph-node (name ?sname&:(eq ?sname (str-cat ?name))) (pos $?pos))
  =>
  (modify ?mf (x (nth$ 1 ?pos)) (y (nth$ 2 ?pos)))
)

(defrule navgraph-get-ins-delivery-coordinates
  (declare (salience ?*PRIORITY-WM*))
  (navgraph-node (name "Ins1") (pos $?ins1))
  (navgraph-node (name "Ins2") (pos $?ins2))
  (navgraph-node (name "deliver1") (pos $?deliver1))
  (navgraph-node (name "deliver2") (pos $?deliver2))
  ?i1f <- (input-storage CYAN Ins1 0 0)
  ?i2f <- (input-storage MAGENTA Ins2 0 0)
  ?d1f <- (deliver CYAN deliver1 0 0)
  ?d2f <- (deliver MAGENTA deliver2 0 0)
  =>
  (retract ?i1f ?i2f ?d1f ?d2f)
  (assert (input-storage CYAN Ins1 (nth$ 1 ?ins1) (nth$ 2 ?ins1))
	  (input-storage MAGENTA Ins2 (nth$ 1 ?ins2) (nth$ 2 ?ins2))
	  (deliver CYAN deliver1 (nth$ 1 ?deliver1) (nth$ 2 ?deliver1))
	  (deliver MAGENTA deliver2 (nth$ 1 ?deliver2) (nth$ 2 ?deliver2)))
)

(defrule navgraph-remove-navgraph-facts ;to reduce amount of facts in clips-webview
  (declare (salience ?*PRIORITY-WM-LOW*))
  (navgraph-node)
  =>
  (navgraph-cleanup)
)