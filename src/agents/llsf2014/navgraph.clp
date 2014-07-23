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
  (navgraph-node (name "Ins1Sec") (pos $?ins1sec))
  (navgraph-node (name "Ins2") (pos $?ins2))
  (navgraph-node (name "Ins2Sec") (pos $?ins2sec))
  (navgraph-node (name "deliver1") (pos $?deliver1))
  (navgraph-node (name "deliver2") (pos $?deliver2))
  ?i1f <- (input-storage CYAN Ins1 0 0)
  ?i2f <- (input-storage MAGENTA Ins2 0 0)
  ?s1f <- (secondary-storage CYAN Ins1Sec 0 0)
  ?s2f <- (secondary-storage MAGENTA Ins2Sec 0 0)
  ?d1f <- (deliver CYAN deliver1 0 0)
  ?d2f <- (deliver MAGENTA deliver2 0 0)
  =>
  (retract ?i1f ?i2f ?s1f ?s2f ?d1f ?d2f)
  (assert (input-storage CYAN Ins1 (nth$ 1 ?ins1) (nth$ 2 ?ins1))
    (secondary-storage CYAN Ins1Sec (nth$ 1 ?ins1sec) (nth$ 2 ?ins1sec))
	  (input-storage MAGENTA Ins2 (nth$ 1 ?ins2) (nth$ 2 ?ins2))
    (secondary-storage MAGENTA Ins2Sec (nth$ 1 ?ins2sec) (nth$ 2 ?ins2sec))
	  (deliver CYAN deliver1 (nth$ 1 ?deliver1) (nth$ 2 ?deliver1))
	  (deliver MAGENTA deliver2 (nth$ 1 ?deliver2) (nth$ 2 ?deliver2)))
)

(defrule navgraph-remove-navgraph-facts ;to reduce amount of facts in clips-webview
  (declare (salience ?*PRIORITY-WM-LOW*))
  (navgraph-node)
  =>
  (navgraph-cleanup)
)

(defrule navgraph-block-ins-edges-for-exploration
  ;block edges at ins to avoid drivig through the pucks with the colli
  (phase EXPLORATION)
  =>
  (navgraph-block-edge "P31" "P41")
  (navgraph-block-edge "IA2H" "P31")
  (navgraph-block-edge "IA2H2" "P41")
  (navgraph-block-edge "P61" "P71")
  (navgraph-block-edge "IA1H" "P61")
  (navgraph-block-edge "IA1H2" "P71")
)

(defrule navgraph-unblock-ins-edges-for-production
  ;block edges at ins to avoid drivig through the pucks with the colli
  (phase PRODUCTION)
  =>
  (navgraph-unblock-edge "P31" "P41")
  (navgraph-unblock-edge "IA2H" "P31")
  (navgraph-unblock-edge "IA2H2" "P41")
  (navgraph-unblock-edge "P61" "P71")
  (navgraph-unblock-edge "IA1H" "P61")
  (navgraph-unblock-edge "IA1H2" "P71")
)