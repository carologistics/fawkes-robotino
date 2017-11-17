;---------------------------------------------------------------------------
;  state-estimation.clp - Assert facts according to blackboard data
;
;  Created: Fri 17 Nov 2017 13:09:03 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(defrule state-estimation-get-init-robot-pose
  (Position3DInterface (id "Pose") (translation $?pose-trans))
  (not (robot-pose))
  =>
  (assert (robot-pose (node (navgraph-closest-to (subseq$ ?pose-trans 1 2)))))
)

(defrule state-estimation-get-closest-node-to-robot
  (Position3DInterface (id "Pose") (translation $?pose-trans))
  ?rp <- (robot-pose (node ?current-pose&:
    (neq  ?current-pose (navgraph-closest-to (subseq$ ?pose-trans 1 2))))
  )
  =>
  (modify ?rp (node (navgraph-closest-to (create$ (subseq$ ?pose-trans 1 2)))))
)
