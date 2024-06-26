;---------------------------------------------------------------------------
;  state-estimation-init.clp - Initialize interfaces for state estimation
;
;  Created: Wed 15 Nov 2017 22:21:21 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(defrule state-estimation-open-pose-interface
  "Open the Pose blackboard interface to get the robot pose."
  (executive-init)
  (ff-feature-loaded blackboard)
  (not (blackboard-interface (type "Position3DInterface") (id "Pose")))
  =>
  (blackboard-open-reading "Position3DInterface" "Pose")
)
