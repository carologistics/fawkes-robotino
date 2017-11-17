;---------------------------------------------------------------------------
;  state-estimation-init.clp - Initialize interfaces for state estimation
;
;  Created: Wed 15 Nov 2017 22:21:21 CET
;  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(deftemplate robot-pose
  (slot node (type STRING))
)

(defrule state-estimation-open-pose-interface
  "Open the Pose blackboard interface to get the robot pose."
  (executive-init)
  (ff-feature-loaded blackboard)
  (not (blackboard-interface (type "Position3DInterface") (id "Pose")))
  =>
  (blackboard-open-reading "Position3DInterface" "Pose")
)

(defrule state-estimation-open-shelf-configuration-interface
  "Open the ShelfConfiguration blackboard interface to get the goal
   configuration."
  (executive-init)
  (ff-feature-loaded blackboard)
  (not (blackboard-interface (type "ShelfConfigurationInterface") (id "shelf")))
  =>
  (blackboard-open-reading "ShelfConfigurationInterface" "shelf")
)

(defrule state-estimation-open-other-robot-pose-interface
  "Open the synced blackboard interface of the other robot."
  (executive-init)
  (ff-feature-loaded blackboard)
  (not
    (blackboard-interface (type "Position3DInterface") (id "OtherRobotPose"))
  )
  =>
  (blackboard-open-reading "Position3DInterface" "OtherRobotPose")
  (path-load "drive-around/state-estimation.clp")
)
