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

(defrule state-estimation-open-pose-navgraph-realted-interfaces
  "Open the Navgraph blackboard interfaces ."
  (executive-init)
  (ff-feature-loaded blackboard)
  =>
  (blackboard-open "NavGraphWithMPSGeneratorInterface" "/navgraph-generator-mps")
  (blackboard-open "NavGraphGeneratorInterface" "/navgraph-generator")
  (blackboard-open "NavigatorInterface" "Navigator")
  ; (blackboard-open "MotorInterface" "Robotino")
)

(defrule state-estimation-gripper-init
  (executive-init)
  (ff-feature-loaded blackboard)
  (not (blackboard-interface (type "AX12GripperInterface") (id "Gripper AX12")))
  =>
  (blackboard-open-reading "AX12GripperInterface" "Gripper AX12")
)

(defrule state-estimation-motor-interfaces
  "Open the Navgraph blackboard interfaces ."
  (executive-init)
  (ff-feature-loaded blackboard)
  (not (blackboard-interface (type "MotorInterface") (id "Robotino")))
  =>
  (blackboard-open "MotorInterface" "Robotino")
)

(defrule exploration-interfaces-init
  (executive-init)
  (ff-feature-loaded blackboard)
  =>
  (blackboard-open "MotorInterface" "Robotino")
  (blackboard-open "LaserLineInterface" "Laser")
  (blackboard-open "TagVisionInterface" "/tag-vision/info")
  (blackboard-open "Position3DInterface" "/explore-zone/found-tag")
  (blackboard-open "Position3DInterface" "Pose")
  (blackboard-open "TagVisionInterface" "/tag-vision/info")
  (blackboard-open "Position3DInterface" "/tag-vision/0")
  (blackboard-open "Position3DInterface" "/tag-vision/1")
  (blackboard-open "Position3DInterface" "/tag-vision/2")
  (blackboard-open "Position3DInterface" "/tag-vision/3")
  (blackboard-open "Position3DInterface" "/tag-vision/4")
  (blackboard-open "Position3DInterface" "/tag-vision/5")
  (blackboard-open "Position3DInterface" "/tag-vision/6")
  (blackboard-open "Position3DInterface" "/tag-vision/7")
  (blackboard-open "Position3DInterface" "/tag-vision/8")
  (blackboard-open "Position3DInterface" "/tag-vision/9")
  (blackboard-open "Position3DInterface" "/tag-vision/10")
  (blackboard-open "Position3DInterface" "/tag-vision/11")
  (blackboard-open "Position3DInterface" "/tag-vision/12")
  (blackboard-open "Position3DInterface" "/tag-vision/13")
  (blackboard-open "Position3DInterface" "/tag-vision/14")
  (blackboard-open "Position3DInterface" "/tag-vision/15")
  (blackboard-open "LaserLineInterface" "/laser-lines/1")
  (blackboard-open "LaserLineInterface" "/laser-lines/2")
  (blackboard-open "LaserLineInterface" "/laser-lines/3")
  (blackboard-open "LaserLineInterface" "/laser-lines/4")
  (blackboard-open "LaserLineInterface" "/laser-lines/5")
  (blackboard-open "LaserLineInterface" "/laser-lines/6")
  (blackboard-open "LaserLineInterface" "/laser-lines/7")
  (blackboard-open "LaserLineInterface" "/laser-lines/8")
  (blackboard-open "MotorInterface" "Robotino")
  (blackboard-open "ZoneInterface" "/explore-zone/info")
 (unwatch rules blackboard-read)
  (unwatch facts RobotinoSensorInterface)
  (unwatch facts Position3DInterface)
  (unwatch facts RobotinoLightInterface)
  (unwatch facts NavGraphWithMPSGeneratorInterface)
  (unwatch facts TagVisionInterface)
  (unwatch facts AX12GripperInterface)
;  (unwatch facts LaserLineInterface)
  (unwatch facts MotorInterface)
  (unwatch facts NavigatorInterface)
  (unwatch facts ZoneInterface)

)
