;load needed interfaces
(defrule use-blackboard
  "Loads interfaces for usage in Clips. These interfaces can only be used after loading."
  (ff-feature blackboard)
  =>
  (blackboard-enable-time-read)


  (blackboard-open "RobotinoLightInterface" "Light determined")
  (blackboard-open "Position3DInterface" "Pose")
  (blackboard-open "RobotinoLightInterface" "/machine-signal/best")
  (blackboard-open "RobotinoSensorInterface" "Robotino")
  (blackboard-open "NavGraphWithMPSGeneratorInterface" "/navgraph-generator-mps")
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
  (blackboard-open "AX12GripperInterface" "Gripper AX12")

  ;unwatch to avoid debug spam
  (unwatch rules blackboard-read)
  (unwatch rules RobotinoSensorInterface-cleanup)
  (unwatch facts RobotinoSensorInterface)
  (unwatch rules Position3DInterface-cleanup)
  (unwatch facts Position3DInterface)
  (unwatch facts RobotinoLightInterface)
  (unwatch rules RobotinoLightInterface-cleanup)
  (unwatch facts NavGraphWithMPSGeneratorInterface)
  (unwatch rules NavGraphWithMPSGeneratorInterface-cleanup)
  (unwatch facts TagVisionInterface)
  (unwatch rules TagVisionInterface-cleanup)
  (unwatch facts AX12GripperInterface)
  (unwatch rules AX12GripperInterface-cleanup)

  (assert (loaded interfaces))
)
