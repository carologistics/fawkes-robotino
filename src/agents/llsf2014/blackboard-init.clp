;load needed interfaces
(defrule use-blackboard
  "Loads interfaces for usage in Clips. These interfaces can only be used after loading."
  (ff-feature blackboard)
  =>
  (blackboard-enable-time-read)


  (blackboard-open "RobotinoLightInterface" "Light determined")
  (blackboard-open "Position3DInterface" "Pose")
  (blackboard-open "RobotinoLightInterface" "Light_State")
  (blackboard-open "RobotinoSensorInterface" "Robotino")

  ;unwatch to avoid debug spam
  (unwatch rules blackboard-read)
  (unwatch rules RobotinoSensorInterface-cleanup)
  (unwatch rules Position3DInterface-cleanup)
  (unwatch rules RobotinoLightInterface-cleanup)
  (unwatch facts RobotinoSensorInterface)
  (unwatch facts Position3DInterface)
  (unwatch facts RobotinoLightInterface)

  (assert (loaded interfaces))
)
