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


  (assert (loaded interfaces))
)
