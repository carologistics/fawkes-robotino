;load needed interfaces
(defrule use-blackboard
  (ff-feature blackboard)
  =>
  (blackboard-enable-time-read)


  (blackboard-open "RobotinoLightInterface" "Light determined")
  (blackboard-open "Position3DInterface" "Pose")
  (blackboard-open "RobotinoLightInterface" "Light_State")


  (assert (loaded interfaces))
)