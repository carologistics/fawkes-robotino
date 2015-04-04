
;---------------------------------------------------------------------------
;  sim.clp - Special rules for testing in the gazebo simulation
;
;  Created: Fri Apr 03 16:37:25 2015
;  Copyright  2015  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deffacts sim-startup-deffacts
  "Startup Facts for testing in the simulation"
  (sim-default-tag "CBS" OUTPUT "/map" (create$ -4.9 4.9 0) (quaternion-from-yaw -1.14))
  (sim-default-tag "CDS" INPUT "/map" (create$ 4.5 1.2 0) (quaternion-from-yaw 2.32))
  (sim-default-tag "CCS1" OUTPUT "/map" (create$ -0.8 5.0 0) (quaternion-from-yaw -0.17))
  (sim-default-tag "CCS2" OUTPUT "/map" (create$ -3.1 2.6 0) (quaternion-from-yaw 0.0))
  (sim-default-tag "CRS1" INPUT "/map" (create$ 1.1 2.4 0) (quaternion-from-yaw -1.67))
  (sim-default-tag "CRS2" INPUT "/map" (create$ 4.7 3.7 0) (quaternion-from-yaw 2.37))
)

(defrule sim-remember-exploration
  "Remember if the agent was in exploration so we now if the Exploration was skipped or not"
  (phase EXPLORATION)
  =>
  (assert (sim-was-in-exploration))
)

(defrule sim-gen-default-navgraph-for-skipped-production
  "If the exploration was skipped generate a default navgraph for production"
  (not (sim-was-in-exploration))
  (phase PRODUCTION)
  ?sdt <- (sim-default-tag ?machine ?side ?frame $?trans $?rot)
  =>
  (retract ?sdt)
  (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "UpdateStationByTagMessage"))
  (blackboard-set-msg-field ?msg "id" ?machine)
  (blackboard-set-msg-field ?msg "side" ?side)
  (blackboard-set-msg-field ?msg "frame" ?frame)
  (blackboard-set-msg-multifield ?msg "tag_translation" ?trans)
  (blackboard-set-msg-multifield ?msg "tag_rotation" ?rot)
  (blackboard-send-msg ?msg)
)