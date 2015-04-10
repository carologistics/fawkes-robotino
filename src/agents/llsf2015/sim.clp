
;---------------------------------------------------------------------------
;  sim.clp - Special rules for testing in the gazebo simulation
;
;  Created: Fri Apr 03 16:37:25 2015
;  Copyright  2015  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate sim-default-tag
  (slot name (type STRING))
  (slot side (type SYMBOL) (allowed-values INPUT OUTPUT))
  (slot frame (type STRING))
  (multislot trans (type FLOAT) (cardinality 3 3))
  (multislot rot (type FLOAT) (cardinality 4 4))
)


(deffacts sim-startup-deffacts
  "Startup Facts for testing in the simulation"
  (sim-default-tag (name "CBS") (side OUTPUT)(frame "/map") (trans  (create$ -4.9 4.9 0)) (rot (quaternion-from-yaw -1.14)))
  (sim-default-tag (name "CDS") (side INPUT)(frame "/map") (trans  (create$ 4.5 1.2 0)) (rot (quaternion-from-yaw 2.32)))
  (sim-default-tag (name "CCS1") (side OUTPUT)(frame "/map") (trans  (create$ -0.8 5.0 0)) (rot (quaternion-from-yaw -0.17)))
  (sim-default-tag (name "CCS2") (side OUTPUT)(frame "/map") (trans  (create$ -3.1 2.6 0)) (rot (quaternion-from-yaw 0.0)))
  (sim-default-tag (name "CRS1") (side INPUT)(frame "/map") (trans  (create$ 1.1 2.4 0)) (rot (quaternion-from-yaw -1.67)))
  (sim-default-tag (name "CRS2") (side INPUT)(frame "/map") (trans  (create$ 4.7 3.7 0)) (rot (quaternion-from-yaw 2.37)))
)

(defrule sim-remember-exploration
  "Remember if the agent was in exploration so we now if the Exploration was skipped or not"
  (phase EXPLORATION)
  =>
  (assert (sim-was-in-exploration))
)

(defrule sim-gen-default-navgraph-for-skipped-production
  "If the exploration was skipped generate a default navgraph for production before switching to production"
  (declare (salience ?*PRIORITY-SIM*))
  (not (sim-was-in-exploration))
  (change-phase PRODUCTION)
  ?sdt <- (sim-default-tag (name ?machine) (side ?side) (frame ?frame) (trans $?trans) (rot $?rot))
  (not (default-navgraph-generated))
  =>
  (retract ?sdt)
  (printout t "Using default position of " ?machine crlf)
  (bind ?msg (blackboard-create-msg "NavGraphWithMPSGeneratorInterface::/navgraph-generator-mps" "UpdateStationByTagMessage"))
  (blackboard-set-msg-field ?msg "id" ?machine)
  (blackboard-set-msg-field ?msg "side" ?side)
  (blackboard-set-msg-field ?msg "frame" ?frame)
  (blackboard-set-msg-multifield ?msg "tag_translation" ?trans)
  (blackboard-set-msg-multifield ?msg "tag_rotation" ?rot)
  (blackboard-send-msg ?msg)
)

(defrule sim-gen-default-navgraph-wait-for-generation
  "If the exploration was skipped generate a default navgraph for production before switching to production"
  (declare (salience ?*PRIORITY-SIM*))
  (not (sim-was-in-exploration))
  ?ch-ph <- (change-phase PRODUCTION)
  (not (sim-default-tag))
  (not (default-navgraph-generated))
  (not (timer (name waiting-for-navgraph-generation)))
  (time $?now)
  =>
  (retract ?ch-ph)
  (printout t "Waiting until navgraph-generation finished" crlf)
  (assert (timer (name waiting-for-navgraph-generation) (time ?now) (seq 1)))
)

(defrule sim-gen-default-navgraph-pause
  "pause the agent until the navgraph generation finished by removing the change-phase facts"
  (declare (salience ?*PRIORITY-SIM*))
  (not (sim-was-in-exploration))
  ?ch-ph <- (change-phase PRODUCTION)
  (timer (name waiting-for-navgraph-generation))
  =>
  (retract ?ch-ph)
)

(defrule sim-gen-default-navgraph-continue-after-generation
  "If the exploration was skipped generate a default navgraph for production before switching to production"
  (declare (salience ?*PRIORITY-SIM*))
  (not (sim-was-in-exploration))
  (not (sim-default-tag ? ? ? $? $?))
  (time $?now)
  ?ws <- (timer (name waiting-for-navgraph-generation) (time $?t&:(timeout ?now ?t 30.0)))
  ; TODO use ready flag in interface to determine if generation finisched
  (not (default-navgraph-generated))
  =>
  (retract ?ws)
  (printout t "navgraph-generation should be finished now" crlf)
  (assert (change-phase PRODUCTION)
	  (default-navgraph-generated))
)