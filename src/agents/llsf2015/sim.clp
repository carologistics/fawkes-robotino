
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
  (sim-default-tag (name "C-BS") (side OUTPUT)(frame "/map") (trans  (create$ -4.9 4.9 0)) (rot (quaternion-from-yaw -1.14)))
  (sim-default-tag (name "C-DS") (side INPUT)(frame "/map") (trans  (create$ 4.5 1.2 0)) (rot (quaternion-from-yaw 2.32)))
  (sim-default-tag (name "C-CS1") (side OUTPUT)(frame "/map") (trans  (create$ -0.8 5.0 0)) (rot (quaternion-from-yaw -0.17)))
  (sim-default-tag (name "C-CS2") (side OUTPUT)(frame "/map") (trans  (create$ -3.1 2.6 0)) (rot (quaternion-from-yaw 0.0)))
  (sim-default-tag (name "C-RS1") (side INPUT)(frame "/map") (trans  (create$ 1.1 2.4 0)) (rot (quaternion-from-yaw -1.67)))
  (sim-default-tag (name "C-RS2") (side INPUT)(frame "/map") (trans  (create$ 4.7 3.7 0)) (rot (quaternion-from-yaw 2.37)))
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
  (assert (found-tag (name ?machine) (side ?side) (frame ?frame)
		     (trans ?trans) (rot ?rot)))
)

(defrule sim-gen-default-navgraph-compute-and-wait-for-generation
  "If the exploration was skipped generate a default navgraph for production before switching to production"
  (declare (salience ?*PRIORITY-SIM*))
  (not (sim-was-in-exploration))
  ?ch-ph <- (change-phase PRODUCTION)
  (not (sim-default-tag))
  (not (default-navgraph-generated))
  (not (last-navgraph-compute-msg (id ?)))
  =>
  (retract ?ch-ph)
  (navgraph-add-all-new-tags)
)

(defrule sim-gen-default-navgraph-pause
  "pause the agent until the navgraph generation finished by removing the change-phase facts"
  (declare (salience ?*PRIORITY-SIM*))
  (not (sim-was-in-exploration))
  ?ch-ph <- (change-phase PRODUCTION)
  (last-navgraph-compute-msg (id ?))
  =>
  (retract ?ch-ph)
)

(defrule sim-gen-default-navgraph-continue-after-generation
  "If the exploration was skipped generate a default navgraph for production before switching to production"
  (declare (salience ?*PRIORITY-SIM*))
  (not (sim-was-in-exploration))
  (not (sim-default-tag))
  ; wait until the navgraph-generator has added the path to the output
  ?lncm <- (last-navgraph-compute-msg (id ?compute-msg-id))
  ?ngg-if <- (NavGraphWithMPSGeneratorInterface (id "/navgraph-generator-mps") (msgid ?compute-msg-id) (final TRUE))
  (not (default-navgraph-generated))
  =>
  (retract ?lncm ?ngg-if)
  (printout t "navgraph-generation should be finished now" crlf)
  (assert (change-phase PRODUCTION)
	  (default-navgraph-generated))
)