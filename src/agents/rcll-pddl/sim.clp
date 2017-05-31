
;---------------------------------------------------------------------------
;  sim.clp - Special rules for testing in the gazebo simulation
;
;  Created: Fri Apr 03 16:37:25 2015
;  Copyright  2015  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


(defrule sim-remember-exploration
  "Remember if the agent was in exploration so we now if the Exploration was skipped or not"
  (phase EXPLORATION)
  =>
  (assert (sim-was-in-exploration))
)