;Initializes and starts the exploration phase agent

(printout t "==== INIT Exploration Agent ===== " crlf)

;load priorities
(printout t "==== Load Priorities ===== " crlf)
(load* (resolve-file llsf2013/priorities.clp))

;load Blackbloard Interfaces
(printout t "==== Load Blackboard-Interfaces ===== " crlf)
(blackboard-add-interface "Position3DInterface" "Pose")
(blackboard-add-interface "RobotinoLightInterface" "Light_State")


;load exploration rules and simulationrules if ready
(defrule initialize-exploration
  (declare (salience 500))
  (agent-init)
  (protobuf-available)
  =>
  ;(printout t "==== Load Config clips-agent ===== " crlf)
  ;(load-config "/clips-agent")

  (printout t "==== Load global variables ===== " crlf)
  (load* (resolve-file llsf2013/globals.clp))

  (printout t "==== Load Exploration-Facts ===== " crlf)
  (batch* (resolve-file llsf2013/exploration/exploration-facts.clp))

  (printout t "==== Load RefboxCommunication-Interfaces ===== " crlf)
  (load* (resolve-file llsf2013/net.clp))

  (if
    (any-factp ((?conf confval))
      (and (eq ?conf:path "/clips-agent/llsf2013/enable-sim")
	   (eq ?conf:type BOOL) (eq ?conf:value true)))
  then
    (printout t "==== Loading Simulation ====" crlf)
    (batch* (resolve-file llsf2013/exploration/exploration-sim.clp))
  )
  (printout t "==== Loading Exploration Rules ====" crlf)
  (load* (resolve-file llsf2013/exploration/exploration-rules.clp))

  (assert (init))
  ;(reset)
  ;(facts)
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;TEST
  ;(printout t "==== Loading Production ====" crlf)
  ;(load* (resolve-file llsf2013/llsf2013.clp))
  

)
