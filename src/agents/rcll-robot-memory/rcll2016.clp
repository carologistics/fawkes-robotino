;---------------------------------------------------------------------------
;  rcll2016.clp - Main file for the rcll2016 agent with the robot memory
;
;  Created: Thu Sep 01 15:18:09 2016
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;             2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule use-asp
  "Checks if we use the asp configuration."
  (declare (salience 750))
  (confval (path "/clips-agent/rcll2016/use-asp") (value TRUE))
  (not (asp-checked))
  =>
  (assert (use-asp))
  (assert (asp-checked))
  (printout t "Use ASP" crlf)
)

(defrule use-no-asp
  "Also checks if we use the asp configuration."
  (declare (salience 750))
  (confval (path "/clips-agent/rcll2016/use-asp") (value FALSE))
  (not (asp-checked))
  =>
  (assert (asp-checked))
  (printout t "Use no ASP" crlf)
)

(path-load  rcll-robot-memory/priorities.clp)
(path-load  rcll-robot-memory/globals.clp)
(path-load  rcll-robot-memory/facts.clp)

(defrule load-config
  "Load configration for initialization."
  (init)
  =>
  (config-load "/clips-agent")
  (config-load "/asp-agent")
  (config-load "/hardware/robotino")
)

(deffacts init 
  "Initializes clips agent."
  (init)
)

; Request clips-features
(defrule enable-blackboard
  "If blackboard feature is set load the blackboard, if it is not yet loaded."
  (ff-feature blackboard)
  (not (ff-feature-loaded blackboard))
  =>
  (printout t "Requesting blackboard feature" crlf)
  (ff-feature-request "blackboard")
  (path-load "rcll-robot-memory/blackboard-init.clp")
)

(defrule enable-motor-switch
  "If motor-switch feature is set load the motor-switch, if it is not yet loaded."
  (ff-feature motor-switch)
  (not (ff-feature-loaded motor-switch))
  =>
  (printout t "Requesting motor-switch feature" crlf)
  (ff-feature-request "motor-switch")
)

(defrule enable-protobuf
  "If protobuf feature is set load the protobuf, if it is not yet loaded."
  (ff-feature protobuf)
  (not (ff-feature-loaded protobuf))
  =>
  (printout t "Requesting protobuf feature" crlf)
  (ff-feature-request "protobuf")
)

(defrule enable-navgraph
  "If navgraph feature is set load the navgraph, if it is not yet loaded and init fact is set."
  (init) ;because we want to load the navgraph after the (reset), which would delete all navgraph facts
  (ff-feature navgraph)
  (not (ff-feature-loaded navgraph))
  =>
  (printout t "Requesting navgraph feature" crlf)
  (ff-feature-request "navgraph")
  (path-load  rcll-robot-memory/navgraph.clp)
)

(defrule enable-tf
  "If transform feature is set load it, if it is not yet loaded."
  (ff-feature tf)
  (not (ff-feature-loaded tf))
  =>
  (printout t "Requesting transform feature" crlf)
  (ff-feature-request "tf")
)

(defrule enable-robot-memroy
  "If the robot_memory feature is set, load it, if it is not yet loaded."
  (ff-feature robot_memory)
  (not (ff-feature-loaded tf))
  =>
  (printout t "Requesting robot-memory feature" crlf)
  (ff-feature-request "robot_memory")
)

(deffunction unwatch-rules-facts ()
  ;unwatch some rules to reduce debug output
  ;this is not possible in the config, because the defrules and deffacts are loaded later
  ; (unwatch rules worldmodel-sync-receive-worldmodel)
  ; (unwatch rules worldmodel-sync-publish-worldmodel)
  ; (unwatch rules wm-update-puck-in-gripper)
  (unwatch rules wm-update-pose)
  (unwatch rules net-send-BeaconSignal)
  (unwatch rules net-recv-BeaconSignal)
  (unwatch rules net-recv-GameState)
  (unwatch rules lock-send-message)
  (unwatch rules lock-receive-message)
  (unwatch rules worldmodel-sync-receive-worldmodel)
  (unwatch rules skill-update-nochange)
  (unwatch rules time-retract)
  (unwatch facts protobuf-msg)
  (unwatch facts active-robot)
  (unwatch facts pose)
  (unwatch facts timer)
  (unwatch facts time)
  (unwatch facts skill-update)
)

(defrule initialize
  "Actual initialization rule. Loads the need rule-files for skills and inter-robot communication."
  ;when all clips features are available and the init file is loaded
  (declare (salience ?*PRIORITY-HIGH*))
  (asp-checked)
  (agent-init)
  (ff-feature-loaded blackboard)
  (loaded interfaces)
  (ff-feature-loaded motor-switch)
  (ff-feature-loaded protobuf)
  (ff-feature-loaded tf)
  (ff-feature-loaded robot_memory)
  (ff-feature navgraph)
  =>
  (path-load  rcll-robot-memory/utils.clp)
  (path-load  rcll-robot-memory/robot-memory.clp)
  (path-load  rcll-robot-memory/worldmodel-synchronization.clp)
  (path-load  rcll-robot-memory/net.clp)
  (path-load  rcll-robot-memory/worldmodel.clp)
  (path-load  rcll-robot-memory/skills.clp)
  (path-load  rcll-robot-memory/lock-managing.clp)
  (path-load  rcll-robot-memory/lock-usage.clp)
  (if
    (any-factp ((?conf confval))
      (and (eq ?conf:path "/clips-agent/rcll2016/enable-sim")
	   (eq ?conf:type BOOL) (eq ?conf:value TRUE)))
  then
    (printout t "Loading simulation" crlf)
    (path-load  rcll-robot-memory/sim.clp)
  )
  (path-load  rcll-robot-memory/game.clp)
  (path-load  rcll-robot-memory/mps-instructions.clp)
  (path-load  rcll-robot-memory/tactical-help.clp)
  (path-load  rcll-robot-memory/task.clp)
  (path-load  rcll-robot-memory/steps.clp)
  (path-load  rcll-robot-memory/coordination.clp)
  (if
    (not (any-factp ((?conf confval))
      (and (eq ?conf:path "/clips-agent/rcll2016/load-reasoner")
        (eq ?conf:type BOOL) (eq ?conf:value FALSE))))
  then
    (path-load  rcll-robot-memory/production.clp)
    (path-load  rcll-robot-memory/exploration.clp)
  else
    (printout t "Not loading the reasoner" crlf)
  )
  (path-load  rcll-robot-memory/config.clp)
  (reset)
  ;(facts)

  (unwatch-rules-facts)
)

(defrule late-silence-debug-facts
  "Disables watching of facts configured in clips-agent/rcll2016/unwatch-facts for debug mode."
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/rcll2016/unwatch-facts") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following facts: " ?lv crlf)
  (foreach ?v ?lv (unwatch facts (sym-cat ?v)))
)

(defrule late-silence-debug-rules
  "Disables watching of rules configured in clips-agent/rcll2016/unwatch-rules for debug mode."
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/rcll2016/unwatch-rules") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following rules: " ?lv crlf)
  (foreach ?v ?lv (unwatch rules (sym-cat ?v)))
)
