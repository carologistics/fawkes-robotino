;---------------------------------------------------------------------------
;  llsf2015.clp - Main file for the llsf2015 agent
;
;  Created: Mon Feb 10 16:09:26 2014
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;             2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; LLSF2014 agent includes
(path-load  abstract-task-execution/priorities.clp)
(path-load  abstract-task-execution/globals.clp)
(path-load  abstract-task-execution/facts.clp)

(defrule load-config
  "Load configration for initialization."
  (init)
  =>
  (config-load "/clips-agent")
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
  (path-load "abstract-task-execution/blackboard-init.clp")
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
  (path-load  abstract-task-execution/navgraph.clp)
)

(deffunction unwatch-rules-facts ()
  ;unwatch some rules to reduce debug output
  ;this is not possible in the config, because the defrules and deffacts are loaded later
  (unwatch rules worldmodel-sync-receive-worldmodel)
  (unwatch rules worldmodel-sync-publish-worldmodel)
  (unwatch rules wm-update-puck-in-gripper)
  (unwatch rules wm-update-pose)
  (unwatch rules wm-update-pose)
  (unwatch rules net-send-BeaconSignal)
  (unwatch rules net-recv-BeaconSignal)
  (unwatch rules net-recv-GameState)
  (unwatch rules lock-send-message)
  (unwatch rules lock-receive-message)
  (unwatch rules worldmodel-sync-receive-worldmodel)
  (unwatch rules skill-update-nochange)
  (unwatch facts protobuf-msg)
  (unwatch facts active-robot)
  (unwatch facts pose)
  (unwatch facts timer)
  (unwatch facts skill-update)
)

(defrule initialize
  "Actual initialization rule. Loads the need rule-files for skills and inter-robot communication."
  ;when all clips features are available and the init file is loaded
  (declare (salience ?*PRIORITY-HIGH*))
  (agent-init)
  (ff-feature-loaded blackboard)
  (loaded interfaces)
  (ff-feature-loaded motor-switch)
  (ff-feature-loaded protobuf)
  (ff-feature navgraph)
  =>
  (path-load  abstract-task-execution/utils.clp)
  (path-load  abstract-task-execution/net.clp)
  (path-load  abstract-task-execution/skills.clp)
  (path-load  abstract-task-execution/worldmodel.clp)
  (path-load  abstract-task-execution/lock-managing.clp)
  (path-load  abstract-task-execution/lock-usage.clp)
  (if
    (any-factp ((?conf confval))
      (and (eq ?conf:path "/clips-agent/abstract-task-execution/enable-sim")
	   (eq ?conf:type BOOL) (eq ?conf:value true)))
  then
    (printout t "Loading simulation" crlf)
    (path-load  abstract-task-execution/sim.clp)
  )
  (path-load  abstract-task-execution/game.clp)
  (path-load  abstract-task-execution/worldmodel-synchronization.clp)
  (path-load  abstract-task-execution/tactical-help.clp)
  (path-load  abstract-task-execution/task.clp)
  (path-load  abstract-task-execution/steps.clp)
  (path-load  abstract-task-execution/coordination.clp)
  (path-load  abstract-task-execution/production.clp)
  (path-load  abstract-task-execution/exploration.clp)
  (path-load  abstract-task-execution/config.clp)
  (reset)
  ;(facts)

  (unwatch-rules-facts)
)

(defrule late-silence-debug-facts
  "Disables watching of facts configured in clips-agent/llsf2015/unwatch-facts for debug mode."
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/llsf2015/unwatch-facts") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following facts: " ?lv crlf)
  (foreach ?v ?lv (unwatch facts (sym-cat ?v)))
)

(defrule late-silence-debug-rules
  "Disables watching of rules configured in clips-agent/llsf2015/unwatch-rules for debug mode."
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/llsf2015/unwatch-rules") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following rules: " ?lv crlf)
  (foreach ?v ?lv (unwatch rules (sym-cat ?v)))
)
