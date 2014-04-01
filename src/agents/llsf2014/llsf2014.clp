;---------------------------------------------------------------------------
;  llsf2014.clp - Main file for the llsf2014 agent
;
;  Created: Mon Feb 10 16:09:26 2014
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;             2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; LLSF2014 agent includes
(path-load  llsf2014/priorities.clp)
(path-load  llsf2014/globals.clp)
(path-load  llsf2014/facts.clp)

(defrule load-config
  (init)
  =>
  (config-load "/clips-agent")
  (config-load "/hardware/robotino")
)

(deffacts init (init))

; Request clips-features
(defrule enable-blackboard
  (ff-feature blackboard)
  (not (ff-feature-loaded blackboard))
  =>
  (printout t "Requesting blackboard feature" crlf)
  (ff-feature-request "blackboard")
  (path-load "llsf2014/blackboard-init.clp")
)
(defrule enable-motor-switch
  (ff-feature motor-switch)
  (not (ff-feature-loaded motor-switch))
  =>
  (printout t "Requesting motor-switch feature" crlf)
  (ff-feature-request "motor-switch")
)
(defrule enable-protobuf
  (ff-feature protobuf)
  (not (ff-feature-loaded protobuf))
  =>
  (printout t "Requesting protobuf feature" crlf)
  (ff-feature-request "protobuf")
)
(defrule enable-navgraph
  (init) ;because we want to load the navgraph after the (reset), which would delete all navgraph facts
  (ff-feature navgraph)
  (not (ff-feature-loaded navgraph))
  =>
  (printout t "Requesting navgraph feature" crlf)
  (ff-feature-request "navgraph")
  (path-load  llsf2014/navgraph.clp)
)

(defrule initialize
  ;when all clips features are available and the init file is loaded
  (declare (salience ?*PRIORITY-HIGH*))
  (agent-init)
  (ff-feature-loaded blackboard)
  (loaded interfaces)
  (ff-feature-loaded motor-switch)
  (ff-feature-loaded protobuf)
  (ff-feature navgraph)
  =>
  (path-load  llsf2014/utils.clp)
  (path-load  llsf2014/net.clp)
  (path-load  llsf2014/skills.clp)
  (path-load  llsf2014/lock-managing.clp)
  (path-load  llsf2014/lock-usage.clp)

  (if
    (any-factp ((?conf confval))
      (and (eq ?conf:path "/clips-agent/llsf2014/enable-sim")
	   (eq ?conf:type BOOL) (eq ?conf:value true)))
  then
    (printout t "Loading simulation" crlf)
    (path-load  llsf2014/sim.clp)
  )
  (path-load  llsf2014/game.clp)
  (path-load  llsf2014/general.clp)
  (path-load  llsf2014/worldmodel.clp)
  (path-load  llsf2014/worldmodel-synchronization.clp)
  (path-load  llsf2014/tactical-help.clp)
  (path-load  llsf2014/tasks.clp)
  (path-load  llsf2014/coordination.clp)
  (path-load  llsf2014/production.clp)
  (path-load  llsf2014/exploration.clp)
  (path-load  llsf2014/config.clp)
  (reset)
  ;(facts)
)


(defrule late-silence-debug-facts
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/llsf2014/unwatch-facts") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following facts: " ?lv crlf)
  (foreach ?v ?lv (unwatch facts (sym-cat ?v)))
)

(defrule late-silence-debug-rules
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/llsf2014/unwatch-rules") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following rules: " ?lv crlf)
  (foreach ?v ?lv (unwatch rules (sym-cat ?v)))
)