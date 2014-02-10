;---------------------------------------------------------------------------
;  llsf2014.clp - Main file for the llsf2014 agent
;
;  Created: Mon Feb 10 16:09:26 2014
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;             2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; LLSF2013 agent includes
(path-load  llsf2013/priorities.clp)
(path-load  llsf2013/globals.clp)
(path-load  llsf2013/facts.clp)

(defrule load-config
  (init)
  =>
  (config-load "/clips-agent")
)

(deffacts init (init))

; Request clips-features
(defrule enable-blackboard
  (ff-feature blackboard)
  =>
  (printout t "Requesting blackboard feature" crlf)
  (ff-feature-request "blackboard")
  (path-load "llsf2013/blackboard-init.clp")
)
(defrule enable-motor-switch
  (ff-feature motor-switch)
  =>
  (printout t "Requesting motor-switch feature" crlf)
  (ff-feature-request "motor-switch")
)
(defrule enable-protobuf
  (ff-feature protobuf)
  =>
  (printout t "Requesting protobuf feature" crlf)
  (ff-feature-request "protobuf")
)

(defrule initialize
  ;when all clips features are available and the init file is loaded
  (declare (salience ?*PRIORITY-HIGH*))
  (agent-init)
  (ff-feature-loaded blackboard)
  (loaded interfaces)
  (ff-feature-loaded motor-switch)
  (ff-feature-loaded protobuf)
  =>
  (path-load  llsf2013/utils.clp)
  (path-load  llsf2013/net.clp)
  (path-load  llsf2013/skills.clp)
  (path-load  llsf2013/lock.clp)

  (if
    (any-factp ((?conf confval))
      (and (eq ?conf:path "/clips-agent/llsf2013/enable-sim")
	   (eq ?conf:type BOOL) (eq ?conf:value true)))
  then
    (printout t "Loading simulation" crlf)
    (path-load  llsf2013/sim.clp)
  )
  (path-load  llsf2013/game.clp)
  (path-load  llsf2013/general.clp)
  (path-load  llsf2013/strategy.clp)
  (path-load  llsf2013/worldmodel.clp)
  (path-load  llsf2013/production.clp)
  (path-load  llsf2013/exploration.clp)
  (reset)
  ;(facts)
)


(defrule late-silence-debug-facts
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/llsf2013/unwatch-facts") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following facts: " ?lv crlf)
  (foreach ?v ?lv (unwatch facts (sym-cat ?v)))
)

(defrule late-silence-debug-rules
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/llsf2013/unwatch-rules") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following rules: " ?lv crlf)
  (foreach ?v ?lv (unwatch rules (sym-cat ?v)))
)
