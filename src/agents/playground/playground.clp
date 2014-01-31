
;---------------------------------------------------------------------------
;  init.clp - Robotino agent decision testing
;
;  Created: Sat Jun 16 12:34:54 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; PLAYGROUND agent includes
(path-load  playground/priorities.clp)
(path-load  playground/globals.clp)
(path-load  playground/facts.clp)

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
  (path-load "playground/blackboard-init.clp")
)
(defrule enable-motor-switch
  (ff-feature motor-switch)
  =>
  (printout t "Requesting motor-switch feature" crlf)
  (ff-feature-request "motor-switch")
)

(defrule initialize
  ;when all clips features are available and the init file is loaded
  (declare (salience ?*PRIORITY-HIGH*))
  (agent-init)
  (ff-feature-loaded blackboard)
  (loaded interfaces)
  (ff-feature-loaded motor-switch)
  =>
  (printout warn "initialize" crlf)
  (path-load  playground/utils.clp)
  (path-load  playground/skills.clp)
  (path-load  playground/rules.clp)

  (reset)
  ;(facts)
)


(defrule late-silence-debug-facts
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/playground/unwatch-facts") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following facts: " ?lv crlf)
  (foreach ?v ?lv (unwatch facts (sym-cat ?v)))
)

(defrule late-silence-debug-rules
  (declare (salience -1000))
  (init)
  (protobuf-available)
  (confval (path "/clips-agent/clips-debug") (type BOOL) (value true))
  (confval (path "/clips-agent/playground/unwatch-rules") (type STRING) (is-list TRUE) (list-value $?lv))
  =>
  (printout t "Disabling watching of the following rules: " ?lv crlf)
  (foreach ?v ?lv (unwatch rules (sym-cat ?v)))
)
