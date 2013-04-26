
;---------------------------------------------------------------------------
;  init.clp - Robotino agent decision testing
;
;  Created: Sat Jun 16 12:34:54 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; LLSF2013 agent includes
(load* (resolve-file llsf2013/priorities.clp))
(load* (resolve-file llsf2013/globals.clp))
(load* (resolve-file llsf2013/facts.clp))

(defrule load-config
  (init)
  =>
  (load-config "/clips-agent")
)

(deffacts init (init))

(defrule initialize
  (declare (salience ?*PRIORITY-HIGH*))
  (agent-init)
  (protobuf-available)
  =>
  (load-config "/clips-agent")

  (blackboard-add-interface "RobotinoLightInterface" "Light determined")

  (load* (resolve-file llsf2013/net.clp))
  (load* (resolve-file llsf2013/utils.clp))
  (load* (resolve-file llsf2013/skills.clp))
  (if
    (any-factp ((?conf confval))
      (and (eq ?conf:path "/clips-agent/llsf2013/enable-sim")
	   (eq ?conf:type BOOL) (eq ?conf:value true)))
  then
    (printout t "Loading simulation" crlf)
    (load* (resolve-file llsf2013/sim.clp))
  )
  (load* (resolve-file llsf2013/worldmodel.clp))
  (load* (resolve-file llsf2013/production.clp))
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
