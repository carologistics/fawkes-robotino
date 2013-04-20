
;---------------------------------------------------------------------------
;  init.clp - Robotino agent decision testing
;
;  Created: Sat Jun 16 12:34:54 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; LLSF2013 agent includes
(load* (resolve-file llsf2013/priorities.clp))
(load* (resolve-file llsf2013/facts.clp))

(defrule load-config
  (init)
  =>
  (load-config "/clips-agent")
)

(deffacts init (init))

(defrule initialize
  (declare (salience ?*PRIORITY_HIGH*))
  (agent-init)
  (protobuf-available)
  =>
  (load-config "/clips-agent")

  (load* (resolve-file llsf2013/utils.clp))
  (if
    (any-factp ((?conf confval))
      (and (eq ?conf:path "/clips-agent/llsf2013/enable-sim")
	   (eq ?conf:type BOOL) (eq ?conf:value true)))
  then
    (printout t "Loading simulation" crlf)
    (load* (resolve-file llsf2013/sim.clp))
  )
  (load* (resolve-file llsf2013/skills.clp))
  (load* (resolve-file llsf2013/worldmodel.clp))
  (load* (resolve-file llsf2013/rules.clp))
  (reset)
  ;(facts)
)
