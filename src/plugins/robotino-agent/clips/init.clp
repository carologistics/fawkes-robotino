
;---------------------------------------------------------------------------
;  init.clp - Robotino agent decision testing
;
;  Created: Sat Jun 16 12:34:54 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*CLIPS_DIR* = (get-clips-dir)
  ?*DEBUG* = 2  ;debug levels: 0 ~ none, 1 ~ minimal, 2 ~ more, 3 ~ maximum
)

(deffunction resolve-file (?file)
  (return (str-cat ?*CLIPS_DIR* ?file))
)

(load* (resolve-file priorities.clp))

(defrule enable-debug
  (declare (salience ?*PRIORITY_HIGH*))
  ?e <- (enable-debug)
  =>
  (printout t "Robotino Agent: enabling debugging" crlf)
  (retract ?e)
  (watch facts)
  (watch rules)
)

(defrule enable-sim-load
  ?e <- (enable-sim ?randomization)
  =>
  (printout t "Loading simulation" crlf)
  (load* (resolve-file sim.clp))
)

(defrule enable-skills
  ?e <- (enable-skills)
  =>
  (printout t "Enabling skill execution" crlf)
  (retract ?e)
  (load* (resolve-file skills.clp))
)

;(dribble-on "trace.txt")

(load* (resolve-file utils.clp))
(load* (resolve-file time.clp))
(load* (resolve-file facts.clp))
(load* (resolve-file worldmodel.clp))
(load* (resolve-file rules.clp))

(reset)
(run)



;(assert (holding S0))
;(facts)
