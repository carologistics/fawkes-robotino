
;---------------------------------------------------------------------------
;  init.clp - Robotino agent decision testing
;
;  Created: Sat Jun 16 12:34:54 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*CLIPS_DIR* = (get-clips-dir)
  ?*DEBUG* = 3  ;debug levels: 0 ~ none, 1 ~ minimal, 2 ~ more, 3 ~ maximum
)

(deffunction debug (?level)
  (return (<= ?level ?*DEBUG*))
)

(deffunction resolve-file (?file)
  (return (str-cat ?*CLIPS_DIR* ?file))
)

(watch facts)
(watch rules)

(load* (resolve-file priorities.clp))
(load* (resolve-file time.clp))
(load* (resolve-file facts.clp))
(load* (resolve-file worldmodel.clp))
(load* (resolve-file sim.clp))
(load* (resolve-file rules.clp))

(reset)
(run)
(printout t "Robotino Agent initialization complete" crlf)

;(assert (holding S0))

(facts)

