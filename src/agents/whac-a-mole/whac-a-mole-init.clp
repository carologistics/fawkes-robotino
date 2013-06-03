

;---------------------------------------------------------------------------
;  whac-a-mole.clp - CLIPS test tools
;
;  Created: Thu Dec 20 12:11:17 2012 (Train from Freiburg to Aachen)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(blackboard-add-interface "Position3DInterface" "Light")
(blackboard-add-interface "Position3DInterface" "Pose")

(printout t "==== INIT ===== " crlf)
