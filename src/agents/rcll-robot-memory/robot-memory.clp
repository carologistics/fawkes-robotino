;---------------------------------------------------------------------------
;  robot-memory.clp - Robotino agent -- Connection to the robot-memory
;
;  Created: Thu Sep 01 15:39:15 2016
;  Copyright  2016 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


(defrule rm-test
  (phase EXPLORATION)
  =>
  (printout warn "using RM" crlf)
  (bind ?doc (bson-create))
  (bson-append ?doc "hello" "world")
  (robmem-insert "robmem.clips" ?doc)
  (bson-destroy ?doc)
  (printout warn "mission accomplished" crlf)
)
