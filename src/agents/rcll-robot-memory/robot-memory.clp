;---------------------------------------------------------------------------
;  robot-memory.clp - Robotino agent -- Connection to the robot-memory
;
;  Created: Thu Sep 01 15:39:15 2016
;  Copyright  2016 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule rm-save-worldmodel
  ; save the worldmodel which is synced between the robot in the robot memory
  (time $?now)
  ?s <- (timer (name save-wm-in-rm) (time $?t&:(timeout ?now ?t ?*WORLDMODEL-SYNC-PERIOD*)) (seq ?seq))
  ;(lock-role MASTER)
  (wm-sync-info (synced-templates $?templates))
  (phase PRODUCTION)
  =>
  ; (printout t "Saving WM in RM" crlf)
  (progn$ (?templ ?templates)
    ;save all facts of this template
    (delayed-do-for-all-facts ((?fact ?templ)) TRUE
      ;save this fact as a document
      (bind ?doc (rm-structured-fact-to-bson ?fact))
      (bind ?query-for-sync-id (str-cat "{\"sync-id\": " (fact-slot-value ?fact sync-id) "}"))
      (robmem-upsert "robmem.clipswm" ?doc ?query-for-sync-id)
      (bson-destroy ?doc)
    )
  )
  (modify ?s (time ?now) (seq (+ ?seq 1)))
)