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
  (bind ?worldmodel (pb-create "llsf_msgs.Worldmodel"))
  (progn$ (?templ ?templates)
    ;save all facts of this template
    (delayed-do-for-all-facts ((?fact ?templ)) TRUE
      ;save this fact as a document
      (bind ?doc (bson-create))
      (bson-append ?doc "type" (str-cat ?templ))
      (bson-append ?doc "sync_id" (fact-slot-value ?fact sync-id))
      ;append kv-pair for each field
      (progn$ (?slot (fact-slot-names ?fact))
        (if (neq ?slot sync-id) then
          (if (deftemplate-slot-multip ?templ ?slot)
            then
            ; append multifield as array
            (bson-append-array ?doc ?slot (fact-slot-value ?fact ?slot))
            else
            ; appand value directly for singlefields
            (bson-append ?doc ?slot (fact-slot-value ?fact ?slot))
          )
        )
      )
      (bind ?query-for-sync-id (str-cat "{\"sync_id\": " (fact-slot-value ?fact sync-id) "}"))
      (robmem-upsert "robmem.clips" ?doc ?query-for-sync-id)
      (bson-destroy ?doc)
    )
  )
  (modify ?s (time ?now) (seq (+ ?seq 1)))
)