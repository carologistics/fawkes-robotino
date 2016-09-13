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

(defrule rm-log-task
  ; log all actions for later analysis about the plan
  (declare (salience ?*PRIORITY-WM-MID*))
  (time $?now)
  ?t <- (task (name ?name) (state ordered))
  =>
  (bind ?doc (rm-structured-fact-to-bson ?t))
  (bson-append-time ?doc "time" ?now)
  (robmem-insert "robmem.clips_action_log" ?doc)
  (bson-destroy ?doc)
)

(defrule rm-log-step
  ; log all actions for later analysis about the plan
  (declare (salience ?*PRIORITY-WM-MID*))
  (time $?now)
  ?s <- (step (name ?name) (state wait-for-activation))
  =>
  (bind ?doc (rm-structured-fact-to-bson ?s))
  (bson-append-time ?doc "time" ?now)
  (robmem-insert "robmem.clips_action_log" ?doc)
  (bson-destroy ?doc)
)

; (defrule rm-log-state
;   ; log all actions for later analysis about the plan
;   (declare (salience ?*PRIORITY-WM-MID*))
;   (time $?now)
;   ?s <- (state ?)
;   ?t <- (input-storage $?)
;   =>
;   (bind ?doc (rm-ordered-fact-to-bson ?s))
;   (bson-append-time ?doc "time" ?now)
;   (robmem-insert "robmem.clips_action_state" ?doc)
;   (bson-destroy ?doc)
; )

(defrule rm-test-fact-from-bson
  ; log all actions for later analysis about the plan
  (state PAUSED)
  (time $?now)
  =>
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" "ordered")
  (bson-append ?doc "values" "single")
  (rm-assert-from-bson ?doc)
  (bson-destroy ?doc)
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" "ordered")
  (bson-append-time ?doc "time" ?now)
  (rm-assert-from-bson ?doc)
  (bson-destroy ?doc)
  (bind ?doc (bson-create))
  (bson-append ?doc "relation" "machine")
  (bson-append ?doc "name" "TST")
  (bson-append-array ?doc "incomint" (create$ two things))
  (rm-assert-from-bson ?doc)
  (bson-destroy ?doc)  
)