;---------------------------------------------------------------------------
;  robot-memory.clp - Robotino agent -- Connection to the robot-memory
;
;  Created: Thu Sep 01 15:39:15 2016
;  Copyright  2016 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule rm-clear-old-worldmodel-on-new-start
  (phase SETUP|PRE_GAME)
  (not (rm-cleared-old-wm))
  =>
  (printout t "Clearing old worl dmodel in robot memory" crlf)
  (robmem-remove "syncedrobmem.clipswm" "{}")
  ;TODO: verify if this worked
  (assert (rm-cleared-old-wm))
)

(defrule rm-save-worldmodel
  ; save the worldmodel which is synced between the robot in the robot memory
  (time $?now)
  ?s <- (timer (name save-wm-in-rm) (time $?t&:(timeout ?now ?t ?*WORLDMODEL-SYNC-PERIOD*)) (seq ?seq))
  (lock-role MASTER)
  (wm-sync-info (synced-templates $?templates))
  (phase EXPLORATION|PRODUCTION)
  =>
  ; (printout t "Saving WM in RM" crlf)
  (progn$ (?templ ?templates)
    ;save all facts of this template
    (delayed-do-for-all-facts ((?fact ?templ)) TRUE
      ;save this fact as a document
      (bind ?doc (rm-structured-fact-to-bson ?fact))
      (bind ?query-for-sync-id (str-cat "{\"sync-id\": " (fact-slot-value ?fact sync-id) "}"))
      (robmem-upsert "syncedrobmem.clipswm" ?doc ?query-for-sync-id)
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
  (robmem-insert "syncedrobmem.clips_action_log" ?doc)
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
  (robmem-insert "syncedrobmem.clips_action_log" ?doc)
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
;   (robmem-insert "syncedrobmem.clips_action_state" ?doc)
;   (bson-destroy ?doc)
; )