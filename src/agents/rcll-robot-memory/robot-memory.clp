;---------------------------------------------------------------------------
;  robot-memory.clp - Robotino agent -- Connection to the robot-memory
;
;  Created: Thu Sep 01 15:39:15 2016
;  Copyright  2016 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule rm-clear-old-worldmodel-on-new-start
  "clear old worldmodel and register triggers for new updates"
  (phase SETUP|PRE_GAME)
  (not (rm-cleared-old-wm))
  =>
  (printout t "Clearing old world model in robot memory" crlf)
  (bind ?remove-query (bson-create))
  (robmem-remove "syncedrobmem.clipswm" ?remove-query)
  (bson-destroy ?remove-query)
  (assert (rm-cleared-old-wm))

  ; create triggers to keep worldmodel in sync with robot memory
  (bind ?query (bson-create))
  (bind ?trigger (robmem-trigger-register "syncedrobmem.clipswm" ?query "robmem-wm"))
  (bson-destroy ?query)
  ; memory trigger to be able to remove it
  (assert (registered-trigger "syncedrobmem.clipswm" ?trigger))
)

(defrule rm-save-worldmodel
  ; save the initial worldmodel which is synced between the robot in the robot memory
  (time $?now)
  (lock-role MASTER)
  (wm-sync-info (synced-templates $?templates))
  (phase EXPLORATION|PRODUCTION)
  (not (rm-saved-initial-wm))
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
  (assert (rm-saved-initial-wm))
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

(deffunction rm-update-fact (?fact)
  ; update in synced robot memory
  (bind ?doc (rm-structured-fact-to-bson ?fact))
  (bind ?query (bson-create))
  (bson-append ?query "sync-id" (fact-slot-value ?fact sync-id))
  (robmem-update "syncedrobmem.clipswm" ?doc ?query)
  (bson-destroy ?doc)
  (bson-destroy ?query)
)

(deffunction rm-insert-fact (?fact)
  ; insert in synced robot memory
  (bind ?doc (rm-structured-fact-to-bson ?fact))
  (robmem-insert "syncedrobmem.clipswm" ?doc)
  (bson-destroy ?doc)
)

(deffunction rm-remove-fact (?fact)
  ; remove in synced robot memory by the fact's sync id
  (bind ?remove-query (bson-create))
  (bson-append ?remove-query "sync-id" (fact-slot-value ?fact sync-id))
  (robmem-remove "syncedrobmem.clipswm" ?remove-query)
  (bson-destroy ?remove-query)
)