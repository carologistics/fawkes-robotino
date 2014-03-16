;---------------------------------------------------------------------------
;  lock-usage.clp - usage of lock mechanisms
;                   Acts as a layer of lock-requesting,waiting for lock
;                   and freeing lock around skill execution
;
;  Created: Fri Feb 28 19:16:02 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate lock-and-execute
  (slot skill (type SYMBOL) (allowed-values get-s0 load-with produce))
  (slot res (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values new get exe) (default new))
)

;;;;;;;;;;;;;;;;
; general rules
;;;;;;;;;;;;;;;;

;release a lock after driven ?*RELEASE-DISTANCE* m away from the position
(defrule lock-use-release-after-left
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?ral <- (release-after-left ?res $?oldPos)
  (Position3DInterface (id "Pose") (translation $?pos&:(> (distance (nth$ 1 ?pos) (nth$ 2 ?pos) (nth$ 1 ?oldPos) (nth$ 2 ?oldPos)) ?*RELEASE-DISTANCE*)))
  =>
  (printout t "Released " ?res crlf)
  (retract ?ral)
  (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res)))
)

(defrule prod-drive-to-waiting-point
  (phase PRODUCTION)
  ?lae <- (lock-and-execute (skill ?) (res ?res) (state get))
  ?l <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  (wait-point ?res ?wait-point)
  =>
  (printout t "Waiting for lock of " ?res " at " ?wait-point crlf)
  (skill-call ppgoto place (str-cat ?wait-point))
)

;;;;;;;;;;;;;;;;
;get-S0
;;;;;;;;;;;;;;;;
(defrule lock-use-lock-get-s0
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill get-s0) (res ?res) (state new))
  =>
  (if (debug 3) then (printout t "Acquiring Lock for " ?res crlf))
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?res)))
  (modify ?lae (state get))
)

(defrule lock-use-execute-get-s0
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill get-s0) (res ?res) (state get))
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  =>
  (if (debug 3) then (printout t "Lock accepted -> Get S0" crlf))
  (retract ?l)
  (modify ?lae (state exe))
  (get-s0 ?res)
)

(defrule lock-use-get-s0-failed
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill get-s0) (res ?res) (state exe))
  ?sf <- (state GET-S0-FAILED)
  =>
  (if (debug 3) then (printout t "Get S0 failed -> retry to get S0" crlf))
  (retract ?sf)
  (get-s0 ?res)
) 

(defrule lock-use-get-s0-done
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill get-s0) (res ?res) (state exe))
  ?sf <- (state GET-S0-FINAL)
  (Position3DInterface (id "Pose") (translation $?pos))
  =>
  (retract ?lae)
  (if (debug 3) then (printout t "Want to release Lock for " ?res ", waiting till 0.5m away" crlf))
  (assert (release-after-left ?res $?pos))
)


;;;;;;;;;;;;;;;;
;load-with
;;;;;;;;;;;;;;;;
(defrule lock-use-lock-load-with
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill load-with) (res ?res) (state new))
  =>
  (if (debug 3) then (printout t "Acquiring Lock for " ?res crlf))
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?res)))
  (modify ?lae (state get))
)

(defrule lock-use-execute-load-with
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill load-with) (res ?res) (state get))
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  (machine (name ?res) (mtype ?mtype) (loaded-with $?lw))
  =>
  (if (debug 3) then (printout t "Lock accepted -> loading machine" crlf))
  (retract ?l)
  (modify ?lae (state exe))

  ;does finish puck has to wait?
  ;(we have to wait if we bring a puck without starting the production)  
  (if (or (eq ?mtype T5)
	  (and (or (eq ?mtype T3) (eq ?mtype T4)) (eq (length$ ?lw) 2))
	  (and (eq ?mtype T2) (eq (length$ ?lw) 1)))
    then
    (bind ?dont-wait true)   

    else
    (bind ?dont-wait false)
  )
  (finish_puck_at ?res ?mtype ?dont-wait)
)

(defrule lock-use-load-with-done
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill load-with) (res ?res) (state exe))
  ?sf <- (state GOTO-FINAL|GOTO-FAILED)
  (Position3DInterface (id "Pose") (translation $?pos))
  =>
  (retract ?lae)
  (if (debug 3) then (printout t "Want to release Lock for " ?res ", waiting till 0.5m away" crlf))
  (assert (release-after-left ?res $?pos))
)


;;;;;;;;;;;;;;;;
;produce
;;;;;;;;;;;;;;;;
(defrule lock-use-lock-produce
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill produce) (res ?res) (state new))
  =>
  (if (debug 3) then (printout t "Acquiring Lock for " ?res crlf))
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?res)))
  (modify ?lae (state get))
)

(defrule lock-use-execute-produce
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill produce) (res ?res) (state get))
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  (machine (name ?res) (mtype ?mtype))
  =>
  (if (debug 3) then (printout t "Lock accepted -> producing at machine" crlf))
  (retract ?l)
  (modify ?lae (state exe))
  (finish_puck_at ?res ?mtype false)
)

(defrule lock-use-produce-done
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill produce) (res ?res) (state exe))
  ?sf <- (state GOTO-FINAL|GOTO-FAILED)
  (Position3DInterface (id "Pose") (translation $?pos))
  =>
  (retract ?lae)
  (if (debug 3) then (printout t "Want to release Lock for " ?res ", waiting till 0.5m away" crlf))
  (assert (release-after-left ?res $?pos))
)
