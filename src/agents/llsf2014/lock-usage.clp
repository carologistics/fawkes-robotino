;---------------------------------------------------------------------------
;  lock-usage.clp - usage of lock mechanisms
;                   Acts as a layer of lock-requesting,waiting for lock
;                   and freeing lock around skill execution
;
;  Created: Fri Feb 28 19:16:02 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

;;;;;;;;;;;;;;;;
; general rules
;;;;;;;;;;;;;;;;
(defrule lock-use-get-lock
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (wait-for-lock (res ?res) (state new) (priority ?p))
  =>
  (if (debug 3) then (printout t "Acquiring Lock for " ?res crlf))
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?res) (priority ?p)))
  (modify ?lae (state get))
  ; Retract all lock releases for ?res that gets the lock
  (do-for-all-facts ((?release lock)) (and (eq ?release:agent ?*ROBOT-NAME*)
                                           (eq ?release:resource ?res)
                                           (eq ?release:type RELEASE))
    (retract ?release)
  )
)

(defrule lock-use-execute
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (wait-for-lock (res ?res) (state get))
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  =>
  (if (debug 3) then (printout t "Lock accepted for " ?res crlf))
  (retract ?l)
  (modify ?lae (state use))
)
 
(defrule lock-use-finished
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (wait-for-lock (res ?res) (state finished))
  (Position3DInterface (id "Pose") (translation $?pos))
  =>
  (retract ?lae)
  (if (debug 3) then (printout t "Want to release Lock for " ?res ", waiting till 0.5m away" crlf))
  (assert (release-after-left ?res $?pos))
)

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

(defrule lock-drive-to-waiting-point
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?lae <- (wait-for-lock (res ?res) (state get))
  ?l <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  (wait-point ?res ?wait-point)
  (not (driving-ro-wait-point))
  =>
  (printout t "Waiting for lock of " ?res " at " ?wait-point crlf)
  (skill-call ppgoto place (str-cat ?wait-point))
  (assert (driving-ro-wait-point))
)

(defrule retract-driving-to-wait-point
  ?s <- (driving-ro-wait-point)
  (not (state WAIT_AND_LOOK_FOR_ALTERATIVE))
  =>
  (retract ?s)
)

(defrule lock-start-waiting-and-searching-for-alternative
  (phase PRODUCTION)
  (wait-for-lock (res ?res) (state get))
  (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  =>
  (assert (state WAIT_AND_LOOK_FOR_ALTERATIVE))
)

(defrule lock-end-waiting-and-searching-for-alternative
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  (phase PRODUCTION)
  ?sf <- (state WAIT_AND_LOOK_FOR_ALTERATIVE)
  (wait-for-lock (res ?) (state use))
  =>
  (retract ?sf)
)
