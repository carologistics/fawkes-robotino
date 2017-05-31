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
  "Request a lock for a resource we want to use."
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
  "Process accepted locks and state for the resource that it is in use."
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (wait-for-lock (res ?res) (state get))
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  =>
  (if (debug 3) then (printout t "Lock accepted for " ?res crlf))
  (retract ?l)
  (modify ?lae (state use))
)
 
(defrule lock-use-finished
  "When the locked resource we used is no longer used, prepare for lock-release.
  Lock is released after ?*RELEASE-DISTANCE* m from the current position."
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (wait-for-lock (res ?res) (state finished))
  (Position3DInterface (id "Pose") (translation $?pos))
  =>
  (retract ?lae)
  (if (debug 3) then (printout t "Want to release Lock for " ?res ", waiting till 0.5m away" crlf))
  (assert (release-after-left ?res $?pos))
)

(defrule lock-use-release-after-left
  "Release a lock after driven ?*RELEASE-DISTANCE* m away from the position."
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?ral <- (release-after-left ?res $?oldPos)
  (Position3DInterface (id "Pose") (translation $?pos&:(> (distance (nth$ 1 ?pos) (nth$ 2 ?pos) (nth$ 1 ?oldPos) (nth$ 2 ?oldPos)) ?*RELEASE-DISTANCE*)))
  =>
  (printout t "Released " ?res crlf)
  (retract ?ral)
  (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res)))
)

(defrule lock-drive-to-zone-waiting-point
  "If our lock for a resource was refused and there exists a wait-point for a near zone go to the wait-point."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?lae <- (wait-for-lock (res ?res) (state get) (place ?place))
  ?l <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  (not (driving-to-wait-point))
  (place-waitpoint-assignment (place ?place) (waitpoint ?nearest-wp))
  =>
  (printout t "Waiting for lock of " ?res " at " ?nearest-wp crlf)
  (skill-call ppgoto place (str-cat ?nearest-wp))
  (assert (driving-to-wait-point))
)

(defrule lock-drive-to-nearest-waiting-point
  "If our lock for a resource was refused and the place is not set for the resource go to the nearest wait-point."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?lae <- (wait-for-lock (res ?res) (state get))
  ?l <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  (wait-point ?wait-point)
  (not (driving-to-wait-point))
  (added-waiting-positions)
  =>
  ;find random waiting point
  (bind ?wpts (create$))
  (delayed-do-for-all-facts ((?wpt zone-waitpoint)) TRUE
    (bind ?wpts (create$ ?wpt:name ?wpts))
  )
  (if (> (length$ ?wpts) 0) then
    (bind ?index (random 1 (length$ ?wpts)))
    (bind ?wait-point (nth$ ?index ?wpts))
  )
  (printout t "Waiting for lock of " ?res " at " ?wait-point crlf)
  (skill-call ppgoto place (str-cat ?wait-point))
  (assert (driving-to-wait-point))
)

(defrule retract-driving-to-wait-point
  "Retracts the driving to wait point fact when we are not looking for an alternative."
  ?s <- (driving-to-wait-point)
  (not (state WAIT_AND_LOOK_FOR_ALTERATIVE))
  =>
  (retract ?s)
)

(defrule lock-start-waiting-and-searching-for-alternative
  "IF we are in production and our lock was refused keep waiting and look for alternative tasks."
  (phase PRODUCTION)
  (wait-for-lock (res ?res) (state get))
  (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  =>
  (assert (state WAIT_AND_LOOK_FOR_ALTERATIVE))
)

(defrule lock-end-waiting-and-searching-for-alternative
  "Stop waiting when a lock is in use while we are in production."
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  (phase PRODUCTION)
  ?sf <- (state WAIT_AND_LOOK_FOR_ALTERATIVE)
  (wait-for-lock (res ?) (state use))
  =>
  (retract ?sf)
)
