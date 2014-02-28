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
  (slot skill (type SYMBOL) (allowed-values get-s0))
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

;;;;;;;;;;;;;;;;
;get-S0
;;;;;;;;;;;;;;;;
(defrule lock-use-lock-get-s0
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill get-s0) (res INS) (state new))
  =>
  (if (debug 3) then (printout t "Acquiring Lock for INS" crlf))
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource INS)))
  (modify ?lae (state get))
)

;TODO: waiting positions
; (defrule prod-wait-for-gets0-at-insert-area
;   (phase PRODUCTION)
;   ?sf <- (state PROD_LOCK_REQUIRED_GET-S0)
;   ?l <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource INS))
;   (ins-wait-point ?ins-wait-point)
;   =>
;   (retract ?l)
;   (printout t "Waiting for lock of INS at " ?ins-wait-point crlf)
;   (skill-call ppgoto place (str-cat ?ins-wait-point))
; )

(defrule lock-use-execute-get-s0
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill get-s0) (res INS) (state get))
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource INS))
  =>
  (if (debug 3) then (printout t "Lock accepted -> Get S0" crlf))
  (retract ?l)
  (assert (state GET-S0))
  (modify ?lae (state exe))
  (get-s0)
)

(defrule lock-use-get-s0-done
  (declare (salience ?*PRIORITY-LOCK_USAGE*))
  ?lae <- (lock-and-execute (skill get-s0) (res INS) (state exe))
  ?sf <- (state GET-S0-FINAL|GET-S0-FAILED)
  (Position3DInterface (id "Pose") (translation $?pos))
  =>
  (retract ?lae)
  (if (debug 3) then (printout t "Want to release Lock for INS, waiting till 0.5m away" crlf))
  (assert (release-after-left INS $?pos))
)
