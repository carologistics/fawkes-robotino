;---------------------------------------------------------------------------
;  goal-commitment.clp - Commit to goals after acquiring all necessary locks
;
;  Created: Fri 27 Apr 2018 20:48:00 CEST
;  Copyright  2018  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.
;

(defrule goal-commitment-no-lock-required
  ?g <- (goal (mode EXPANDED) (id ?goal-id) (params $?params))
  (not (goal-commitment-pending ?))
  (test (eq (nth$ 1 (goal-to-lock ?goal-id ?params)) nil))
  =>
  ;(printout t "Goal " ?goal-id " (params " ?params ")"
  ;            " does not require any locks, committing" crlf)
  (modify ?g (mode COMMITTED))
)

(defrule goal-commitment-request
  ?g <- (goal (mode EXPANDED) (id ?goal-id) (params $?params))
  (test (neq (nth$ 1 (goal-to-lock ?goal-id ?params)) nil))
  (not (goal-commitment-pending ?))
  =>
  (foreach ?lock (goal-to-lock ?goal-id ?params)
    (printout warn "Requesting " ?lock crlf)
    (mutex-try-lock-async ?lock ?*GOAL-COMMITMENT-AUTO-RENEW*)
  )
  (assert (goal-commitment-pending ?goal-id))
)

(defrule goal-commitment-commit
  ?g <- (goal (mode EXPANDED) (id ?goal-id) (params $?params))
  ; We requested at least one lock.
  (mutex (name ?name&:(member$ ?name (goal-to-lock ?goal-id ?params)))
         (request LOCK))
  ; All locks are acquired.
  (not (mutex (name ?name&:(member$ ?name (goal-to-lock ?goal-id ?params)))
              (request LOCK) (response ~ACQUIRED)))
  ?c <- (goal-commitment-pending ?goal-id)
  =>
  (printout warn "COMMITTING to " ?goal-id ", all locks acquired!" crlf)
  (delayed-do-for-all-facts ((?mutex mutex))
               (member$ ?mutex:name (goal-to-lock ?goal-id ?params))
    (modify ?mutex (request NONE) (response NONE))
  )
  (modify ?g (mode COMMITTED))
  (retract ?c)
)

(defrule goal-commitment-reject-unlock
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  ?g <- (goal (mode EXPANDED) (id ?goal-id) (params $?params))
  ?m <- (mutex (name ?name&:(member$ ?name (goal-to-lock ?goal-id ?params)))
               (request LOCK) (response REJECTED))
  (not (mutex (name ?name&:(member$ ?name (goal-to-lock ?goal-id ?params)))
              (request LOCK) (response PENDING)))
  ?c <- (goal-commitment-pending ?goal-id)
  =>
  (printout warn "REJECTING " ?goal-id ", failed to acquire lock " ?name ", "
                 "releasing other locks" crlf)
  (delayed-do-for-all-facts ((?mutex mutex))
               (member$ ?mutex:name (goal-to-lock ?goal-id ?params))
    (switch ?mutex:response
      (case ACQUIRED then (mutex-unlock-async ?mutex))
      (case NONE then (modify ?mutex (request NONE)))
      (case REJECTED then (modify ?mutex (request NONE) (response NONE)))
    )
  )
)

(defrule goal-commitment-reject
  (declare (salience ?*SALIENCE-GOAL-REJECT*))
  ?g <- (goal (mode EXPANDED) (id ?goal-id) (params $?params))
;  ?m <- (mutex (name ?name&:(member$ ?name (goal-to-lock ?goal-id ?params)))
;               (request LOCK) (response REJECTED))
  (not (mutex (name ?name&:(member$ ?name (goal-to-lock ?goal-id ?params)))
              (request LOCK|UNLOCK)))
  ?c <- (goal-commitment-pending ?goal-id)
  =>
  (printout warn "Released all locks for " ?goal-id " -> goal REJECTED" crlf)
  (modify ?g (mode FINISHED) (outcome REJECTED))
  (retract ?c)
)
