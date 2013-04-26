
;---------------------------------------------------------------------------
;  rules.clp - Robotino agent decision testing -- rules
;
;  Created: Sat Jun 16 12:35:16 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


; --- RULES - wait for start signal
;(defrule start
;  ?s <- (state WAIT_START)
;  (start)
;  =>
;  (retract ?s)
;  (assert (state IDLE))
;)

; --- GAME STATE handling

(defrule change-phase
  ?cf <- (change-phase ?phase)
  ?pf <- (phase ?)
  =>
  (retract ?cf ?pf)
  (assert (phase ?phase))
)

(defrule change-state-ignore
  (phase ~PRODUCTION)
  ?cf <- (change-state ?)
  =>
  (retract ?cf)
)

(defrule start
  (phase PRODUCTION)
  ?sf <- (state WAIT_START)
  ?cf <- (change-state RUNNING)
  ?rf <- (refbox-state ?)
  =>
  (retract ?sf ?cf ?rf)
  (assert (state IDLE))
  (assert (refbox-state RUNNING))
)

(defrule pause
  ?sf <- (state ?state&~PAUSED)
  ?cf <- (change-state ?cs&~RUNNING)
  ?rf <- (refbox-state ~?cs)
  =>
  (retract ?sf ?cf ?rf)
  (motor-disable)
  (assert (state PAUSED))
  (assert (refbox-state PAUSED))
  (assert (pre-pause-state ?state))
)

(defrule unpause
  ?sf <- (state PAUSED)
  ?pf <- (pre-pause-state ?old-state)
  ?cf <- (change-state RUNNING)
  ?rf <- (refbox-state PAUSED)
  =>
  (retract ?sf ?pf ?cf ?rf)
  (assert (state ?old-state))
  (assert (refbox-state RUNNING))
  (motor-enable)
)


; --- RULES - skill done

(defrule get-s0-done
  ?sf <- (state GET-S0-FINAL|GET-S0-FAILED)
  =>
  (retract ?sf)
  (assert (state IDLE))
)

(defrule goto-final
  ?sf <- (state GOTO-FINAL)
  (not (goto-target ?))
  =>
  (retract ?sf)
  (assert (state IDLE))
)

(defrule goto-failed
  ?sf <- (state GOTO-FAILED)
  (not (goto-target ?))
  =>
  (retract ?sf)
  (assert (state IDLE))
)

; --- RULES - next machine/place to go to

(defrule get-s0
  ?sf <- (state IDLE)
  (holding NONE)
  =>
  (if (debug 3) then (printout t "Need to get S0" crlf))
  (retract ?sf)
  (assert (state GET-S0))
  (get-s0)
)

(defrule s0-t5
  (declare (salience ?*PRIORITY-P*))
  ?sf <- (state IDLE)
  (holding S0)
  (machine (mtype T5) (name ?name))
  =>
  (if (debug 2) then (printout t "S0 2 -- Going to T5 named " ?name crlf))
  (retract ?sf)
  (goto-machine ?name)  
)

(defrule s0-t3-s1-s2
  (declare (salience ?*PRIORITY-P*))
  ?sf <- (state IDLE)
  (holding S0)
  (machine (mtype T3) (loaded-with $?l&:(subsetp (create$ S1 S2) ?l)) (name ?name))
  =>
  (if (debug 2) then (printout t "S0 1 -- Going to T3 named " ?name crlf))
  (retract ?sf)
  (goto-machine ?name)
)

(defrule s0-t2-s1
  (declare (salience ?*PRIORITY-S1*))
  ?sf <- (state IDLE)
  (holding S0)
  (machine (mtype T2) (loaded-with $?l&:(member$ S1 ?l)) (name ?name))
  =>
  (if (debug 2) then (printout t "S0 4 -- Going to T2 named " ?name crlf))
  (retract ?sf)
  (goto-machine ?name)
)

(defrule s0-t1
  (declare (salience ?*PRIORITY-T1*))
  ?sf <- (state IDLE)
  (holding S0)
  (machine (mtype T1) (name ?name))
  =>
  (if (debug 2) then (printout t "S0 5 -- Going to T1 named " ?name crlf))
  (retract ?sf)
  (goto-machine ?name)
)

(defrule s1-t3-s2
  (declare (salience ?*PRIORITY-S2*))
  ?sf <- (state IDLE)
  (holding S1)
  (machine (mtype T3) (loaded-with $?l&:(subsetp (create$ S2) ?l)) (name ?name))
  =>
  (if (debug 2) then (printout t "S1 1 -- Going T3 named " ?name crlf))
  (retract ?sf)
  (goto-machine ?name)
)

(defrule s1-t2-not-s1
  (declare (salience ?*PRIORITY-T2*))  
  ?sf <- (state IDLE)
  (holding S1)
  (machine (mtype T2) (name ?name) (loaded-with $?l&~:(subsetp (create$ S1) ?l)))
  =>
  (if (debug 2) then (printout t "S1 3 -- Going to T2 named " ?name crlf))
  (retract ?sf)
  (goto-machine ?name)
)

(defrule s2-t3-has-some-but-not-s2
  (declare (salience ?*PRIORITY-T3*))  
  ?sf <- (state IDLE)
  (holding S2)
  (machine (mtype T3) (name ?name)
	   (loaded-with $?l&:(> (length$ ?l) 0)&~:(subsetp (create$ S2) ?l)))
  =>
  (if (debug 2) then (printout t "S2 1 -- Going to T3 named " ?name crlf))
  (retract ?sf)
  (goto-machine ?name)
)

(defrule s2-t3-empty
  (declare (salience ?*PRIORITY-T3*))  
  ?sf <- (state IDLE)
  (holding S2)
  (machine (mtype T3) (name ?name) (loaded-with $?l&:(= (length$ ?l) 0)))
  =>
  (if (debug 2) then (printout t "S2 1 -- Going to T3 named " ?name crlf))
  (retract ?sf)
  (goto-machine ?name)
)


(defrule deliver-p
  (declare (salience ?*PRIORITY-DELIVER*))
  ?sf <- (state IDLE)
  (holding P1|P2|P3)
  =>
  (if (debug 2) then (printout t "P -- Need to deliver P " crlf))
  (retract ?sf)
  (goto-machine deliver)
)
