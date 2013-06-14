
;---------------------------------------------------------------------------
;  rules.clp - Robotino agent decision testing -- rules
;
;  Created: Sat Jun 16 12:35:16 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; --- RULES - skill done

(defrule prod-get-s0-done
  (phase PRODUCTION)
  ?sf <- (state GET-S0-FINAL|GET-S0-FAILED)
  =>
  (retract ?sf)
  (if (debug 3) then (printout t "Release Lock for INS" crlf))
  (assert (state IDLE)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource INS))
  )
)

(defrule prod-goto-final
  (phase PRODUCTION)
  ?sf <- (state GOTO-FINAL)
  (not (goto-target ?))
  ?f <- (goto-has-locked ?goal)
  =>
  (retract ?sf ?f)
  (assert (state IDLE)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?goal))
  )
)

(defrule prod-goto-failed
  (phase PRODUCTION)
  ?sf <- (state GOTO-FAILED)
  (not (goto-target ?))
  ?f <- (goto-has-locked ?goal)
  =>
  (retract ?sf ?f)
  (assert (state IDLE)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?goal))
  )
)

; --- RULES - next machine/place to go to

(defrule prod-get-s0
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (holding NONE)
  =>
  (if (debug 3) then (printout t "Need to get S0" crlf))
  (if (debug 3) then (printout t "Requirering Lock for INS" crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GET-S0)
	  (lock (type GET) (agent ?*ROBOT-NAME*) (resource INS))
  )
)

(defrule prod-figure-out-waiting-for-ins-point
  (phase PRODUCTION)
  (confval (path "/clips-agent/llsf2013/wait-for-ins-point") (value ?ins-wait-point))
  =>
  (assert (ins-wait-point ?ins-wait-point))
)

(defrule prod-wait-for-gets0-at-insert-area
  (phase PRODUCTION)
  ?sf <- (state PROD_LOCK_REQUIRED_GET-S0)
  ?l <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource INS))
  (ins-wait-point ?ins-wait-point)
  =>
  (printout t "Waiting for lock of INS at " ?ins-wait-point crlf)
  (skill-call ppgoto place (str-cat ?ins-wait-point))
)

(defrule prod-execute-get-s0
  (phase PRODUCTION)
  ?sf <- (state PROD_LOCK_REQUIRED_GET-S0)
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource INS))
  =>
  (if (debug 3) then (printout t "Lock accepted -> Get S0" crlf))
  (retract ?sf ?l)
  (assert (state GET-S0))
  (get-s0)
)

(defrule prod-s0-t5
  (declare (salience ?*PRIORITY-P*))
  (phase PRODUCTION)
  (role EXPLORATION_P3)
  ?sf <- (state IDLE)
  (holding S0)
  (machine (mtype T5) (name ?name) (allowed TRUE))
  =>
  (if (debug 2) then (printout t "S0 2 -- Going to T5 named " ?name crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO ?name))  
)

(defrule prod-s0-t3-s1-s2
  (declare (salience ?*PRIORITY-P*))
  (phase PRODUCTION)
  (role EXPLORATION_P1P2)
  ?sf <- (state IDLE)
  (holding S0)
  (machine (mtype T3|T4) (loaded-with $?l&:(subsetp (create$ S1 S2) ?l)) (name ?name) (allowed TRUE))
  =>
  (if (debug 2) then (printout t "S0 1 -- Going to T3 named " ?name crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
)

(defrule prod-s0-t2-s1
  (declare (salience ?*PRIORITY-S1*))
  (phase PRODUCTION)
  (role EXPLORATION_P1P2)
  ?sf <- (state IDLE)
  (holding S0)
  (machine (mtype T2) (loaded-with $?l&:(member$ S1 ?l)) (name ?name) (allowed TRUE))
  =>
  (if (debug 2) then (printout t "S0 4 -- Going to T2 named " ?name crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
)

(defrule prod-s0-t1
  (declare (salience ?*PRIORITY-T1*))
  (phase PRODUCTION)
  (role EXPLORATION_P1P2)
  ?sf <- (state IDLE)
  (holding S0)
  (machine (mtype T1) (name ?name) (allowed TRUE))
  =>
  (if (debug 2) then (printout t "S0 5 -- Going to T1 named " ?name crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
)

(defrule prod-s1-t3-s2
  (declare (salience ?*PRIORITY-S2*))
  (phase PRODUCTION)
  (role EXPLORATION_P1P2)
  ?sf <- (state IDLE)
  (holding S1)
  (machine (mtype T3|T4) (loaded-with $?l&:(subsetp (create$ S2) ?l)) (name ?name) (allowed TRUE))
  =>
  (if (debug 2) then (printout t "S1 1 -- Going T3 named " ?name crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
)

(defrule prod-s1-t2-not-s1
  (declare (salience ?*PRIORITY-T2*))  
  (phase PRODUCTION)
  (role EXPLORATION_P1P2)
  ?sf <- (state IDLE)
  (holding S1)
  (machine (mtype T2) (name ?name) (loaded-with $?l&~:(subsetp (create$ S1) ?l)) (allowed TRUE))
  =>
  (if (debug 2) then (printout t "S1 3 -- Going to T2 named " ?name crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
)

(defrule prod-s2-t3-has-some-but-not-s2
  (declare (salience ?*PRIORITY-T3*))  
  (phase PRODUCTION)
  (role EXPLORATION_P1P2)
  ?sf <- (state IDLE)
  (holding S2)
  (machine (mtype T3|T4) (name ?name)
	   (loaded-with $?l&:(> (length$ ?l) 0)&~:(subsetp (create$ S2) ?l)) (allowed TRUE))
  =>
  (if (debug 2) then (printout t "S2 1 -- Going to T3 named " ?name crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
)

(defrule prod-s2-t3-empty
  (declare (salience ?*PRIORITY-T3*))  
  (phase PRODUCTION)
  (role EXPLORATION_P1P2)
  ?sf <- (state IDLE)
  (holding S2)
  (machine (mtype T3|T4) (name ?name) (loaded-with $?l&:(= (length$ ?l) 0)) (allowed TRUE))
  =>
  (if (debug 2) then (printout t "S2 1 -- Going to T3 named " ?name crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
)


(defrule prod-deliver-p
  (declare (salience ?*PRIORITY-DELIVER*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (holding P1|P2|P3)
  =>
  (if (debug 2) then (printout t "P -- Need to deliver P " crlf))
  (retract ?sf)
  (assert (state PROD_LOCK_REQUIRED_GOTO deliver))
)

(defrule prod-goto-request-lock
  (phase PRODUCTION)
  (state PROD_LOCK_REQUIRED_GOTO ?goal)
  =>
  (if (debug 2) then (printout t "Requirering lock of " ?goal crlf))
  (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?goal)))
)

(defrule prod-execute-goto-machine
  (phase PRODUCTION)
  ?sf <- (state PROD_LOCK_REQUIRED_GOTO ?goal)
  ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?goal))
  =>
  (retract ?sf ?l)
  (assert (goto-has-locked ?goal))
  (goto-machine ?goal)
)
