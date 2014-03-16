;---------------------------------------------------------------------------
;  production.clp - Planning for LLSF production
;                   Here we plan which job to execute when the robot is idle
;                   The job-execution is located in tasks.clp
;
;  Created: Sat Jun 16 12:35:16 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;                   Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule prod-load-T2-with-S0
  (declare (salience ?*PRIORITY-LOAD-T2-WITH-S0*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (machine (mtype T2) (loaded-with $?l&~:(member$ S0 ?l))
	   (incoming $?i&~:(member$ S0 ?i)) (name ?name))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  =>
  (printout t "PROD: Loading T2 " ?name " with S0" crlf)
  (retract ?sf)
  (assert (state TASK-PROPOSED)
	  (proposed-task (name load-with-S0) (args (create$ ?name)))
  )
)

(defrule prod-load-T2-with-S1
  (declare (salience ?*PRIORITY-LOAD-T2-WITH-S1*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (machine (mtype T1) (name ?name-T1))
  (machine (mtype T2) (loaded-with $?l&~:(member$ S1 ?l))
	   (incoming $?i&~:(member$ S1 ?i)) (name ?name-T2))
  (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T1 ?name-T2))) (state rejected)))
  =>
  (printout t "PROD: Loading T2 " ?name-T2 " with S1 after producing S1 at " ?name-T1 crlf)
  (retract ?sf)
  (assert (state TASK-PROPOSED)
	  (proposed-task (name load-with-S1) (args (create$ ?name-T1 ?name-T2)))
  )
)

(defrule prod-load-T3_T4-with-S0
  (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S0*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S0 ?l)) 
	   (incoming $?i&~:(member$ S0 ?i)) (name ?name))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  =>
  (printout t "PROD: Loading T3/T4 " ?name " with S0" crlf)
  (retract ?sf)
  (assert (state TASK-PROPOSED)
	  (proposed-task (name load-with-S0) (args (create$ ?name)))
  )
)

(defrule prod-load-T3_T4-with-S1
  (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (machine (mtype T1) (name ?name-T1))
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S1 ?l))
	   (incoming $?i&~:(member$ S1 ?i)) (name ?name-T3_T4))
  (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T1 ?name-T3_T4))) (state rejected)))
  =>
  (printout t "PROD: Loading T3/T4 " ?name-T3_T4 " with S1 after producing S1 at " ?name-T1 crlf)
  (retract ?sf)
  (assert (state TASK-PROPOSED)
	  (proposed-task (name load-with-S1) (args (create$ ?name-T1 ?name-T3_T4)))
  )
)

; (defrule prod-load-T3_T4-with-S2
;   (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S2*))
;   (phase PRODUCTION)
;   ?sf <- (state IDLE)
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S2 ?l)) 
; 	   (incoming $?i&~:(member$ S2 ?i)) (name ?name))
;   (not (proposed-task (name pick-and-load) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
;   =>
;   (printout t "PROD: Loading T3/T4 " ?name " with S2" crlf)
;   (retract ?sf)
;   (assert (state TASK-PROPOSED)
; 	  (proposed-task (name pick-and-load) (args (create$ ?name)))
;   )
; )

(defrule prod-load-T5-with-S0
  (declare (salience ?*PRIORITY-LOAD-T5-WITH-S0*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (machine (mtype T5) (loaded-with $?l&~:(member$ S0 ?l))
    (incoming $?i&~:(member$ S0 ?i)) (name ?name))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  =>
  (printout t "PROD: Loading T5 " ?name " with S0" crlf)
  (retract ?sf)
  (assert (state TASK-PROPOSED)
    (proposed-task (name load-with-S0) (args (create$ ?name)))
  )
)

; (defrule prod-get-consumed-done
;   (phase PRODUCTION)
;   ?sf <- (state GET-CONSUMED-FINAL|GET-CONSUMED-FAILED)
;   (Position3DInterface (id "Pose") (translation $?pos))
;   ?f <- (goto-has-locked ?machine)
;   =>
;   (retract ?sf ?f)
;   (printout t "prod-get-consumed-done" crlf)
;   (assert (state IDLE)
; 	  (want-to-release ?machine $?pos)
;   )
; )

; (defrule prod-goto-final
;   (phase PRODUCTION)
;   ?sf <- (state GOTO-FINAL)
;   (not (goto-target ?))
;   ?f <- (goto-has-locked ?goal)
;   (Position3DInterface (id "Pose") (translation $?pos))
;   =>
;   (retract ?sf ?f)
;   (assert (state IDLE)
; 	  (want-to-release ?goal $?pos)
;   )
; )

; (defrule prod-goto-failed
;   (phase PRODUCTION)
;   ?sf <- (state GOTO-FAILED)
;   (not (goto-target ?))
;   ?f <- (goto-has-locked ?goal)
;   (Position3DInterface (id "Pose") (translation $?pos))
;   =>
;   (retract ?sf ?f)
;   (assert (state IDLE)
; 	  (want-to-release ?goal $?pos)
;   )
; )

; (defrule prod-release-after-driving-away
;   ?wtr <- (want-to-release ?res $?oldPos)
;   (Position3DInterface (id "Pose") (translation $?pos&:(> (distance (nth$ 1 ?pos) (nth$ 2 ?pos) (nth$ 1 ?oldPos) (nth$ 2 ?oldPos)) ?*RELEASE-DISTANCE*)))
;   =>
;   (printout t "Released " ?res crlf)
;   (retract ?wtr)
;   (assert (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res)))
; )

; ; --- RULES - next machine/place to go to

; (defrule prod-get-s0
;   (declare (salience ?*PRIORITY-GET-S0-INS*))
;   (phase PRODUCTION)
;   ?sf <- (state IDLE)
;   (holding NONE)
;   =>
;   (if (debug 3) then (printout t "Need to get S0" crlf))
;   (if (debug 3) then (printout t "Requirering Lock for INS" crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GET-S0)
; 	  (want-to-get-lock INS)
;   )
; )

; (defrule prod-recycle-to-get-s0
;   (declare (salience ?*PRIORITY-GET-S0-RECYCLE*))
;   (phase PRODUCTION)
;   ?sf <- (state IDLE)
;   (holding NONE)
;   ?m <- (machine (junk ?n&:(> ?n 0)) (name ?goal&~M7))
;   (confval (path "/clips-agent/llsf2013/recycle") (type STRING) (value "ifpossible"))
;   =>
;   (if (debug 3) then (printout t "Need to get S0, Recycle to get one" crlf)
;                      (printout t "Requirering Lock for " ?goal crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_RECYCLE ?goal)
; 	  (want-to-get-lock ?goal)
;   )
; )

; (defrule prod-recycle-to-get-s0-after-deliver
;   (declare (salience ?*PRIORITY-GET-S0-RECYCLE*))
;   (phase PRODUCTION)
;   ?sf <- (state IDLE)
;   (holding NONE)
;   ?m <- (machine (junk ?n&:(> ?n 0)) (name ?goal&~M7))
;   (confval (path "/clips-agent/llsf2013/recycle") (type STRING) (value "afterdeliver"))
;   (delivered P1|P2)
;   =>
;   (if (debug 3) then (printout t "Need to get S0, Recycle to get one" crlf))
;   (if (debug 3) then (printout t "Requirering Lock for " ?goal crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_RECYCLE ?goal)
; 	  (want-to-get-lock ?goal)
;   )
; )

; (defrule prod-recycle-puck
;   (phase PRODUCTION)
;   ?sf <- (state IDLE)
;   (holding CO)
;   (machine (mtype RECYCLE) (name ?name))
;   (role ?role)
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "Recycling consumed puck" ?name crlf))
;   (printout t "prod-recycle-puck" crlf)
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name))    
; )

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

; (defrule prod-execute-get-s0
;   (phase PRODUCTION)
;   ?sf <- (state PROD_LOCK_REQUIRED_GET-S0)
;   ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource INS))
;   =>
;   (if (debug 3) then (printout t "Lock accepted -> Get S0" crlf))
;   (retract ?sf ?l)
;   (assert (state GET-S0))
;   (get-s0)
; )

; (defrule prod-execute-recycle
;   (phase PRODUCTION)
;   ?sf <- (state PROD_LOCK_REQUIRED_RECYCLE ?goal)
;   ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?goal))
;   =>
;   (printout t "Lock accepted -> Get Consumed Puck at " ?goal crlf)
;   (retract ?sf ?l)
;   (assert (state GET-CONSUMED)
; 	  (goto-has-locked ?goal))
;   (get-consumed ?goal)
; )

; (defrule prod-s0-t5
;   (declare (salience ?*PRIORITY-P*))
;   (phase PRODUCTION)
;   (role ?role)
;   ?sf <- (state IDLE)
;   (holding S0)
;   (machine (mtype T5) (name ?name))
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "S0 2 -- Going to T5 named " ?name crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name)
; 	  (want-to-produce-final-product);dirty robocup improvement
;   )  
; )

; (defrule prod-s0-t3-s1-s2
;   (declare (salience ?*PRIORITY-P*))
;   (phase PRODUCTION)
;   (role ?role)
;   ?sf <- (state IDLE)
;   (holding S0)
;   (machine (mtype T3|T4) (loaded-with $?l&:(subsetp (create$ S1 S2) ?l)) (name ?name))
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "S0 1 -- Going to T3 named " ?name crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name)
; 	  (want-to-produce-final-product);dirty robocup improvement
;   )
; )

; (defrule prod-s0-t2-s1
;   (declare (salience ?*PRIORITY-S1*))
;   (phase PRODUCTION)
;   (role ?role)
;   ?sf <- (state IDLE)
;   (holding S0)
;   (machine (mtype T2) (loaded-with $?l&:(member$ S1 ?l)) (name ?name))
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "S0 4 -- Going to T2 named " ?name crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
; )

; (defrule prod-s0-t1
;   (declare (salience ?*PRIORITY-T1*))
;   (phase PRODUCTION)
;   (role ?role)
;   ?sf <- (state IDLE)
;   (holding S0)
;   (machine (mtype T1) (name ?name))
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "S0 5 -- Going to T1 named " ?name crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
; )

; (defrule prod-s1-t3-s2
;   (declare (salience ?*PRIORITY-S2*))
;   (phase PRODUCTION)
;   (role ?role)
;   ?sf <- (state IDLE)
;   (holding S1)
;   (machine (mtype T3|T4) (loaded-with $?l&:(subsetp (create$ S2) ?l)) (name ?name))
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "S1 1 -- Going T3 named " ?name crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
; )

; (defrule prod-s1-t2-not-s1
;   (declare (salience ?*PRIORITY-T2*))  
;   (phase PRODUCTION)
;   (role ?role)
;   ?sf <- (state IDLE)
;   (holding S1)
;   (machine (mtype T2) (name ?name) (loaded-with $?l&~:(subsetp (create$ S1) ?l)))
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "S1 3 -- Going to T2 named " ?name crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
; )

; (defrule prod-s2-t3-has-some-but-not-s2
;   (declare (salience ?*PRIORITY-T3*))  
;   (phase PRODUCTION)
;   (role ?role)
;   ?sf <- (state IDLE)
;   (holding S2)
;   (machine (mtype T3|T4) (name ?name)
; 	   (loaded-with $?l&:(> (length$ ?l) 0)&~:(subsetp (create$ S2) ?l)))
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "S2 1 -- Going to T3 named " ?name crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
; )

; (defrule prod-s2-t3-empty
;   (declare (salience ?*PRIORITY-T3*))  
;   (phase PRODUCTION)
;   (role ?role)
;   ?sf <- (state IDLE)
;   (holding S2)
;   (machine (mtype T3|T4) (name ?name) (loaded-with $?l&:(= (length$ ?l) 0)))
;   (machine-alloc (machine ?name) (role ?role))
;   =>
;   (if (debug 2) then (printout t "S2 1 -- Going to T3 named " ?name crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO ?name))
; )


; (defrule prod-deliver-p
;   (declare (salience ?*PRIORITY-DELIVER*))
;   (phase PRODUCTION)
;   ?sf <- (state IDLE)
;   (holding P1|P2|P3)
;   (holding ?p)
;   =>
;   (if (debug 2) then (printout t "P -- Need to deliver P " crlf))
;   (retract ?sf)
;   (assert (state PROD_LOCK_REQUIRED_GOTO deliver))
;   (if (or (eq ?p P1) (eq ?p P2))
;     then
;     (assert (delivered-p1p2))
;   )
; )

; (defrule prod-goto-request-lock
;   (phase PRODUCTION)
;   (state PROD_LOCK_REQUIRED_GOTO ?goal)
;   =>
;   (if (debug 2) then (printout t "Requirering lock of " ?goal crlf))
;   (assert (want-to-get-lock ?goal))
; )

; ;TODO: look at this again after robocup:
; (defrule prod-execute-goto-machine-robocup-quickfix-verison
;   (phase PRODUCTION)
;   ?sf <- (state PROD_LOCK_REQUIRED_GOTO ?goal)
;   (machine (name ?goal) (mtype ?mtype))
;   ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?goal))
;   ?wtpfp <- (want-to-produce-final-product)
;   =>
;   (retract ?sf ?l ?wtpfp)
;   (assert (goto-has-locked ?goal))
;   (goto-machine-final-product ?goal ?mtype)
; )

; (defrule prod-execute-goto-machine
;   (phase PRODUCTION)
;   ?sf <- (state PROD_LOCK_REQUIRED_GOTO ?goal)
;   (machine (name ?goal) (mtype ?mtype))
;   ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?goal))
;   (not (want-to-produce-final-product))
;   =>
;   (retract ?sf ?l)
;   (assert (goto-has-locked ?goal))
;   (goto-machine ?goal ?mtype)
; )

; (defrule prod-execute-deliver
;   (phase PRODUCTION)
;   ?sf <- (state PROD_LOCK_REQUIRED_GOTO deliver)
;   ?l <- (lock (type ACCEPT) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource deliver))
;   =>
;   (retract ?sf ?l)
;   (assert (goto-has-locked deliver))
;   (goto-machine deliver DE)
; )

; (defrule prod-wait-for-deliver
;   (phase PRODUCTION)
;   (state PROD_LOCK_REQUIRED_GOTO deliver)
;   (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource deliver))
;   (deliver-wait-point ?wait-point)
;   =>
;   (printout t "Waiting for lock of DELIVER at " ?wait-point crlf)
;   (skill-call take_puck_to place (str-cat ?wait-point))
; )

; (defrule prod-reuse-lock
;   (phase PRODUCTION)
;   ?wgf <- (want-to-get-lock ?res)
;   ?wrf <- (want-to-release ?res $?)
;   =>
;   (printout t "Reusing lock of " ?res crlf)
;   (retract ?wgf ?wrf)
;   (assert (lock (type ACCEPT) (agent ?*ROBOT-NAME*) (resource ?res)))
; )

; (defrule prod-call-get-lock
;   (phase PRODUCTION)
;   ?wf <- (want-to-get-lock ?res)
;   (not (want-to-release ?res $?))
;   =>
;   (retract ?wf)
;   (assert (lock (type GET) (agent ?*ROBOT-NAME*) (resource ?res)))
; )
