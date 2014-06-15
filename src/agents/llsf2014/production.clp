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

;;;;;;;;;;;;;;;;;;;;;;;;;
;role change P3->nothing
;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule prod-role-P3-change-after-all-p3-orders
  "Drop the P3 role when there are no more P3 orders."
  (phase PRODUCTION)
  ?r <- (role P3-ONLY)
  (game-time $?time)
  (order (id ?id1) (product P3)
	 (end ?end1&:(> (nth$ 1 ?time) ?end1)))
  (order (id ?id2&:(neq ?id1 ?id2)) (product P3)
	 (end ?end2&:(> (nth$ 1 ?time) ?end2)))
  (order (id ?id3&:(and (neq ?id3 ?id2) (neq ?id3 ?id1))) (product P3)
	 (end ?end3&:(> (nth$ 1 ?time) ?end3)))  
  =>
  (printout warn "changing role from P3-ONLY to nothing because there are no more orders" crlf)
  (retract ?r)
  (assert (role nothing))
)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; decision rules for the next most important task to propose
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule prod-propose-task-idle
  "If we are idle change state to the proposed task."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (proposed-task (state proposed))
  =>
  (retract ?sf)
  (assert (state TASK-PROPOSED))
)

(defrule prod-change-to-more-important-task-when-waiting-for-lock
  "If we run a low-priority task and look for an alternative AND a task with a higher priority is proposed, drop the current work and change to the priorized task."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?t <- (task (state running) (priority ?old-p))
  ?pt <- (proposed-task (state proposed) (priority ?p&:(> ?p ?old-p)))
  ?sf1 <- (state WAIT_AND_LOOK_FOR_ALTERATIVE)
  ?sf2 <- (state WAIT-FOR-LOCK)
  ?lock-get <- (lock (type GET) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  ?lock-ref <- (lock (type REFUSE) (agent ?a&:(eq ?a ?*ROBOT-NAME*)) (resource ?res))
  ?wfl <- (wait-for-lock (res ?res) (state get))
  ?exec <- (execute-skill ? ?)
  =>
  (retract ?sf1 ?sf2 ?wfl ?lock-get ?exec ?lock-ref)
  (modify ?t (state finished))
  (assert
    (state TASK-PROPOSED)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res))
  )
)

(defrule prod-remove-proposed-tasks
  "Remove all proposed tasks, when you are neither idle nor looking for an alternative."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  (not (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE))
  ?pt <- (proposed-task (state proposed))
  =>
  (retract ?pt)
)

(defrule prod-deliver-P3
  "Role P3-ONLY: If we don't have anything to do, pick and deliver a P3."
  (declare (salience ?*PRIORITY-DELIVER-P3*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T5) (incoming $?i&~:(member$ PICK_PROD ?i)) (name ?name) (produced-puck P3) (team ?team-color))
  (game-time $?time)
  (order (product P3) (quantity-requested ?qr) (id  ?order-id)
	 ;(in-delivery ?in-delivery&:(< ?in-delivery ?qr))
	 (quantity-delivered ?qd&:(< ?qd ?qr))
	 (begin ?begin&:(<= ?begin (nth$ 1 ?time)))
	 (end ?end&:(<= (nth$ 1 ?time) ?end)))
  ;(not (proposed-task (name pick-and-deliver) (args $?args&:(subsetp ?args (create$ ?name P3 (+ ?in-delivery 1) ?order-id))) (state rejected)))
  (holding NONE)
  ;(not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-P3*))))
  (role P3-ONLY)
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-DELIVER-P3*))))
  =>
  (printout t "PROD: Deliver P3 from " ?name crlf)
  (assert (proposed-task (name pick-and-deliver) (priority ?*PRIORITY-DELIVER-P3*)
			 ;(args (create$ ?name P3 (+ ?in-delivery 1) ?order-id)))
			 (args (create$ ?name P3 1 ?order-id)))
  )
)

(defrule prod-deliver-P1-P2
  "Pick and deliver a P1 or P2."
  (declare (salience ?*PRIORITY-DELIVER-P1P2*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (incoming $?i&~:(member$ PICK_PROD ?i)) (name ?name) (produced-puck ?puck&P1|P2)  (team ?team-color))
  (game-time $?time)
  (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr))
	 (begin ?begin&:(<= ?begin (nth$ 1 ?time)))
	 (end ?end&:(<= (nth$ 1 ?time) ?end)))
  (not (proposed-task (name pick-and-deliver) (args $?args&:(subsetp ?args (create$ ?name ?puck  (+ ?in-delivery 1) ?order-id))) (state rejected)))
  (holding NONE)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-P1P2*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-DELIVER-P1P2*))))
  =>
  (printout t "PROD: Deliver " ?puck " from " ?name crlf)
  (assert (proposed-task (name pick-and-deliver) (priority ?*PRIORITY-DELIVER-P1P2*)
			 (args (create$ ?name ?puck  (+ ?in-delivery 1)  ?order-id)))
  )
)

(defrule prod-recycle
  "Recycle used pucks."
  (declare (salience ?*PRIORITY-RECYCLE*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (name ?name) (mtype T2|T3|T4) (junk ?j&:(> ?j 0))
	   (incoming $?i&~:(member$ PICK_CO ?i)) (produced-puck NONE) (team ?team-color))
  (machine (name ?recycle) (mtype RECYCLE) (team ?team-color))
  (not (proposed-task (name recycle) (args $?args&:(subsetp ?args (create$ ?name ?recycle))) (state rejected)))
  (holding NONE)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-RECYCLE*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-RECYCLE*))))
  =>
  (printout t "PROD: Recycling from " ?name crlf)
  (assert (proposed-task (name recycle) (args (create$ ?name ?recycle)) (priority ?*PRIORITY-RECYCLE*))
  )
)

(defrule prod-start-T2-with-S0
  "Starts production at a T2 with S0."
  (declare (salience ?*PRIORITY-START-T2-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (name ?name) (mtype T2) (loaded-with $?l&~:(member$ S0 ?l))
	   (incoming $?i&~:(member$ BRING_S0 ?i)&:(or (member$ S1 ?l) (member$ BRING_S1 ?i)))
	   (produced-puck NONE) (team ?team-color)
	   (incoming-agent $?ia&~:(member ?*ROBOT-NAME* ?ia)))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (holding NONE|S0)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-START-T2-WITH-S0*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-START-T2-WITH-S0*))))
  =>
  (printout t "PROD: Starting T2 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-START-T2-WITH-S0*))
  )
)

(defrule prod-load-T2-with-S1
  "Load T2 with S1."
  (declare (salience ?*PRIORITY-LOAD-T2-WITH-S1*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T2) (loaded-with $?l&~:(member$ S1 ?l))
	   (incoming $?i&~:(member$ BRING_S1 ?i)) (name ?name-T2) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T2))) (state rejected)))
  (holding NONE|S0|S1)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T2-WITH-S1*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T2-WITH-S1*))))
  =>
  (printout t "PROD: Loading T2 " ?name-T2 " with S1 after producing at a T1" crlf)
  (assert (proposed-task (name load-with-S1) (args (create$ ?name-T2)) (priority ?*PRIORITY-LOAD-T2-WITH-S1*))
  )
)

(defrule prod-prepare-T3_T4-with-S1
  "Load T3/T4 with S1 if a S2 is already in production."
  (declare (salience ?*PRIORITY-PREPARE-T3_T4-WITH-S1*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  ;T3/T4 to load
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S1 ?l)) 
	   (incoming $?i&~:(member$ BRING_S1 ?i)) (name ?name-T3_T4)
	   (produced-puck NONE) (team ?team-color) (priority ?prio))
  ;no T3/T4 with higher priority
  (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S1 ?l2)) (team ?team-color)
		(incoming $?i2&~:(member$ BRING_S1 ?i2)) (produced-puck NONE)
		(priority ?p2&:(> ?p2 ?prio))))
  ;there is a S2 in production
  (machine (mtype T2) (name ?name_T2) (produced-puck NONE) (team ?team-color)
	   (incoming $?i-t2) (loaded-with $?l-t2&:(and (or (member$ S1 ?l-t2) (member$ BRING_S1 ?i-t2))
						       (or (member$ S0 ?l-t2) (member$ BRING_S0 ?i-t2))))
	   (incoming-agent $?ia&~:(member ?*ROBOT-NAME* ?ia)))
  (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
  (holding NONE|S0|S1)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREPARE-T3_T4-WITH-S1*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-PREPARE-T3_T4-WITH-S1*))))
  =>
  (printout t "PROD: Preparing T3/T4 " ?name-T3_T4 " with S1" crlf)
  (assert (proposed-task (name load-with-S1) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-PREPARE-T3_T4-WITH-S1*))
  )
)

(defrule prod-prepare-T3_T4-with-S0
  "Load T3/T4 with S0 if a S2 is already in production."
  (declare (salience ?*PRIORITY-PREPARE-T3_T4-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  ;T3/T4 to load
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S0 ?l)) 
	   (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name-T3_T4)
	   (produced-puck NONE) (team ?team-color) (priority ?prio))
  ;no T3/T4 with higher priority
  (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S0 ?l2)) (team ?team-color)
		(incoming $?i2&~:(member$ BRING_S0 ?i2)) (produced-puck NONE)
		(priority ?p2&:(> ?p2 ?prio))))
  ;there is a S2 in production
  (machine (mtype T2) (name ?name_T2) (produced-puck NONE) (team ?team-color)
	   (incoming $?i-t2) (loaded-with $?l-t2&:(and (or (member$ S1 ?l-t2) (member$ BRING_S1 ?i-t2))
						       (or (member$ S0 ?l-t2) (member$ BRING_S0 ?i-t2))))
	   (incoming-agent $?ia&~:(member ?*ROBOT-NAME* ?ia)))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
  (holding NONE|S0)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREPARE-T3_T4-WITH-S0*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-PREPARE-T3_T4-WITH-S0*))))
  =>
  (printout t "PROD: Preparing T3/T4 " ?name-T3_T4 " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-PREPARE-T3_T4-WITH-S0*))
  )
)

(defrule prod-start-T3_T4-with-S0
  "Starts production at a T3/T4 with S0."
  (declare (salience ?*PRIORITY-START-T3_T4-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S0 ?l)) 
	   (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color))
  (machine (name ?name) (loaded-with $?l)
	   (incoming $?i&:(or (subsetp (create$ S1 S2) ?l)
			      (subsetp (create$ BRING_S1 BRING_S2) ?i)
			      (and (member$ S1 ?l) (member$ BRING_S2 ?i))
			      (and (member$ S2 ?l) (member$ BRING_S1 ?i))
			      ))
	   (team ?team-color))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (holding NONE|S0)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-START-T3_T4-WITH-S0*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-START-T3_T4-WITH-S0*))))
  =>
  (printout t "PROD: Loading T3/T4 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-START-T3_T4-WITH-S0*))
  )
)

(defrule prod-load-T3_T4-with-S0
  "Loads T3/T4 with an S0."
  (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S0 ?l)) 
	   (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (holding NONE|S0)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S0*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T3_T4-WITH-S0*))))
  =>
  (printout t "PROD: Loading T3/T4 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S0*))
  )
)

(defrule prod-load-T3_T4-with-S1
  "Loads T3/T4 with an S1."
  (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S1 ?l))
	   (incoming $?i&~:(member$ BRING_S1 ?i)) (name ?name-T3_T4) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
  (holding NONE|S0|S1)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
  =>
  (printout t "PROD: Loading T3/T4 " ?name-T3_T4 " with S1 after producing at T1" crlf)
  (assert (proposed-task (name load-with-S1) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
  )
)

(defrule prod-load-continue-T3_T4-with-S1
  "Continue to load T3/T4 with S1 after S1 was produced at T1."
  (declare (salience ?*PRIORITY-CONTINUE-T3_T4-WITH-S1*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S1 ?l))
	   (name ?name-T3_T4) (produced-puck NONE) (team ?team-color)
	   (incoming $?i&:(and (not (member$ BRING_S1 ?i))
			       (or (member$ S2 ?l) (member$ BRING_S1 ?i)))))
  (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
  (holding NONE|S0|S1)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
  =>
  (printout t "PROD: Continue Loading T3/T4 " ?name-T3_T4 " with S1 after producing at T1" crlf)
  (assert (proposed-task (name load-with-S1) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
  )
)

(defrule prod-load-T3_T4-with-S2
  "Load T3/T4 with S2. Prefer machines with higher priority."
  (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S2*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  ;T3/T4 to load
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S2 ?l)) 
	   (incoming $?i&~:(member$ BRING_S2 ?i)) (name ?name_T3T4)
	   (produced-puck NONE) (team ?team-color) (priority ?prio))
  ;no T3/T4 with higher priority
  (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S2 ?l2)) (team ?team-color)
		(incoming $?i2&~:(member$ BRING_S2 ?i2)) (produced-puck NONE)
		(priority ?p2&:(> ?p2 ?prio))))
   ;T2 to get the S2 from
  (machine (mtype T2) (produced-puck S2) 
	   (incoming $?i-T2&~:(member$ PICK_PROD ?i-T2)) (name ?name_T2) (team ?team-color))
  (not (proposed-task (name pick-and-load) (args $?args&:(subsetp ?args (create$ ?name_T2 ?name_T3T4))) (state rejected)))
  (holding NONE)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S2*))))
  (not (role P3-ONLY))
  (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T3_T4-WITH-S2*))))
  =>
  (printout t "PROD: Loading T3/T4 " ?name_T3T4 " with S2 from " ?name_T2 crlf)
  (assert (proposed-task (name pick-and-load) (args (create$ ?name_T2 ?name_T3T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S2*))
  )
)

(defrule prod-load-T5-with-S0
  "Load T5 with S0 if you are in P3-ONLY role."
  (declare (salience ?*PRIORITY-LOAD-T5-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE);|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T5) (loaded-with $?l&~:(member$ S0 ?l))
    (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color))
  ;(not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (holding NONE|S0)
  ;(not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T5-WITH-S0*))))
  (role P3-ONLY)
  =>
  (printout t "PROD: Loading T5 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-LOAD-T5-WITH-S0*))
  )
)

; (defrule prod-wait-in-front-of-T5-with-S0
;   (declare (salience ?*PRIORITY-LOAD-T5-WITH-S0*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T5) (name ?name) (team ?team-color))
;   (holding NONE)
;   (role P3-ONLY)
;   =>
;   (printout t "PROD: Loading T5 " ?name " with S0" crlf)
;   (assert (proposed-task (name wait-at-T5) (args (create$ ?name)) (priority ?*PRIORITY-LOAD-T5-WITH-S0*))
;   )
; )

(defrule prod-deliver-unintentionally-holding-produced-puck
  "If holding a puck that should not be held (e.g. picked it up while driving) deliver it."
  (declare (salience ?*PRIORITY-DELIVER-HOLDING*))
  (phase PRODUCTION)
  (state IDLE)
  (holding ?puck&P1|P2|P3)
  (team-color ?team-color&~nil)
  ; (game-time $?time)
  ; (order (product P3) (quantity-requested ?qr) (id  ?order-id)
  ; 	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr))
  ; 	 (begin ?begin&:(<= ?begin (nth$ 1 ?time)))
  ; 	 (end ?end&:(<= (nth$ 1 ?time) ?end)))
  (not (proposed-task (name deliver) (args $?args&:(subsetp ?args (create$ ?puck))) (state rejected)))
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-HOLDING*))))
  =>
  (printout error "PROD: Deliver unintentionally holding produced puck" crlf)
  (assert (proposed-task (name deliver) (priority ?*PRIORITY-DELIVER-HOLDING*)
			 (args (create$ ?puck)))
  )
)

(defrule prod-load-unintentionally-holding-S2
  "If we hold an S2 unintentionally load the next free T3/T4 with the highest priority that doesn't hold an S2 yet with it."
  (declare (salience ?*PRIORITY-LOAD-HOLDING-S2*))
  (phase PRODUCTION)
  (state IDLE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S2 ?l)) 
	   (incoming $?i&~:(member$ BRING_S2 ?i)) (name ?name_T3T4)
	   (produced-puck NONE) (team ?team-color) (priority ?prio))
  (not (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S2 ?l)) (team ?team-color)
		(incoming $?i&~:(member$ BRING_S2 ?i)) (produced-puck NONE)
		(priority ?p2&:(> ?p2 ?prio))))
  (not (proposed-task (name load-with-S2) (args $?args&:(subsetp ?args (create$ ?name_T3T4))) (state rejected)))
  (holding S2)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-HOLDING-S2*))))
  (not (role P3-ONLY))
  =>
  (printout t "PROD: Loading T3/T4 " ?name_T3T4 " with S2" crlf)
  (assert (proposed-task (name load-with-S2) (args (create$ ?name_T3T4)) (priority ?*PRIORITY-LOAD-HOLDING-S2*))
  )
)

(defrule prod-recycle-unintentionally-holding-CO
  "If we hold a C0 unintentionally, recycle it."
  (declare (salience ?*PRIORITY-RECYCLE*))
  (phase PRODUCTION)
  (state IDLE)
  (holding CO)
  (team-color ?team-color&~nil)
  (machine (name ?name) (mtype RECYCLE) (team ?team-color))
  (not (proposed-task (name recycle-holding) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-RECYCLE*))))
  (not (role P3-ONLY))
  =>
  (printout t "PROD: Recycle hilding CO at " ?name crlf)
  (assert (proposed-task (name recycle-holding) (args (create$ ?name)) (priority ?*PRIORITY-RECYCLE*))
  )
)
