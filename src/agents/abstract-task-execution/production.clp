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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; decision rules for the next most important task to propose
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrule prod-propose-task-idle
  "If we are idle change state to the proposed task."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (task (state proposed))
  =>
  (retract ?sf)
  (assert (state TASK-PROPOSED))
)

(defrule prod-change-to-more-important-task-when-waiting-for-lock
  "If we run a low-priority task and look for an alternative AND a task with a higher priority is proposed, drop the current work and change to the priorized task."
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?t <- (task (state running) (priority ?old-p))
  ?pt <- (task (state proposed) (priority ?p&:(> ?p ?old-p)))
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
  ?pt <- (task (state proposed))
  =>
  (retract ?pt)
)


(defrule prod-produce-P3-and-deliver
  "Complete production of a P3 at the beginning without leaving the T5 machine while producing."
  (declare (salience ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T5) (loaded-with $?l&~:(member$ S0 ?l))
    (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color)
    (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0))
  )
  (not (and (task (name produce-with-S0) (state rejected) (id ?rej-id))
	    (step (name produce-at) (id ?rej-st&:(eq ?rej-st (+ ?rej-id 2))) (machine ?name))))
  (holding NONE|S0)
  (not (task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))))
  (not (no-more-needed P3))
  =>
  (printout t "PROD: PRODUCE P3 at T5 " ?name " waiting and deliver afterweards" crlf)
  ;generate random task id
  (bind ?task-id (random 0 1000000000))
  (assert (task (name produce-p3-and-deliver) (id ?task-id) (state proposed)
		(steps (create$ (+ ?task-id 1) (+ ?task-id 2) (+ ?task-id 3)))
		(current-step (+ ?task-id 1)) (priority ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))
	  (step (name get-s0) (id (+ ?task-id 1))
		(task-priority ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))
	  (step (name produce-at) (id (+ ?task-id 2)) (machine ?name))
	  (step (name deliver) (id (+ ?task-id 3)) (product-type P3))
	  (needed-task-lock (task-id ?task-id) (action BRING_S0) (place ?name))
  )
)


; (defrule prod-deliver-P3
;   "If we don't have anything to do, pick and deliver a P3."
;   (declare (salience ?*PRIORITY-DELIVER-P3*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T5) (incoming $?i&~:(member$ PICK_PROD ?i)) (name ?name) (produced-puck P3) (team ?team-color))
;   (game-time $?time)
;   (order (product P3) (quantity-requested ?qr) (id  ?order-id)
; 	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr))
; 	 (quantity-delivered ?qd&:(< ?qd ?qr)) (begin ?begin)
; 	 (end ?end&:(tac-can-use-timeslot (nth$ 1 ?time) ?begin ?end (+ ?*SKILL-DURATION-GET-PRODUCED* ?*SKILL-DURATION-DELIVER*))))
;   (not (proposed-task (name pick-and-deliver) (args $?args&:(subsetp ?args (create$ ?name P3 (+ ?in-delivery 1) ?order-id))) (state rejected)))
;   (holding NONE)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-P3*))))
;   (not (locked-resource (agent ?a&:(neq (sym-cat ?a) (sym-cat ?*ROBOT-NAME*)))
; 			(resource P3-ONLY))) ;only if there is no other P3 agent
;   (not (no-more-needed P3))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-DELIVER-P3*))))
;   =>
;   (printout t "PROD: Deliver P3 from " ?name crlf)
;   (assert (proposed-task (name pick-and-deliver) (priority ?*PRIORITY-DELIVER-P3*)
; 			 (args (create$ ?name P3 (+ ?in-delivery 1) ?order-id)))
;   )
; )

; (defrule prod-deliver-P1-P2
;   "Pick and deliver a P1 or P2."
;   (declare (salience ?*PRIORITY-DELIVER-P1P2*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T3|T4) (incoming $?i&~:(member$ PICK_PROD ?i)) (name ?name) (produced-puck ?puck&P1|P2)  (team ?team-color))
;   (game-time $?time)
;   (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
; 	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr)) (begin ?begin)
; 	 (end ?end&:(tac-can-use-timeslot (nth$ 1 ?time) ?begin ?end (+ ?*SKILL-DURATION-GET-PRODUCED* ?*SKILL-DURATION-DELIVER*))))
;   (not (proposed-task (name pick-and-deliver) (args $?args&:(subsetp ?args (create$ ?name ?puck  (+ ?in-delivery 1) ?order-id))) (state rejected)))
;   (holding NONE)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-P1P2*))))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-DELIVER-P1P2*))))
;   =>
;   (printout t "PROD: Deliver " ?puck " from " ?name crlf)
;   (assert (proposed-task (name pick-and-deliver) (priority ?*PRIORITY-DELIVER-P1P2*)
; 			 (args (create$ ?name ?puck  (+ ?in-delivery 1)  ?order-id)))
;   )
; )

; (defrule prod-recycle
;   "Recycle used pucks."
;   (declare (salience ?*PRIORITY-RECYCLE*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (name ?name) (mtype T2|T3|T4) (junk ?j&:(> ?j 0))
;      (incoming $?i&~:(member$ PICK_CO ?i)) (produced-puck NONE) (team ?team-color))
;   (machine (name ?recycle) (mtype RECYCLE) (team ?team-color)
;      (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   (not (proposed-task (name recycle) (args $?args&:(subsetp ?args (create$ ?name ?recycle))) (state rejected)))
;   (holding NONE)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-RECYCLE*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-RECYCLE*))))
;   =>
;   (printout t "PROD: Recycling from " ?name crlf)
;   (assert (proposed-task (name recycle) (args (create$ ?name ?recycle)) (priority ?*PRIORITY-RECYCLE*))
;   )
; )

; (defrule prod-start-T2-with-S0
;   "Starts production at a T2 with S0."
;   (declare (salience ?*PRIORITY-START-T2-WITH-S0*))
;   (phase PRODUCTION)
;   (state ?state&IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (name ?name) (mtype T2) (loaded-with $?l&~:(member$ S0 ?l))
; 	   (incoming $?i&~:(member$ BRING_S0 ?i)&:(or (member$ S1 ?l) (member$ BRING_S1 ?i)))
; 	   (produced-puck NONE) (team ?team-color) (junk 0)
; 	   (incoming-agent $?ia&~:(member (sym-cat ?*ROBOT-NAME*) ?ia))
; 	   (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
;   (holding NONE|S0)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-START-T2-WITH-S0*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-START-T2-WITH-S0*))))
;   =>
;   (printout t "PROD: Starting T2 " ?name " with S0" crlf)
;   (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-START-T2-WITH-S0*))
;   )
; )

; (defrule prod-load-T2-with-S1
;   "Load T2 with S1."
;   (declare (salience ?*PRIORITY-LOAD-T2-WITH-S1*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T2) (loaded-with $?l&~:(member$ S1 ?l))
; 	   (incoming $?i&~:(member$ BRING_S1 ?i)) (name ?name-T2)
; 	   (produced-puck NONE) (team ?team-color) (junk 0) (priority ?prio)
; 	   (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   ;no T2 with higher priority
;   (not (machine (mtype T2) (loaded-with $?l2&~:(member$ S1 ?l2)) (team ?team-color)
; 		(incoming $?i2&~:(member$ BRING_S1 ?i2)) (produced-puck NONE) (junk 0)
; 		(priority ?p2&:(> ?p2 ?prio)) (out-of-order-until $?ooo2&:(eq (nth$ 1 ?ooo2) 0))))
;   (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T2))) (state rejected)))
;   (holding NONE|S0|S1)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T2-WITH-S1*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T2-WITH-S1*))))
;   =>
;   (printout t "PROD: Loading T2 " ?name-T2 " with S1 after producing at a T1" crlf)
;   (assert (proposed-task (name load-with-S1) (args (create$ ?name-T2)) (priority ?*PRIORITY-LOAD-T2-WITH-S1*))
;   )
; )

; (defrule prod-prepare-T3_T4-with-S1
;   "Load T3/T4 with S1 if a S2 is already in production and no other T3_T4 is being prepared now."
;   (declare (salience ?*PRIORITY-PREPARE-T3_T4-WITH-S1*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   ;T3/T4 to load
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S1 ?l)) 
; 	   (incoming $?i&~:(member$ BRING_S1 ?i)) (name ?name-T3_T4)
; 	   (produced-puck NONE) (team ?team-color) (junk 0) (priority ?prio)
; 	   (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   ;no T3/T4 with higher priority
;   (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S1 ?l2)) (team ?team-color)
; 		(incoming $?i2&~:(member$ BRING_S1 ?i2)) (produced-puck NONE) (junk 0)
; 		(priority ?p2&:(> ?p2 ?prio))(out-of-order-until $?ooo2&:(eq (nth$ 1 ?ooo2) 0))))
;   ;no other T3/T4 is being prepared now
;   (not (machine (mtype T3|T4) (loaded-with $?l3&~:(member$ S2 ?l3)) (team ?team-color)
; 		(incoming $?i3&:(or (member$ BRING_S1 ?i3) (member$ S1 ?l3))) (junk 0)
; 		(produced-puck NONE) (final-prod-time $?fpt&:(eq 0 (nth$ 1 ?fpt)))
; 		(out-of-order-until $?ooo3&:(eq (nth$ 1 ?ooo3) 0))))
;   ;there is a S2 in production
;   (machine (mtype T2) (name ?name_T2) (produced-puck NONE) (team ?team-color)
; 	   (incoming $?i-t2) (loaded-with $?l-t2&:(and (or (member$ S1 ?l-t2) (member$ BRING_S1 ?i-t2))
; 						       (or (member$ S0 ?l-t2) (member$ BRING_S0 ?i-t2))))
; 	   (incoming-agent $?ia&~:(member (sym-cat ?*ROBOT-NAME*) ?ia)))
;   (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
;   (holding NONE|S0|S1)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREPARE-T3_T4-WITH-S1*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-PREPARE-T3_T4-WITH-S1*))))
;   =>
;   (printout t "PROD: Preparing T3/T4 " ?name-T3_T4 " with S1" crlf)
;   (assert (proposed-task (name load-with-S1) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-PREPARE-T3_T4-WITH-S1*))
;   )
; )

; (defrule prod-prepare-T3_T4-with-S0
;   "Load T3/T4 with S0 if a S2 is already in production and a S1 is already on the way."
;   (declare (salience ?*PRIORITY-PREPARE-T3_T4-WITH-S0*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   ;T3/T4 to load
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S0 ?l)) 
; 	   (incoming $?i&:(and (not (member$ BRING_S0 ?i))
; 			       (or (member$ S1 ?l) (member$ BRING_S1 ?i))))
; 	   (name ?name-T3_T4) (produced-puck NONE)
; 	   (team ?team-color) (junk 0) (priority ?prio)
; 	   (incoming-agent $?ia&~:(member (sym-cat ?*ROBOT-NAME*) ?ia))
; 	   (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   ;no T3/T4 with higher priority
;   (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S0 ?l2)) (team ?team-color)
; 		(incoming $?i2&~:(member$ BRING_S0 ?i2)) (produced-puck NONE) (junk 0)
; 		(priority ?p2&:(> ?p2 ?prio)) (out-of-order-until $?ooo2&:(eq (nth$ 1 ?ooo2) 0))))
;   ;there is a S2 in production
;   (machine (mtype T2) (name ?name_T2) (produced-puck NONE) (team ?team-color)
; 	   (incoming $?i-t2) (loaded-with $?l-t2&:(and (or (member$ S1 ?l-t2) (member$ BRING_S1 ?i-t2))
; 						       (or (member$ S0 ?l-t2) (member$ BRING_S0 ?i-t2))))
; 	   (incoming-agent $?ia-t2&~:(member (sym-cat ?*ROBOT-NAME*) ?ia-t2)))
;   (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
;   (holding NONE|S0)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PREPARE-T3_T4-WITH-S0*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-PREPARE-T3_T4-WITH-S0*))))
;   =>
;   (printout t "PROD: Preparing T3/T4 " ?name-T3_T4 " with S0" crlf)
;   (assert (proposed-task (name load-with-S0) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-PREPARE-T3_T4-WITH-S0*))
;   )
; )

; (defrule prod-start-T3_T4-with-S0
;   "Starts production at a T3/T4 with S0."
;   (declare (salience ?*PRIORITY-START-T3_T4-WITH-S0*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S0 ?l)) (junk 0)
; 	   (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color))
;   (machine (name ?name) (loaded-with $?l)
; 	   (incoming $?i&:(or (subsetp (create$ S1 S2) ?l)
; 			      (subsetp (create$ BRING_S1 BRING_S2) ?i)
; 			      (and (member$ S1 ?l) (member$ BRING_S2 ?i))
; 			      (and (member$ S2 ?l) (member$ BRING_S1 ?i))
; 			      ))
; 	   (team ?team-color) (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
;   (holding NONE|S0)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-START-T3_T4-WITH-S0*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-START-T3_T4-WITH-S0*))))
;   =>
;   (printout t "PROD: Loading T3/T4 " ?name " with S0" crlf)
;   (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-START-T3_T4-WITH-S0*))
;   )
; )

; (defrule prod-load-T3_T4-with-S0
;   "Loads T3/T4 with an S0."
;   (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S0*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S0 ?l)) (junk 0)
; 	   (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color)
; 	   (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)) (priority ?prio))
;   (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S0 ?l2)) (junk 0)
; 	   (incoming $?i2&~:(member$ BRING_S0 ?i2)) (produced-puck NONE) (team ?team-color)
; 	   (out-of-order-until $?ooo2&:(eq (nth$ 1 ?ooo2) 0))
;      (priority ?p2&:(> ?p2 ?prio))))
;   (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
;   (holding NONE|S0)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S0*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T3_T4-WITH-S0*))))
;   =>
;   (printout t "PROD: Loading T3/T4 " ?name " with S0" crlf)
;   (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S0*))
;   )
; )

; (defrule prod-load-T3_T4-with-S1
;   "Loads T3/T4 with an S1."
;   (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S1 ?l)) (junk 0) (priority ?prio)
; 	   (incoming $?i&~:(member$ BRING_S1 ?i)) (name ?name-T3_T4) (produced-puck NONE) (team ?team-color)
; 	   (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   ;no T3/T4 with higher priority
;   (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S1 ?l2)) (team ?team-color)
; 		(incoming $?i2&~:(member$ BRING_S1 ?i2)) (produced-puck NONE) (junk 0)
; 		(priority ?p2&:(> ?p2 ?prio)) (out-of-order-until $?ooo2&:(eq (nth$ 1 ?ooo2) 0))))
;   (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
;   (holding NONE|S0|S1)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
;   =>
;   (printout t "PROD: Loading T3/T4 " ?name-T3_T4 " with S1 after producing at T1" crlf)
;   (assert (proposed-task (name load-with-S1) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
;   )
; )

; (defrule prod-load-continue-T3_T4-with-S1
;   "Continue to load T3/T4 with S1 after S1 was produced at T1."
;   (declare (salience ?*PRIORITY-CONTINUE-T3_T4-WITH-S1*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S1 ?l)) (junk 0)
; 	   (name ?name-T3_T4) (produced-puck NONE) (team ?team-color)
; 	   (incoming $?i&:(and (not (member$ BRING_S1 ?i))
; 			       (or (member$ S2 ?l) (member$ BRING_S2 ?i))))
; 	   (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
;   (holding NONE|S0|S1)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
;   =>
;   (printout t "PROD: Continue Loading T3/T4 " ?name-T3_T4 " with S1 after producing at T1" crlf)
;   (assert (proposed-task (name load-with-S1) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
;   )
; )

; (defrule prod-load-T3_T4-with-S2
;   "Load T3/T4 with S2. Prefer machines with higher priority."
;   (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S2*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   ;T3/T4 to load
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S2 ?l)) (junk 0)
; 	   (incoming $?i&~:(member$ BRING_S2 ?i)) (name ?name_T3T4)
; 	   (produced-puck NONE) (team ?team-color) (priority ?prio)
; 	   (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)))
;   ;no T3/T4 with higher priority
;   (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S2 ?l2)) (team ?team-color)
; 		(incoming $?i2&~:(member$ BRING_S2 ?i2)) (produced-puck NONE)
; 		(priority ?p2&:(> ?p2 ?prio)) (junk 0)
; 		(out-of-order-until $?ooo2&:(eq (nth$ 1 ?ooo2) 0))))
;    ;T2 to get the S2 from
;   (machine (mtype T2) (produced-puck S2) 
; 	   (incoming $?i-T2&~:(member$ PICK_PROD ?i-T2)) (name ?name_T2) (team ?team-color))
;   (not (proposed-task (name pick-and-load) (args $?args&:(subsetp ?args (create$ ?name_T2 ?name_T3T4))) (state rejected)))
;   (holding NONE)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S2*))))
;   (not (role P3-ONLY))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-LOAD-T3_T4-WITH-S2*))))
;   =>
;   (printout t "PROD: Loading T3/T4 " ?name_T3T4 " with S2 from " ?name_T2 crlf)
;   (assert (proposed-task (name pick-and-load) (args (create$ ?name_T2 ?name_T3T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S2*))
;   )
; )

; (defrule prod-load-T5-with-S0
;   "Load T5 with S0 if you are in P3-ONLY role."
;   (declare (salience ?*PRIORITY-LOAD-T5-WITH-S0*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T5) (loaded-with $?l&~:(member$ S0 ?l))
;     (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color)
;     (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0))
;   )
;   (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
;   (holding NONE|S0)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T5-WITH-S0*))))
;   (not (locked-resource (resource P3-ONLY))) ;load only if there is no P3 agent
;   (not (no-more-needed P3))
;   =>
;   (printout t "PROD: Loading T5 " ?name " with S0" crlf)
;   (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-LOAD-T5-WITH-S0*))
;   )
; )

; (defrule prod-produce-P3-at-begin
;   "Fast production of a P3 at the beginning without leaving the T5 machine while producing."
;   (declare (salience ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T5) (loaded-with $?l&~:(member$ S0 ?l))
;     (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color)
;     (out-of-order-until $?ooo&:(eq (nth$ 1 ?ooo) 0)) (x ?t5-x) (y ?t5-y)
;   )
;   (not (proposed-task (name produce-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
;   (holding NONE|S0)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))))
;   (not (locked-resource (agent ?a&:(neq (sym-cat ?a) (sym-cat ?*ROBOT-NAME*)))
; 			(resource P3-ONLY))) ;only if there is no other P3 agent
;   (not (no-more-needed P3))
;   (or (game-time $?time&:(< (nth$ 1 ?time) ?*TIME-P3-PRODUCTION-WITHOUT-LEAVING*))
;       (deliver ?team-color ? ?del-x ?del-y&:(> (distance ?del-x ?del-y ?t5-x ?t5-y) ?*MAX-T5-LEAVE-DISTANCE*)))
;   =>
;   (printout t "PROD: PRODUCE P3 at T5 " ?name " without waiting" crlf)
;   (assert (proposed-task (name produce-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-PRODUCE-T5-AT-BEGIN*))
;   )
; )

; (defrule prod-just-in-time-P3-production
;   "Only for P3-Only agent. Drive to T5 start producing when an order is there and deliver it."
;   (declare (salience ?*PRIORITY-JUST-IN-TIME-P3*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T5) (loaded-with $?l&~:(member$ S0 ?l))
;     (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name)
;     (produced-puck NONE) (team ?team-color)
;   )
;   (holding NONE|S0)
;   (role P3-ONLY)
;   (not (no-more-needed P3))
;   =>
;   (printout t "PROD: Doing just-in-time production at T5 " ?name crlf)
;   (assert (proposed-task (name just-in-time-P3) (args (create$ ?name)) (priority ?*PRIORITY-JUST-IN-TIME-P3*))
;   )
; )

; (defrule prod-deliver-holding-produced-puck
;   "If we hold a puck and there is an order for it, deliver it."
;   (declare (salience ?*PRIORITY-DELIVER-HOLDING*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (holding ?puck&P1|P2|P3)
;   (team-color ?team-color&~nil)
;   (game-time $?time)
;   (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
;   	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr)) (begin ?begin)
; 	 (end ?end&:(tac-can-use-timeslot (nth$ 1 ?time) ?begin ?end ?*SKILL-DURATION-DELIVER*)))
;   (not (proposed-task (name deliver) (args $?args&:(subsetp ?args (create$ holding ?puck (+ ?in-delivery 1) ?order-id))) (state rejected)))
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-HOLDING*))))
;   =>
;   (printout t "PROD: Deliver holding produced puck" crlf)
;   (assert (proposed-task (name deliver) (priority ?*PRIORITY-DELIVER-HOLDING*)
; 			 (args (create$ holding ?puck (+ ?in-delivery 1)  ?order-id)))
;   )
; )

; (defrule prod-store-holding-produced-puck
;   "If we hold a puck and there is no order for it, store it."
;   (declare (salience ?*PRIORITY-DELIVER-HOLDING*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (holding ?puck&P1|P2|P3)
;   (team-color ?team-color&~nil)
;   (game-time $?time)
;   (not (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
; 	      (in-delivery ?in-delivery&:(< ?in-delivery ?qr)) (begin ?begin)
; 	      (end ?end&:(tac-can-use-timeslot (nth$ 1 ?time) ?begin ?end ?*SKILL-DURATION-DELIVER*))))
;   (puck-storage (name ?storage) (puck NONE) (team ?team-color) (incoming $?i-st&~:(member$ STORE_PUCK ?i-st)))
;   (not (proposed-task (name store) (args $?args&:(subsetp ?args (create$ ?storage ?puck))) (state rejected)))
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-HOLDING*))))
;   =>
;   (printout t "PROD: Store holding produced puck" crlf)
;   (assert (proposed-task (name store) (priority ?*PRIORITY-DELIVER-HOLDING*)
; 			 (args (create$ ?storage ?puck)))
;   )
; )

; (defrule prod-load-unintentionally-holding-S2
;   "If we hold an S2 unintentionally load the next free T3/T4 with the highest priority that doesn't hold an S2 yet with it."
;   (declare (salience ?*PRIORITY-LOAD-HOLDING-S2*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S2 ?l)) (junk 0)
; 	   (incoming $?i&~:(member$ BRING_S2 ?i)) (name ?name_T3T4)
; 	   (produced-puck NONE) (team ?team-color) (priority ?prio))
;   (not (machine (mtype T3|T4) (loaded-with $?l2&~:(member$ S2 ?l2)) (team ?team-color)
; 		(incoming $?i2&~:(member$ BRING_S2 ?i2)) (produced-puck NONE) (junk 0)
; 		(priority ?p2&:(> ?p2 ?prio))))
;   (not (proposed-task (name load-with-S2) (args $?args&:(subsetp ?args (create$ ?name_T3T4))) (state rejected)))
;   (holding S2)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-HOLDING-S2*))))
;   (not (role P3-ONLY))
;   =>
;   (printout t "PROD: Loading T3/T4 " ?name_T3T4 " with S2" crlf)
;   (assert (proposed-task (name load-with-S2) (args (create$ ?name_T3T4)) (priority ?*PRIORITY-LOAD-HOLDING-S2*))
;   )
; )

; (defrule prod-recycle-unintentionally-holding-CO
;   "If we hold a C0 unintentionally, recycle it."
;   (declare (salience ?*PRIORITY-RECYCLE*))
;   (phase PRODUCTION)
;   (state IDLE)
;   (holding CO)
;   (team-color ?team-color&~nil)
;   (machine (name ?name) (mtype RECYCLE) (team ?team-color))
;   (not (proposed-task (name recycle-holding) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-RECYCLE*))))
;   (not (role P3-ONLY))
;   =>
;   (printout t "PROD: Recycle hilding CO at " ?name crlf)
;   (assert (proposed-task (name recycle-holding) (args (create$ ?name)) (priority ?*PRIORITY-RECYCLE*))
;   )
; )

; (defrule prod-store-product
;   "If there is a produced puck, we want to store it next to the delivery-gates to fulfill orders quicker and clear the machine."
;   (declare (salience ?*PRIORITY-STORE-PRODUCED*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (machine (mtype T3|T4|T5) (incoming $?i&~:(member$ PICK_PROD ?i)) (name ?name) (produced-puck ?puck&P1|P2|P3) (team ?team-color))
;   (puck-storage (name ?storage) (puck NONE) (team ?team-color)
; 		(incoming $?i-st&~:(member$ STORE_PUCK ?i-st)))
;   ;do not store a second P3
;   (not (puck-storage (puck P3&:(eq ?puck P3)) (team ?team-color)))
;   (not (proposed-task (name pick-and-store) (args $?args&:(subsetp ?args (create$ ?name ?puck ?storage))) (state rejected)))
;   (holding NONE)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-STORE-PRODUCED*))))
;   (not (no-more-needed ?puck&:(eq ?puck P3))) ;store everything but unneded P3s
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-STORE-PRODUCED*))))
;   =>
;   (printout t "PROD: Store " ?puck " from " ?name " at " ?storage crlf)
;   (assert (proposed-task (name pick-and-store) (priority ?*PRIORITY-STORE-PRODUCED*)
; 			 (args (create$ ?name ?puck ?storage)))
;   )
; )

; (defrule prod-deliver-stored-product
;   "Deliver a product we stored before."
;   (declare (salience ?*PRIORITY-DELIVER-STORED*))
;   (phase PRODUCTION)
;   (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
;   (team-color ?team-color&~nil)
;   (puck-storage (name ?storage) (puck ?puck) (team ?team-color)
; 		(incoming $?i-st&~:(member$ GET_STORED_PUCK ?i-st)))
;   (game-time $?time)
;   (order (product ?puck) (quantity-requested ?qr) (id  ?order-id)
; 	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr)) (begin ?begin)
; 	 (end ?end&:(tac-can-use-timeslot (nth$ 1 ?time) ?begin ?end (+ ?*SKILL-DURATION-GET-STORED-PUCK* ?*SKILL-DURATION-DELIVER*))))
;   (not (proposed-task (name get-stored-and-deliver) (args $?args&:(subsetp ?args (create$ ?storage ?puck (+ ?in-delivery 1) ?order-id))) (state rejected)))
;   (holding NONE)
;   (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-STORED*))))
;   (not (task (state running) (priority ?old-p&:(>= ?old-p ?*PRIORITY-DELIVER-STORED*))))
;   =>
;   (printout t "PROD: Deliver stored " ?puck " from " ?storage crlf)
;   (assert (proposed-task (name get-stored-and-deliver)
; 			 (priority ?*PRIORITY-DELIVER-STORED*)
; 			 (args (create$ ?storage ?puck (+ ?in-delivery 1) ?order-id)))
;   )
; )
