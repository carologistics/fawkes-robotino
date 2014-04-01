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

(defrule prod-propose-task-idle
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  ?sf <- (state IDLE)
  (proposed-task (state proposed))
  =>
  (retract ?sf)
  (assert (state TASK-PROPOSED))
)

(defrule prod-change-to-more-important-task-when-waiting-for-lock
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
  (assert (state TASK-PROPOSED)
	  (lock (type RELEASE) (agent ?*ROBOT-NAME*) (resource ?res)))
)

(defrule prod-remove-proposed-tasks
  (declare (salience ?*PRIORITY-LOW*))
  (phase PRODUCTION)
  (not (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE))
  ?pt <- (proposed-task (state proposed))
  =>
  (retract ?pt)
)

(defrule prod-deliver-P3
  (declare (salience ?*PRIORITY-DELIVER-P3*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T5) (incoming $?i&~:(member$ PICK_PROD ?i)) (name ?name) (produced-puck P3) (team ?team-color))
  (game-time $?time)
  (order (product P3) (quantity-requested ?qr) (id  ?order-id)
	 (in-delivery ?in-delivery&:(< ?in-delivery ?qr))
	 (begin ?begin&:(<= ?begin (nth$ 1 ?time)))
	 (end ?end&:(<= (nth$ 1 ?time) ?end)))
  (not (proposed-task (name pick-and-deliver) (args $?args&:(subsetp ?args (create$ ?name P3 (+ ?in-delivery 1) ?order-id))) (state rejected)))
  (holding NONE)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-DELIVER-P3*))))
  =>
  (printout t "PROD: Deliver P3 from " ?name crlf)
  (assert (proposed-task (name pick-and-deliver) (priority ?*PRIORITY-DELIVER-P3*)
			 (args (create$ ?name P3 (+ ?in-delivery 1) ?order-id)))
  )
)

(defrule prod-deliver-P1-P2
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
  =>
  (printout t "PROD: Deliver " ?puck " from " ?name crlf)
  (assert (proposed-task (name pick-and-deliver) (priority ?*PRIORITY-DELIVER-P1P2*)
			 (args (create$ ?name ?puck  (+ ?in-delivery 1)  ?order-id)))
  )
)

(defrule prod-recycle
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
  =>
  (printout t "PROD: Recycling from " ?name crlf)
  (assert (proposed-task (name recycle) (args (create$ ?name ?recycle)) (priority ?*PRIORITY-RECYCLE*))
  )
)

(defrule prod-start-T2-with-S0
  (declare (salience ?*PRIORITY-START-T2-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T2) (loaded-with $?l&~:(member$ S0 ?l))
	   (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color))
  (machine (name ?name) (loaded-with $?l) (incoming $?i&:(or (member$ S1 ?l) (member$ BRING_S1 ?i))) (team ?team-color))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (holding NONE|S0)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-START-T2-WITH-S0*))))
  =>
  (printout t "PROD: Starting T2 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-START-T2-WITH-S0*))
  )
)

(defrule prod-load-T2-with-S0
  (declare (salience ?*PRIORITY-LOAD-T2-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T2) (loaded-with $?l&~:(member$ S0 ?l))
	   (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (holding NONE|S0)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T2-WITH-S0*))))
  =>
  (printout t "PROD: Loading T2 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-LOAD-T2-WITH-S0*))
  )
)

(defrule prod-load-T2-with-S1
  (declare (salience ?*PRIORITY-LOAD-T2-WITH-S1*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T2) (loaded-with $?l&~:(member$ S1 ?l))
	   (incoming $?i&~:(member$ BRING_S1 ?i)) (name ?name-T2) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T2))) (state rejected)))
  (holding NONE|S0|S1)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T2-WITH-S1*))))
  =>
  (printout t "PROD: Loading T2 " ?name-T2 " with S1 after producing at a T1" crlf)
  (assert (proposed-task (name load-with-S1) (args (create$ ?name-T2)) (priority ?*PRIORITY-LOAD-T2-WITH-S1*))
  )
)

(defrule prod-start-T3_T4-with-S0
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
  =>
  (printout t "PROD: Loading T3/T4 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-START-T3_T4-WITH-S0*))
  )
)

(defrule prod-load-T3_T4-with-S0
  (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S0 ?l)) 
	   (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (holding NONE|S0)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S0*))))
  =>
  (printout t "PROD: Loading T3/T4 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S0*))
  )
)

(defrule prod-load-T3_T4-with-S1
  (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S1 ?l))
	   (incoming $?i&~:(member$ BRING_S1 ?i)) (name ?name-T3_T4) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S1) (args $?args&:(subsetp ?args (create$ ?name-T3_T4))) (state rejected)))
  (holding NONE|S0|S1)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S1*))))
  =>
  (printout t "PROD: Loading T3/T4 " ?name-T3_T4 " with S1 after producing at T1" crlf)
  (assert (proposed-task (name load-with-S1) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
  )
)

(defrule prod-load-continue-T3_T4-with-S1
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
  =>
  (printout t "PROD: Continue Loading T3/T4 " ?name-T3_T4 " with S1 after producing at T1" crlf)
  (assert (proposed-task (name load-with-S1) (args (create$ ?name-T3_T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S1*))
  )
)

(defrule prod-load-T3_T4-with-S2
  (declare (salience ?*PRIORITY-LOAD-T3_T4-WITH-S2*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S2 ?l)) 
	   (incoming $?i&~:(member$ BRING_S2 ?i)) (name ?name_T3T4) (produced-puck NONE) (team ?team-color))
  (machine (mtype T2) (produced-puck S2) 
	   (incoming $?i-T2&~:(member$ PICK_PROD ?i-T2)) (name ?name_T2) (team ?team-color))
  (not (proposed-task (name pick-and-load) (args $?args&:(subsetp ?args (create$ ?name_T2 ?name_T3T4))) (state rejected)))
  (holding NONE)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T3_T4-WITH-S2*))))
  =>
  (printout t "PROD: Loading T3/T4 " ?name_T3T4 " with S2 from " ?name_T2 crlf)
  (assert (proposed-task (name pick-and-load) (args (create$ ?name_T2 ?name_T3T4)) (priority ?*PRIORITY-LOAD-T3_T4-WITH-S2*))
  )
)

(defrule prod-load-T5-with-S0
  (declare (salience ?*PRIORITY-LOAD-T5-WITH-S0*))
  (phase PRODUCTION)
  (state IDLE|WAIT_AND_LOOK_FOR_ALTERATIVE)
  (team-color ?team-color&~nil)
  (machine (mtype T5) (loaded-with $?l&~:(member$ S0 ?l))
    (incoming $?i&~:(member$ BRING_S0 ?i)) (name ?name) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S0) (args $?args&:(subsetp ?args (create$ ?name))) (state rejected)))
  (holding NONE|S0)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-T5-WITH-S0*))))
  =>
  (printout t "PROD: Loading T5 " ?name " with S0" crlf)
  (assert (proposed-task (name load-with-S0) (args (create$ ?name)) (priority ?*PRIORITY-LOAD-T5-WITH-S0*))
  )
)

(defrule prod-deliver-unintentionally-holding-produced-puck
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
  (declare (salience ?*PRIORITY-LOAD-HOLDING-S2*))
  (phase PRODUCTION)
  (state IDLE)
  (team-color ?team-color&~nil)
  (machine (mtype T3|T4) (loaded-with $?l&~:(member$ S2 ?l)) 
	   (incoming $?i&~:(member$ BRING_S2 ?i)) (name ?name_T3T4) (produced-puck NONE) (team ?team-color))
  (not (proposed-task (name load-with-S2) (args $?args&:(subsetp ?args (create$ ?name_T3T4))) (state rejected)))
  (holding S2)
  (not (proposed-task (state proposed) (priority ?max-prod&:(>= ?max-prod ?*PRIORITY-LOAD-HOLDING-S2*))))
  =>
  (printout t "PROD: Loading T3/T4 " ?name_T3T4 " with S2" crlf)
  (assert (proposed-task (name load-with-S2) (args (create$ ?name_T3T4)) (priority ?*PRIORITY-LOAD-HOLDING-S2*))
  )
)