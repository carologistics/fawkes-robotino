
;---------------------------------------------------------------------------
;  rules.clp - Robotino agent decision testing -- rules
;
;  Created: Sat Jun 16 12:35:16 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; --- RULES - general housekeeping
(defrule retract-time
  (declare (salience ?*PRIORITY_LAST*))
  ?f <- (time $?)
  =>
  (retract ?f)
  ;(facts)
)

; --- RULES - next machine/place to go to

(defrule start
  ?s <- (state WAIT_START)
  (start)
  =>
  (retract ?s)
  (assert (state IDLE))
)


(defrule get-s0
  (state IDLE)
  (holding NONE)
  =>
  (if (debug 1) then (printout t "Need to get S0" crlf))
  (assert (get-s0))
)

(defrule s0-m3-s1-s2
  (declare (salience ?*PRIORITY_P*))
  (state IDLE)
  (holding S0)
  (machine (mtype M3) (loaded-with $?l&:(subsetp (create$ S1 S2) ?l)) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S0 1 -- Need to go to M3 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s0-m12-s1
  (state IDLE)
  (holding S0)
  (machine (mtype M1_2) (loaded-with $?l&:(subsetp (create$ S1) ?l)) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S0 2 -- Need to go to M1_2 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s0-m23-s1
  (state IDLE)
  (holding S0)
  (machine (mtype M2_3) (loaded-with $?l&:(subsetp (create$ S1) ?l)) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S0 3 -- Need to go to M2_3 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)


(defrule s0-m2-s1
  (declare (salience ?*PRIORITY_S2*))
  (state IDLE)
  (holding S0)
  (machine (mtype M2) (loaded-with $?l&:(subsetp (create$ S1) ?l)) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S0 4 -- Need to go to M2 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s0-m1
  (state IDLE)
  (holding S0)
  (machine (mtype M1) (egc NO) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S0 5 -- Need to go to M1 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s0-m-random
  (state IDLE)
  (holding S0)
  (machine (mtype UNKNOWN) (name ?name))
  (not (machine (mtype M1)))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S0 6 -- Need to go to M? named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)


(defrule s1-m3-s2
  (state IDLE)
  (holding S1)
  (machine (mtype M3) (loaded-with $?l&:(subsetp (create$ S2) ?l)) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S1 1 -- Need to go to M3 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s1-m23-s0
  (state IDLE)
  (holding S1)
  (machine (mtype M2_3) (loaded-with $?l&:(subsetp (create$ S0) ?l)) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S1 2 -- Need to go to M2_3 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s1-m2-not-s1
  (state IDLE)
  (holding S1)
  (machine (mtype M2) (name ?name) (loaded-with $?l&~:(subsetp (create$ S1) ?l)))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S1 3 -- Need to go to M2 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s1-m-random
  (state IDLE)
  (holding S1)
  (machine (mtype UNKNOWN) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S1 4 -- Need to go to M? named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s2-m3
  (state IDLE)
  (holding S2)
  (machine (mtype M3) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S2 1 -- Need to go to M3 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule s2-m23
  (state IDLE)
  (holding S2)
  (machine (mtype M2_3) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S2 2 -- Need to go to M2_3 named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)


(defrule s2-random
  (state IDLE)
  (holding S2)
  (machine (mtype UNKNOWN) (name ?name))
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ ?name) ?nodes)))
  =>
  (if (debug 1) then (printout t "S2 3 -- Need to go to M? named " ?name crlf))
  (modify ?g (nodes (append$ ?nodes ?name)))
)

(defrule deliver-p
  (state IDLE)
  (holding P)
  ?g <- (goto (nodes $?nodes&~:(subsetp (create$ "deliver") ?nodes)))
  =>
  (if (debug 1) then (printout t "P -- Need to deliver P " crlf))
  (modify ?g (nodes "deliver"))
)

; --- RULES for skill execution
(defrule skill-get-s0
  (declare (salience ?*PRIORITY_SKILL*))
  ?s <- (state IDLE)
  ?g <- (get-s0)
  =>
  (if (debug 1) then (printout t "Calling get-s0" crlf))
  ;(get-s0)
  (retract ?s ?g)
  (assert (state GET-S0))
)


(defrule skill-goto
  (declare (salience ?*PRIORITY_SKILL*))
  ; put first here to not match empty clause
  ?s <- (state IDLE)
  ?g <- (goto (nodes ?first $?nodes))
  (holding ?h)
  =>
  (if (debug 1) then (printout t "Calling goto for " ?first ?nodes crlf))
  ;(goto-machine (create$ ?first ?nodes))
  (modify ?g (nodes))
  (retract ?s)
  (assert (state GOTO))
  (assert (goto-holding ?h))
  (assert (goto-target ?first))
)

(defrule skill-goto-final
  ?s  <- (state GOTO-FINAL)
  ?gt <- (goto-target ?node)
  ?gh <- (goto-holding ?was-holding)
  (machine (name ?node))
  (holding ?now-holding)
  =>
  (retract ?s ?gt ?gh)
  (assert (wm-eval (machine ?node)
                   (was-holding ?was-holding) (now-holding ?now-holding)))
  (assert (state IDLE))
)
