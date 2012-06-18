
;---------------------------------------------------------------------------
;  rules.clp - Robotino agent decision testing -- rules
;
;  Created: Sat Jun 16 12:35:16 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


; --- RULES - wait for start signal
(defrule start
  ?s <- (state WAIT_START)
  (start)
  =>
  (retract ?s)
  (assert (state IDLE))
)

; --- RULES - next machine/place to go to

(defrule get-s0
  (state IDLE)
  (holding NONE)
  =>
  (if (debug 3) then (printout t "Need to get S0" crlf))
  (assert (get-s0))
)

(defrule s0-m3-s1-s2
  (declare (salience ?*PRIORITY_P*))
  (state IDLE)
  (holding S0)
  (machine (mtype ?mt&M3) (loaded-with $?l&:(subsetp (create$ S1 S2) ?l)) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S0 1 -- Considering M3 named " ?name crlf))
  (modify ?g (min-prio (machine-prio ?mt))
          (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)))

)

(defrule s0-m12-s1
  (state IDLE)
  (holding S0)
  (machine (mtype ?mt&M1_2) (loaded-with $?l&:(subsetp (create$ S1) ?l)) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S0 2 -- Considering M1_2 named " ?name crlf))
  (modify ?g (min-prio (machine-prio ?mt))
          (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)))
)

(defrule s0-m23-s1
  (state IDLE)
  (holding S0)
  (machine (mtype ?mt&M2_3) (loaded-with $?l&:(subsetp (create$ S1) ?l)) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S0 3 -- Considering M2_3 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)


(defrule s0-m2-s1
  (declare (salience ?*PRIORITY_S2*))
  (state IDLE)
  (holding S0)
  (machine (mtype ?mt&M2) (loaded-with $?l&:(subsetp (create$ S1) ?l)) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S0 4 -- Considering M2 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule s0-m1
  (state IDLE)
  (holding S0)
  (machine (mtype ?mt&M1) (egc NO) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S0 5 -- Considering M1 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule s0-m-random
  (state IDLE)
  (holding S0)
  (machine (mtype ?mt&UNKNOWN) (name ?name))
  (not (machine (mtype M1)))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S0 6 -- Considering M? named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)


(defrule s1-m3-s2
  (state IDLE)
  (holding S1)
  (machine (mtype ?mt&M3) (loaded-with $?l&:(subsetp (create$ S2) ?l)) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S1 1 -- Considering M3 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule s1-m23-s0
  (state IDLE)
  (holding S1)
  (machine (mtype ?mt&M2_3) (loaded-with $?l&:(subsetp (create$ S0) ?l)) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S1 2 -- Considering M2_3 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule s1-m2-not-s1
  (state IDLE)
  (holding S1)
  (machine (mtype ?mt&M2) (name ?name) (loaded-with $?l&~:(subsetp (create$ S1) ?l)))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S1 3 -- Considering M2 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule s1-m-random
  (state IDLE)
  (holding S1)
  (machine (mtype ?mt&UNKNOWN) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S1 4 -- Considering M? named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule s2-m3-has-some-but-not-s2
  (state IDLE)
  (holding S2)
  (machine (mtype ?mt&M3) (name ?name) (loaded-with $?l&:(> (length$ ?l) 0)&~:(subsetp (create$ S2) ?l)))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S2 1 -- Considering M3 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule s2-m3
  (state IDLE)
  (holding S2)
  (machine (mtype ?mt&M3) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S2 1 -- Considering M3 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule s2-m23
  (state IDLE)
  (holding S2)
  (machine (mtype ?mt&M2_3) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S2 2 -- Considering M2_3 named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)


(defrule s2-random
  (state IDLE)
  (holding S2)
  (machine (mtype ?mt&UNKNOWN) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "S2 3 -- Considering M? named " ?name crlf))
  (modify ?g (machines (merge-goto-machines ?mp (machine-prio ?mt) ?machines ?name)) (min-prio (machine-prio ?mt)))
)

(defrule deliver-p
  (state IDLE)
  (holding P)
  (machine (mtype ?mt&DELIVER) (name ?name))
  ?g <- (goto (machines $?machines&~:(subsetp (create$ ?name) ?machines))
              (min-prio ?mp&:(<= ?mp (machine-prio ?mt))))
  =>
  (if (debug 2) then (printout t "P -- Need to deliver P " crlf))
  (modify ?g (machines ?name) (min-prio (machine-prio ?mt)))
)

; --- RULES for skill execution
(defrule skill-get-s0
  (declare (salience ?*PRIORITY_SKILL*))
  ?s <- (state IDLE)
  ?g <- (get-s0)
  =>
  (if (debug 3) then (printout t "Calling get-s0" crlf))
  (get-s0)
  (retract ?s ?g)
  (assert (state GET-S0))
)


(defrule skill-goto
  (declare (salience ?*PRIORITY_SKILL*))
  ; put first here to not match empty clause
  ?s <- (state IDLE)
  ?g <- (goto (machines ?first $?machines))
  (holding ?h)
  =>
  (if (debug 1) then
    (printout t "Calling take_puck_to_best for " (create$ ?first ?machines)
              " and puck " ?h crlf))
  (goto-machine (implode$ (create$ ?first ?machines)) ?h)
  (modify ?g (machines) (min-prio ?*GOTOPRIO_UNK*))
  (retract ?s)
  (assert (state GOTO))
  (assert (goto-holding ?h))
  (assert (goto-target ?first))
)

(defrule skill-goto-final
  ?s  <- (state GOTO-FINAL)
  ?gt <- (goto-target ?machine)
  ?gh <- (goto-holding ?was-holding)
  (machine (name ?machine))
  (holding ?now-holding)
  =>
  (retract ?s ?gt ?gh)
  (assert (wm-eval (machine ?machine)
                   (was-holding ?was-holding) (now-holding ?now-holding)))
  (assert (state IDLE))
)
