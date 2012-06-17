
;---------------------------------------------------------------------------
;  sim.clp - Robotino agent -- simulation
;
;  Created: Sat Jun 16 16:58:22 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate sim-machine
  (slot name (type STRING))
  (slot mtype (type SYMBOL) (allowed-values M1 M2 M3 DELIVER))
  (multislot loaded-with (type SYMBOL) (allowed-symbols S0 S1 S2))
  (slot junk (type INTEGER) (default 0))
  (slot delivered (type INTEGER) (default 0))
)

(deffacts simulation
  (sim-machine (name "m1")  (mtype M1))
  (sim-machine (name "m2")  (mtype M1))
  (sim-machine (name "m3")  (mtype M1))
  (sim-machine (name "m4")  (mtype M1))
  (sim-machine (name "m5")  (mtype M2))
  (sim-machine (name "m6")  (mtype M2))
  (sim-machine (name "m7")  (mtype M2))
  (sim-machine (name "m8")  (mtype M3))
  (sim-machine (name "m9")  (mtype M3))
  (sim-machine (name "m10") (mtype M3))
  (sim-machine (name "deliver") (mtype DELIVER))
  (s0-left 20)
)


(defrule get-s0-succeeds
  ?s  <- (state GET-S0)
  ?h  <- (holding NONE)
  ?gf <- (get-s0-final)
  ?l  <- (s0-left ?n&:(> ?n 0))
  =>
  (if (debug 1) then (printout t "get-s0 succeeded" crlf))
  (retract ?s ?h ?l)
  (assert (s0-left (- ?n 1)))
  (assert (holding S0))
  (assert (state IDLE))
)


(defrule get-s0-fails
  ?s <- (state GET-S0)
  ?h <- (holding NONE)
  ?l <- (s0-left ?n&:(<= ?n 0))
  =>
  (if (debug 1) then (printout t "NO MORE S0 AVAILABLE" crlf))
  (facts)
)


(defrule goto-succeeds-s0
  ?s  <- (state GOTO)
  ?gf <- (goto-final ?now-holding)
  ?h  <- (holding S0)
  (goto-target ?node)
  ?m  <- (sim-machine (name ?node) (mtype ?mt) (loaded-with $?loaded) (junk ?junk)) 
  =>
  (retract ?s ?h ?gf)
  (if (eq ?mt M1) then
    (assert (holding S1))
  else
    (if (eq ?mt M2) then
      (if (subsetp (create$ S1) ?loaded) then
        (assert (holding S2))
        (modify ?m (loaded-with) (junk (+ ?junk (length$ ?loaded))))
      else
        (assert (holding NONE))
        (modify ?m (loaded-with (insert$ ?loaded (+ (length$ ?loaded) 1) S0)))
      )
    else
      (if (eq ?mt M3) then
        (if (subsetp (create$ S1 S2) ?loaded) then
          (assert (holding P))
          (modify ?m (loaded-with) (junk (+ ?junk (length$ ?loaded))))
        else
          (assert (holding NONE))
          (modify ?m (loaded-with (insert$ ?loaded (+ (length$ ?loaded) 1) S0)))
        )
      )
    )
  )
  (assert (state GOTO-FINAL))
)


(defrule goto-succeeds-s1
  ?s  <- (state GOTO)
  ?h  <- (holding S1)
  ?gf <- (goto-final ?now-holding)
  (goto-target ?node)
  ?m  <- (sim-machine (name ?node) (mtype ?mt)
                      (loaded-with $?loaded) (junk ?junk)) 
  =>
  (retract ?s ?h ?gf)
  (if (eq ?mt M1) then
    (assert (holding S1))
  else
    (if (eq ?mt M2) then
      (if (subsetp (create$ S0) ?loaded) then
        (assert (holding S2))
        (modify ?m (loaded-with) (junk (+ ?junk (length$ ?loaded))))
      else
        (assert (holding NONE))
        (modify ?m (loaded-with (insert$ ?loaded (+ (length$ ?loaded) 1) S1)))
      )
    else
      (if (eq ?mt M3) then
        (if (subsetp (create$ S0 S2) ?loaded) then
          (assert (holding P))
          (modify ?m (loaded-with) (junk (+ ?junk (length$ ?loaded))))
        else
          (assert (holding NONE))
          (modify ?m (loaded-with (insert$ ?loaded (+ (length$ ?loaded) 1) S1)))
        )
      )
    )
  )
  (assert (state GOTO-FINAL))
)

(defrule goto-succeeds-s2
  ?s  <- (state GOTO)
  ?h  <- (holding S2)
  ?gf <- (goto-final ?now-holding)
  (goto-target ?node)
  ?m  <- (sim-machine (name ?node) (mtype ?mt) (loaded-with $?loaded) (junk ?junk))
  =>
  (retract ?s ?h ?gf)
  (if (eq ?mt M1) then
    (assert (holding S2))
  else
    (if (eq ?mt M2) then
      (assert (holding S2))
      (modify ?m (loaded-with) (junk (+ ?junk (length$ ?loaded))))
    else
      (if (eq ?mt M3) then
        (if (subsetp (create$ S0 S1) ?loaded) then
          (assert (holding P))
          (modify ?m (loaded-with) (junk (+ ?junk (length$ ?loaded))))
        else
          (assert (holding NONE))
          (modify ?m (loaded-with (insert$ ?loaded (+ (length$ ?loaded) 1) S2)))
        )
      )
    )
  )
  (assert (state GOTO-FINAL))
)

(defrule goto-succeeds-p
  ?s  <- (state GOTO)
  ?h  <- (holding P)
  ?gf <- (goto-final ?now-holding)
  (goto-target ?node)
  ?m  <- (sim-machine (name ?node) (mtype DELIVER) (delivered ?delivered)) 
  =>
  (if (debug 1) then (printout t "***** DELIVERED a P! *****" crlf))
  (retract ?s ?h ?gf)
  (modify ?m (delivered (+ ?delivered 1)))
  (assert (holding NONE))
  (assert (state GOTO-FINAL))
)
