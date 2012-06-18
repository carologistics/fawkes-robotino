
;---------------------------------------------------------------------------
;  worldmodel.clp - Robotino agent -- world model update rules
;
;  Created: Sat Jun 16 18:50:53 2012 (Mexico City)
;  Copyright  2012  Tim Niemueller [www.niemueller.de]
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; knowledge evaluation request
(deftemplate wm-eval
  (slot machine (type STRING))
  (slot was-holding (type SYMBOL) (allowed-values S0 S1 S2 P NONE))
  (slot now-holding (type SYMBOL) (allowed-values S0 S1 S2 P NONE))
)

; updates from external source
(deftemplate wm-ext-update
  (slot machine (type STRING))
  (slot mtype (type SYMBOL)
        (allowed-values M1_EXPRESS M1 M2 M3 M1_2 M2_3 RECYCLING TEST))
  (multislot loaded-with (type SYMBOL) (allowed-values S0 S1 S2))
)

(defrule wm-ext-mtype
  (declare (salience ?*PRIORITY_WM_EXT*))
  (wm-ext-update (machine ?name) (mtype ?mt))
  ?m <- (machine (name ?name) (mtype UNKNOWN))
  =>
  (modify ?m (mtype ?mt))
)

(defrule wm-ext-loaded-with
  (declare (salience ?*PRIORITY_WM_EXT*))
  (wm-ext-update (machine ?name) (loaded-with $?lw))
  ?m <- (machine (name ?name) (loaded-with $?l&:(> (length$ ?lw) 0)&:(subsetp ?l ?lw)))
  =>
  (modify ?m (loaded-with ?lw))
)


; Knowledge we can gain if pushing puck to unkown machine

(defrule wm-determine-unk-s0-s1
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S0) (now-holding S1))
  ; no junk at M1 ever
  ?m <- (machine (name ?name) (mtype UNKNOWN) (productions ?productions))
  =>
  (retract ?w)
  (modify ?m (mtype M1) (loaded-with) (productions (+ ?productions 1)))
)

(defrule wm-determine-unk-s0-none
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S0) (now-holding NONE))
  ?m <- (machine (name ?name) (mtype UNKNOWN) (loaded-with $?loaded))
  =>
  (retract ?w)
  (modify ?m (mtype M2_3) (loaded-with (insert$ ?loaded 1 S0)))
)

(defrule wm-determine-unk-s1-none
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S1) (now-holding NONE))
  ?m <- (machine (name ?name) (mtype UNKNOWN) (loaded-with $?loaded))
  =>
  (retract ?w)
  (modify ?m (mtype M2_3) (loaded-with (insert$ ?loaded 1 S1)))
)

(defrule wm-determine-unk-s1-s1
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S1) (now-holding S1))
  ?m <- (machine (name ?name) (mtype UNKNOWN))
  =>
  (retract ?w)
  (modify ?m (mtype M1))
)

(defrule wm-determine-unk-s2-s2
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S2) (now-holding S2))
  ?m <- (machine (name ?name) (mtype UNKNOWN))
  =>
  (retract ?w)
  (modify ?m (mtype M1_2))
)

(defrule wm-determine-unk-s2-none
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S2) (now-holding NONE))
  ?m <- (machine (name ?name) (mtype UNKNOWN) (loaded-with $?loaded))
  =>
  (retract ?w)
  (modify ?m (mtype M3) (loaded-with (insert$ ?loaded 1 S2)))
)


; Knowledge we can gain if pushing puck to m1,2 machine

(defrule wm-determine-m12-s0-s1
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S0) (now-holding S1))
  ; it's a M1 really, no junk ever
  ?m <- (machine (name ?name) (mtype M1_2) (productions ?productions))
  =>
  (retract ?w)
  (modify ?m (mtype M1) (loaded-with) (productions (+ ?productions 1)))
)

(defrule wm-determine-m12-s0-none
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S0) (now-holding NONE))
  ?m <- (machine (name ?name) (mtype M1_2) (loaded-with $?loaded))
  =>
  (retract ?w)
  (modify ?m (mtype M2) (loaded-with (insert$ ?loaded 1 S0)))
)

(defrule wm-determine-m12-s1-s1
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S1) (now-holding S1))
  ?m <- (machine (name ?name) (mtype M1_2))
  =>
  (retract ?w)
  (modify ?m (mtype M1))
)

(defrule wm-determine-m12-s1-s2
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S1) (now-holding S2))
  ?m <- (machine (name ?name) (mtype M1_2) (loaded-with $?loaded) (junk ?junk)
                 (productions ?productions))
  =>
  (retract ?w)
  (modify ?m (mtype M2) (loaded-with) (junk (+ ?junk (length$ ?loaded)))
          (productions (+ ?productions 1)))
)

(defrule wm-determine-m12-s1-none
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S1) (now-holding NONE))
  ?m <- (machine (name ?name) (mtype M1_2) (loaded-with $?loaded))
  =>
  (retract ?w)
  (modify ?m (mtype M2) (loaded-with (insert$ ?loaded 1 S1)))
)


; Knowledge we can gain if pushing puck to m2,3 machine

(defrule wm-determine-m23-s0-s2
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S0) (now-holding S2))
  ?m <- (machine (name ?name) (mtype M2_3) (loaded-with $?loaded) (junk ?junk)
                 (productions ?productions))
  =>
  (retract ?w)
  (modify ?m (mtype M2) (loaded-with) (junk (+ ?junk (length$ ?loaded)))
          (productions (+ ?productions 1)))
)

(defrule wm-determine-m23-s1-s2
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S1) (now-holding S2))
  ?m <- (machine (name ?name) (mtype M2_3) (loaded-with $?loaded) (junk ?junk)
                 (productions ?productions))
  =>
  (retract ?w)
  (modify ?m (mtype M2) (loaded-with) (junk (+ ?junk (length$ ?loaded)))
          (productions (+ ?productions 1)))
)

(defrule wm-determine-m23-s2-none
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S2) (now-holding NONE))
  ?m <- (machine (name ?name) (mtype M2_3) (loaded-with $?loaded))
  =>
  (retract ?w)
  (modify ?m (mtype M3) (loaded-with (insert$ ?loaded 1 S2)))
)

(defrule wm-determine-m23-s2-s2
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S2) (now-holding S2))
  ?m <- (machine (name ?name) (mtype M2_3))
  =>
  (retract ?w)
  (modify ?m (mtype M2))
)

(defrule wm-determine-m23_s1-s0-none
  (declare (salience ?*PRIORITY_WM*))
  ?w <- (wm-eval (machine ?name) (was-holding S0) (now-holding NONE))
  ?m <- (machine (name ?name) (mtype M2_3)
                 (loaded-with $?l&:(subsetp (create$ S1) ?l)))
  =>
  (retract ?w)
  (modify ?m (mtype M3) (loaded-with (insert$ ?l 1 S0)))
)

(defrule wm-good-consumed
  (declare (salience ?*PRIORITY_WM_DEF*))
  ?w <- (wm-eval (machine ?name) (was-holding ?p&~NONE) (now-holding NONE))
  ?m <- (machine (name ?name)
                 (loaded-with $?l&~:(subsetp (create$ ?p) ?l)))
  =>
  (retract ?w)
  (modify ?m (loaded-with (insert$ ?l 1 ?p)))
)

; A production of an M1 has been finished
(defrule wm-m1-production-done
  (declare (salience ?*PRIORITY_WM_DEF*))
  ?w <- (wm-eval (machine ?name) (was-holding S0) (now-holding S1))
  ?m <- (machine (name ?name) (mtype M1) (productions ?productions))
  =>
  (retract ?w)
  (modify ?m (loaded-with) (productions (+ ?productions 1)))
)

; A production of an M2 or M3 has been finished, set loaded-with to
; the empty set and increase junk cound accordingly
(defrule wm-m2-m3-production-done
  (declare (salience ?*PRIORITY_WM_DEF*))
  ?w <- (wm-eval (machine ?name)
                 (was-holding ?wh&~NONE) (now-holding ?n&~NONE&:(neq ?wh ?n)))
  ?m <- (machine (name ?name) (mtype M2|M3) (loaded-with $?loaded) (junk ?junk)
                 (productions ?productions))
  =>
  (retract ?w)
  (modify ?m (loaded-with) (junk (+ ?junk (length$ ?loaded)))
          (productions (+ ?productions 1)))
)

; A production of an M2 or M3 has been finished, set loaded-with to
; the empty set and increase junk cound accordingly
(defrule wm-delivered
  (declare (salience ?*PRIORITY_WM_DEF*))
  ?w <- (wm-eval (machine ?name) (was-holding P) (now-holding NONE))
  ?m <- (machine (name ?name) (mtype DELIVER) (productions ?productions))
  =>
  (retract ?w)
  (modify ?m (productions (+ ?productions 1)))
)


(defrule wm-cleanup-wm-eval
  (declare (salience ?*PRIORITY_CLEANUP*))
  ?w <- (wm-eval)
  =>
  (retract ?w)
)

(defrule wm-cleanup-ext-update
  (declare (salience ?*PRIORITY_CLEANUP*))
  ?w <- (wm-ext-update)
  =>
  (retract ?w)
)
