;Whac a mole game
;---------------------------------------------------------------------------
;  Whac-a-mole.clp
;
;  Created: Thu Dec 20 12:11:17 2012
;  Copyright  2012  Tim Niemueller [www.niemueller.de], Alex, Frederik, Richard
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule test-skill
  ?st  <- (start)
  =>
  (retract ?st)
  (assert (status "drivingToLight"))
  (assert (goal 0.0 0.0))
  (printout t "Yippi ka yeah. I am playing Whac a mole." crlf)
)


(defrule print-command
  (Position3DInterface (id "Light") (translation $?t))
  =>
  (printout t "Got new Light-pos: (" (nth$ 1 ?t)", " (nth$ 2 ?t) ")" crlf)
)

;(defrule poschange
;  (Position3DInterface (id "Pose") (time $?time) (translation $?t))
;  ?pos <- (position ?x ?y&:(<> (nth$ 1 ?t) ?x)|:(<> (nth$ 2 ?t) ?y))
;  ?lmt <- (lastmovetime $?tv)
;  =>
;  (retract ?pos)
;  (retract ?lmt)
;  (assert (position (nth$ 1 ?t) (nth$ 2 ?t)))
;  (assert (lastmovetime ?time))
;)

;(defrule nochange-reset
;  (Position3DInterface (id "Pose") (time $?time))
;  ?lmt <- (lastmovetime $?tv&:(timeout ?time ?tv 1.0))
;  =>
;  (skill-call ppgoto place "West")
;  (retract ?lmt)
;  (assert (lastmovetime ?time))
;)

(defrule goto-found-light
  ?s <- (status ?text)
  (Position3DInterface (id "Light") (translation $?t&:(<> (nth$ 1 ?t) 0.)|:(<> (nth$ 2 ?t) 0.)))
  ?g <- (goal ?g1 ?g2&:(<> (nth$ 2 ?t) ?g2)|:(<> (nth$ 1 ?t) ?g1))
  =>
  (retract ?s)
  (retract ?g)
  (assert (status "drivingToLight"))
  (assert (goal (nth$ 1 ?t) (nth$ 2 ?t)))
  (skill-call relgoto rel_x (nth$ 1 ?t) rel_y (nth$ 2 ?t))
  (printout t "goto light" (nth$ 1 ?t) " " (nth$ 2 ?t) crlf)
)

(defrule goto-nearest-visionpoint
  ?s <- (status "drivingToLight")
  (Position3DInterface (id "Light") (translation $?t&:(= (nth$ 1 ?t) 0.)&:(= (nth$ 2 ?t) 0.)))
  (Position3DInterface (id "Pose") (time $?time) (translation $?pos))
  (navpoint (name ?v) (x ?x) (y ?y))
  (blocked ?blocked&~?v)
  (navpoint (name "North West") (x ?x1) (y ?y1&:(<= (+ (* (- ?x (nth$ 1 ?pos)) (- ?x (nth$ 1 ?pos))) (* (- ?y (nth$ 2 ?pos)) (- ?y (nth$ 2 ?pos)))) (+ (* (- ?x1 (nth$ 1 ?pos)) (- ?x1 (nth$ 1 ?pos))) (* (- ?y1 (nth$ 2 ?pos)) (- ?y1 (nth$ 2 ?pos)))))))
  (navpoint (name "South") (x ?x2) (y ?y2&:(<= (+ (* (- ?x (nth$ 1 ?pos)) (- ?x (nth$ 1 ?pos))) (* (- ?y (nth$ 2 ?pos)) (- ?y (nth$ 2 ?pos)))) (+ (* (- ?x2 (nth$ 1 ?pos)) (- ?x2 (nth$ 1 ?pos))) (* (- ?y2 (nth$ 2 ?pos)) (- ?y2 (nth$ 2 ?pos)))))))
  (navpoint (name "South East") (x ?x3) (y ?y3&:(<= (+ (* (- ?x (nth$ 1 ?pos)) (- ?x (nth$ 1 ?pos))) (* (- ?y (nth$ 2 ?pos)) (- ?y (nth$ 2 ?pos)))) (+ (* (- ?x3 (nth$ 1 ?pos)) (- ?x3 (nth$ 1 ?pos))) (* (- ?y3 (nth$ 2 ?pos)) (- ?y3 (nth$ 2 ?pos)))))))
  (navpoint (name "North East") (x ?x4) (y ?y4&:(<= (+ (* (- ?x (nth$ 1 ?pos)) (- ?x (nth$ 1 ?pos))) (* (- ?y (nth$ 2 ?pos)) (- ?y (nth$ 2 ?pos)))) (+ (* (- ?x4 (nth$ 1 ?pos)) (- ?x4 (nth$ 1 ?pos))) (* (- ?y4 (nth$ 2 ?pos)) (- ?y4 (nth$ 2 ?pos)))))))
  ?lmt <- (lastnavpointtime $?tv)
  =>
  (printout t "nearest vp:" ?v crlf)
  (skill-call ppgoto place ?v)
  (retract ?s)
  (assert (status ?v))
  (retract ?lmt)
  (assert (lastnavpointtime ?time))
)

(defrule search-pattern
  ?final <- (skill (name "ppgoto") (status FINAL) (skill-string ?skill))
  (Position3DInterface (id "Pose") (time $?time))
  ?lmt <- (lastnavpointtime $?tv)
  ?s <- (status ?v)
  (navpath (start ?v) (goal ?g))
  ?blocked <- (blocked ?b)
  =>
  (printout t "driving to " ?g crlf)
  (skill-call ppgoto place ?g)
  (retract ?s)
  (retract ?blocked)
  (retract ?final)
  (retract ?lmt)
  (assert (status ?g))
  (assert (blocked ?v))
  (assert (lastnavpointtime ?time))
)

(defrule search-pattern-time
  ?final <- (skill (name "ppgoto") (status RUNNING) (skill-string ?skill))
  (Position3DInterface (id "Pose") (time $?time))
  ?lmt <- (lastnavpointtime $?tv&:(timeout ?time ?tv 60.0))
  ?s <- (status ?v)
  (navpath (start ?v) (goal ?g))
  ?blocked <- (blocked ?b)
  =>
  (printout t "driving to " ?g crlf)
  (skill-call ppgoto place ?g)
  (retract ?s)
  (retract ?blocked)
  (retract ?final)
  (retract ?lmt)
  (assert (status ?g))
  (assert (blocked ?v))
  (assert (lastnavpointtime ?time))
)
