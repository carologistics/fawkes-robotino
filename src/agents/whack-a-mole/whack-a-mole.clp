;---------------------------------------------------------------------------
;  whac-a-mole.clp
;
;  Created: Sat Jun 22 16:33:55 2013
;  Copyright  2013 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule whack-starting-message
  (phase WHACK_A_MOLE_CHALLENGE)
  =>
  (printout t "Yippi ka yeah. I am playing the Whack-A-Mole-Challenge. Waiting for start." crlf)
)

(defrule whack-running-message
  (phase WHACK_A_MOLE_CHALLENGE)
  (state IDLE)
  (not (already-started))
  =>
  (printout t "Go! Go! Go!" crlf)
  (assert (already-started))
)

(defrule whack-try-to-find-light
  (phase WHACK_A_MOLE_CHALLENGE)
  ?sf <- (state IDLE)
  =>
  (printout t "Looking for a light." crlf)
  (retract ?sf)
  (assert (state SEARCHING))
  (skill-call drive_to_closest_light)
)

(defrule whack-drive-around-to-find-light
  (phase WHACK_A_MOLE_CHALLENGE)
  ?sf <- (state NO_LIGHT_VISIBLE)
  ?df <- (driven-to ?last)
  (navpath (start ?last) (goal ?next))
  =>
  (printout t "Driving to " ?next " to find light." crlf)
  (retract ?sf ?df)
  (assert (state DRIVING)
	  (driven-to ?next)
  )
  (skill-call ppgoto place (str-cat ?next))
)

(defrule whack-ppgoto-finished
  ?skf <- (skill (name "ppgoto") (status FINAL|FAILED) (skill-string ?skill))
  ?sf <- (state DRIVING)
  =>
  (retract ?skf ?sf)
  (assert (state IDLE))
)

(defrule whack-find-light-final
  ?skf <- (skill (name "drive_to_closest_light") (status FINAL))
  ?sf <- (state SEARCHING)
  =>
  (retract ?skf ?sf)
  (assert (state IDLE))
)

(defrule whack-find-light-failed
  ?skf <- (skill (name "drive_to_closest_light") (status FAILED))
  ?sf <- (state SEARCHING)
  =>
  (retract ?skf ?sf)
  (assert (state NO_LIGHT_VISIBLE))
)

;(defrule debug-skill-fail
;  (declare (salience ?*PRIORITY-HIGH*))
;  ?skf <- (skill (name "drive_to_closest_light") (status FINAL))
;  =>
;  (modify ?skf (status FAILED))  
;)
