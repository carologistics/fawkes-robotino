;---------------------------------------------------------------------------
;  whac-a-mole.clp
;
;  Created: Sat Jun 22 16:33:55 2013
;  Copyright  2013 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

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

