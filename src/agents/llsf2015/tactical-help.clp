;---------------------------------------------------------------------------
;  tactical-help.clp - Some helper-functions/rules for tactical decisions
;
;  Created: Fri Mar 21 16:36:48 2014
;  Copyright  2014 Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; (deffacts tac-wait-positions
;   "facts to generate wait positions for each machine (Did not include this in the config because I would need a List of Lists)"
;   (common-wait-point WAIT_FOR_DELIVER_1 M12)
;   (common-wait-point WAIT_FOR_DELIVER_2 M24)
;   (common-wait-point WAIT_FOR_ROW_1 M22 M23)
;   (common-wait-point WAIT_FOR_ROW_2 M15 M18 M19)
;   (common-wait-point WAIT_FOR_ROW_3 M20 R2)
;   (common-wait-point WAIT_FOR_ROW_4 M14 M16 M17) 
;   (common-wait-point WAIT_FOR_ROW_5 M2 M4 M5)
;   (common-wait-point WAIT_FOR_ROW_6 M8 R1)
;   (common-wait-point WAIT_FOR_ROW_7 M3 M6 M7)
;   (common-wait-point WAIT_FOR_ROW_8 M10 M11)
;   (common-wait-point WAIT_FOR_INS_1_ROBOTINO_3 M9)
;   (common-wait-point WAIT_FOR_INS_2_ROBOTINO_3 M21)
;   (common-wait-point WAIT_FOR_INS_1_ROBOTINO_3 M13)
;   (common-wait-point WAIT_FOR_INS_1_ROBOTINO_2 M1)  
; )

(defrule tac-create-wait-point-facts
  "Convert the previously defined wait-point-lists to get a wait-point fact as for Ins and deliver"
  ?cwp <- (common-wait-point ?wait-point $?reses)
  (team-color CYAN|MAGENTA) ;to ensure the robot name is already set
  =>
  (foreach ?res ?reses
    (assert (wait-point ?res ?*ROBOT-NAME* ?wait-point))
  )
  (retract ?cwp)
)

(deffunction tac-can-use-timeslot (?current-time ?begin ?end ?estimated-time-needed)
  "Can the agent finish a task in a certain time slot with estimated execution time"
  (return (and (>= ?current-time (- ?begin ?estimated-time-needed))
	       (<= ?current-time (- ?end ?estimated-time-needed))))
)
