
;---------------------------------------------------------------------------
;  game.clp - Robotino agent game related rules
;
;  Created: Fri Apr 26 18:31:13 2013 (Magdeburg)
;  Copyright  2013  Tim Niemueller [www.niemueller.de]
;             2013  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

; --- GAME STATE handling

(defrule change-phase
  ?cf <- (change-phase ?phase)
  ?pf <- (phase ?old)
  =>
  (retract ?cf ?pf)
  (assert (phase ?phase))
)

(defrule change-state-ignore
  (phase ~PRODUCTION&~EXPLORATION&~WHACK_A_MOLE_CHALLENGE)
  ?cf <- (change-state ?)
  =>
  (retract ?cf)
)

(defrule enable-motor-on-start
  (declare (salience ?*PRIORITY-HIGH*))
  (phase EXPLORATION|PRODUCTION|WHACK_A_MOLE_CHALLENGE)
  (state WAIT_START)
  (change-state RUNNING)
  (not (simulation-is-running))
  =>
  (printout warn "***** Enabling motor *****" crlf)
  (motor-enable)
)

(defrule start
  (phase EXPLORATION|PRODUCTION|WHACK_A_MOLE_CHALLENGE)
  ?sf <- (state WAIT_START)
  ?cf <- (change-state RUNNING)
  ?rf <- (refbox-state ?)
  (time $?now)
  =>
  (retract ?sf ?cf ?rf)
  (assert (state MOVING_INTO_FIELD))
  (skill-call motor_move x 0.25 y 0)
  ;wait with P3-ONLY to avoid collision in the beginning
  (assert (timer (name wait-before-start) (time ?now)))
)

(defrule start-playing
  ?sf <- (state MOVING_INTO_FIELD)
  ?skf <- (skill-done (name "motor_move") (status FINAL|FAILED)) 
  (not (role P3-ONLY))
  =>
  (retract ?sf ?skf)
  (assert (state IDLE))
  (assert (refbox-state RUNNING))
  (assert (exploration-start))
)

(defrule start-playing-as-P3_ONLY
  ?sf <- (state MOVING_INTO_FIELD)
  ?skf <- (skill-done (name "motor_move") (status FINAL|FAILED)) 
  (role P3-ONLY)
  (time $?now)
  ?timer <- (timer (name wait-before-start) (time $?t&:(timeout ?now ?t 10.0)))
  =>
  (retract ?sf ?skf ?timer)
  (assert (state IDLE))
  (assert (refbox-state RUNNING))
  (assert (exploration-start))
)

(defrule pause
  ?sf <- (state ?state&~PAUSED)
  ?cf <- (change-state ?cs&~RUNNING)
  ?rf <- (refbox-state ~?cs)
  (not (simulation-is-running))
  =>
  (retract ?sf ?cf ?rf)
  (motor-disable)
  (assert (state PAUSED))
  (assert (refbox-state PAUSED))
  (assert (pre-pause-state ?state))
)

(defrule unpause
  ?sf <- (state PAUSED)
  ?pf <- (pre-pause-state ?old-state)
  ?cf <- (change-state RUNNING)
  ?rf <- (refbox-state PAUSED)
  (not (simulation-is-running))
  =>
  (retract ?sf ?pf ?cf ?rf)
  (assert (state ?old-state))
  (assert (refbox-state RUNNING))
  (motor-enable)
)

(defrule play-exploration-without-refbox
  (confval (path "/clips-agent/llsf2014/play-exploration-without-refbox") (value true))
  =>
  (assert 
    (exploration-start)
    (phase EXPLORATION)
    (state IDLE)
    (refbox-state RUNNING)
  )
  (printout t "PLAYING EXPLORATION WITHOUT REFBOX!" crlf)
)

(defrule game-remove-exploration-states
  (phase PRODUCTION)
  ?s <- (state EXP_START|
	       EXP_IDLE|
	       EXP_LOCK_REQUIRED|
	       EXP_DRIVING_TO_MACHINE|
	       EXP_DRIVING_TO_MACHINE_GLOBAL|
	       EXP_WAITING_AT_MACHINE|
	       EXP_FOUND_NEXT_MACHINE|
	       EXP_PREPARE_FOR_PRODUCTION|
	       EXP_LOCK_ACCEPTED|
	       EXP_PREPARE_FOR_PRODUCTION_FINISHED)
  =>
  (printout warn "removing exp-state because we are in production" crlf)
  (retract ?s)
)