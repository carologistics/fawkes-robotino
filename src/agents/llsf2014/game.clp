
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
  "Changes game state"
  ?cf <- (change-phase ?phase)
  ?pf <- (phase ?old)
  =>
  (retract ?cf ?pf)
  (assert (phase ?phase))
)

(defrule change-state-ignore
  "State changes are only allowed during play"
  (phase ~PRODUCTION&~EXPLORATION&~WHACK_A_MOLE_CHALLENGE)
  ?cf <- (change-state ?)
  =>
  (retract ?cf)
)

(defrule enable-motor-on-start
  "Enables the motor on game start. Motor is not started in clips-simulation."
  (declare (salience ?*PRIORITY-HIGH*))
  (phase EXPLORATION|PRODUCTION|WHACK_A_MOLE_CHALLENGE)
  (state WAIT_START)
  (change-state RUNNING)
  (not (simulation-is-running))
  =>
  (printout warn "***** Enabling motor *****" crlf)
  (motor-enable)
)

(defrule started-in-which-phase
  "determine in which phase the agent was started to determine if it has to announce a restart"
  (state WAIT_START)
  (phase ?phase&~PRE_GAME)
  (not (started-in ?))
  =>
  (assert (started-in ?phase))
)

(defrule start
  "If bots are waiting for game start and the state RUNNING is requested move the bots into the field. Additional fact for P-3 bot with waiting time to delay its start."
  (phase EXPLORATION|PRODUCTION|WHACK_A_MOLE_CHALLENGE)
  ?sf <- (state WAIT_START)
  ?cf <- (change-state RUNNING)
  ?rf <- (refbox-state ?)
  (started-in ?phase)
  (time $?now)
  =>
  (retract ?sf ?cf ?rf)
  (assert (state MOVING_INTO_FIELD))
  (if (eq ?phase SETUP)
    then
    (assert (lock-announce-restart-finished))
    else
    (assert (lock-announce-restart))
  )
  (skill-call motor_move x 0.25 y 0)
  ;wait with P3-ONLY to avoid collision in the beginning
  (assert (timer (name wait-before-start) (time ?now)))
)

(defrule start-playing
  "When a bot (exluding P-3) has moved into the field, start the exploration."
  ?sf <- (state MOVING_INTO_FIELD)
  ?skf <- (skill-done (name "motor_move") (status FINAL|FAILED)) 
  (lock-announce-restart-finished)
  (not (role P3-ONLY))
  =>
  (retract ?sf ?skf)
  (assert (state IDLE))
  (assert (refbox-state RUNNING))
  (assert (exploration-start))
)

(defrule start-playing-as-P3_ONLY
  "When 10 seconds of game time have elapsed, move P-3 bot into the field."
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
  "If game state is something in the middle of the game (not PAUSED or WAIT_START) and the requested change state is not RUNNING change states to PAUSED. The motor is disabled. Rule is not applied in clips-simulation."
  ?sf <- (state ?state&~PAUSED&~WAIT_START)
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
  "If state is PAUSE and a state change to RUNNING is requested, change states to RUNNING. The motor is enabled again. Rule is not applied in clips-simulation."
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
  "If config value play-exploration-without-refbox is true, immediately start EXPLORATION."
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
  "When entering PRODUCTION phase, remove all states that are only needed in EXPLORATION."
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
