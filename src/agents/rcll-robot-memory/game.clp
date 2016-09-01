
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
  "If bots are waiting for game start and the state RUNNING is requested, check if the robot has to announce that it was restarted during the game = not in the setup phase."
  (phase EXPLORATION|PRODUCTION|WHACK_A_MOLE_CHALLENGE)
  ?sf <- (state WAIT_START)
  ?cf <- (change-state RUNNING)
  (started-in ?phase)
  (team-color ?team-color)
  (move-into-field-waittime ?wait-cfg)
  (time $?now)
  =>
  (retract ?sf ?cf)
  (assert (state MOVE_INTO_FIELD))
  (if (eq ?phase SETUP)
    then
    (bind ?wait ?wait-cfg)
    (assert (lock-announce-restart-finished))
    else
    (bind ?wait 0)
    (assert (lock-announce-restart))
  )
  (skill-call drive_into_field team ?team-color wait ?wait)
)

(defrule move-into-field-done
  "If the bot finished to move into the field, start the game."
  ?sf     <- (state MOVE_INTO_FIELD)
  ?final  <- (skill-done (name "drive_into_field") (status FINAL|FAILED))
  =>
  (retract ?sf ?final)
  (assert (state RESTART))
)

(defrule start-playing
  "Start the exploration, after the start / restart with restart announce"
  ?sf <- (state RESTART)
  (lock-announce-restart-finished)
  =>
  (retract ?sf)
  (assert (state IDLE))
  (assert (exploration-start))
)

(defrule pause
  "If game state is something in the middle of the game (not PAUSED or WAIT_START) and the requested change state is not RUNNING change states to PAUSED. The motor is disabled. Rule is not applied in clips-simulation."
  ?sf <- (state ?state&~PAUSED&~WAIT_START)
  ?cf <- (change-state ?cs&~RUNNING)
  ?rf <- (refbox-state ~?cs)
  (not (simulation-is-running))
  =>
  (printout t "Paused, disabling motor" crlf)
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
  (printout t "Unpaused, enabling motor" crlf)
  (retract ?sf ?pf ?cf ?rf)
  (assert (state ?old-state))
  (assert (refbox-state RUNNING))
  (motor-enable)
)

(defrule play-exploration-without-refbox
  "If config value play-exploration-without-refbox is true, immediately start EXPLORATION."
  (confval (path "/clips-agent/rcll2016/play-exploration-without-refbox") (value true))
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
  ?s <- (state ?exp-state&:(eq "EXP_" (sub-string 1 4 (str-cat ?exp-state))))
  =>
  (printout warn "removing exp-state because we are in production" crlf)
  (retract ?s)
)
