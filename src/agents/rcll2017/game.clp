
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
  (or
    (change-phase ~PRODUCTION)
    (waitpoints-done)
  )
  =>
  (retract ?cf ?pf)
  (assert (phase ?phase))
)

(defrule change-state-ignore
  "State changes are only allowed during play"
  (phase ~PRODUCTION&~EXPLORATION&~POST_GAME)
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
  (pose (x ?px) (y ?py))
  (not (and
    (active-robot (name ?name) (x ?x&:(< (abs ?x) (abs ?px))) (y ?y))
    (insertion-zone ?z&:(eq ?z (get-zone 0 (create$ ?x ?y))))
    (team-robot ?name)
  ))
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
  (printout t ?*ROBOT-NAME* crlf)
  (if (eq ?*ROBOT-NAME* "R-1")
    then
    (skill-call drive_into_field team ?team-color wait 0)
    else
    (skill-call drive_into_field team ?team-color wait ?wait)
  )
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
  ; (not (simulation-is-running))
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
  ; (not (simulation-is-running))
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


(defrule game-receive-field-layout-protobuf
"At the end of the exploration phase, the Refbox sends the true field layout.
 Assert a field-ground-truth fact for each machine the Refbox told us about."
  (declare (salience ?*PRIORITY-LOW*))
  (not (received-field-layout))
  ?msg <- (protobuf-msg (type "llsf_msgs.MachineInfo") (ptr ?p))
=>
  (foreach ?machine (pb-field-list ?p "machines")
    (bind ?name (sym-cat (pb-field-value ?machine "name")))
    (bind ?rot  FALSE)
    (bind ?zone FALSE)
    (bind ?type FALSE)
    (if (pb-has-field ?p "rotation") then
      (bind ?rot  (pb-field-value ?machine "rotation"))
    )
    (if (pb-has-field ?p "zone") then
      (bind ?zone (clips-name (pb-field-value ?machine "zone")))
    )
    (if (pb-has-field ?p "type") then
      (bind ?type (sym-cat (pb-field-value ?machine "type")))
    )

    (if (and ?zone ?rot ?type) then
      (assert
        (field-ground-truth
          (machine ?name)
          (yaw (deg-to-rad ?rot))
          (orientation ?rot)
          (zone ?zone)
          (mtype ?type)
        )
      )
      (assert (received-field-layout))
    else
      (printout t "Received incomplete ground-truth from refbox. Machine: " ?name
        ", rot: " ?rot ", zone: " ?zone ", type: " ?type crlf)
    )
  )
  (retract ?msg)
)


(defrule game-zone-exploration-mark-correct
  ?gt <- (field-ground-truth (machine ?machine) (orientation ?ori) (zone ?zone) (mtype ?mtype))
  (found-tag (name ?machine))
  (exploration-result (machine ?machine) (zone ?zone) (orientation ?ori))
  ?ze <- (zone-exploration (name ?zone))
=>
  (modify ?ze (correct TRUE))
  (retract ?gt)
)


(defrule game-complete-zone-exploration-with-ground-truth
"Integrate incoming field-ground-truth facts into our world model."
  ?gt <- (field-ground-truth (machine ?machine) (yaw ?yaw) (zone ?zone) (mtype ?mtype))
  (not (found-tag (name ?machine)))
  ?ze <- (zone-exploration (name ?zone) (machine UNKNOWN|NONE))
=>
  (assert
    (found-tag (name ?machine) (side INPUT) (frame "map")
      (trans (tag-offset ?zone ?yaw 0.17))
      (rot (tf-quat-from-yaw ?yaw))
    )
  )
  (modify ?ze (machine ?machine) (correct TRUE))
  (retract ?gt)
)
