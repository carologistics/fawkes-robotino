
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
  ?pf <- (phase ?)
  =>
  (retract ?cf ?pf)
  (assert (phase ?phase))
)

(defrule change-state-ignore
  (phase ~PRODUCTION&~EXPLORATION)
  ?cf <- (change-state ?)
  =>
  (retract ?cf)
)

(defrule start
  (phase EXPLORATION)
  ?sf <- (state WAIT_START)
  ?cf <- (change-state RUNNING)
  ?rf <- (refbox-state ?)
  =>
  (retract ?sf ?cf ?rf)
  (assert (state IDLE))
  (assert (refbox-state RUNNING))
  (assert (exploration-start))
)

(defrule pause
  ?sf <- (state ?state&~PAUSED)
  ?cf <- (change-state ?cs&~RUNNING)
  ?rf <- (refbox-state ~?cs)
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
  =>
  (retract ?sf ?pf ?cf ?rf)
  (assert (state ?old-state))
  (assert (refbox-state RUNNING))
  (motor-enable)
)

(defrule play-exploration-without-refbox
  (confval (path "/clips-agent/llsf2013/play-exploration-without-refbox") (value true))
  =>
  (assert 
    (exploration-start)
    (phase EXPLORATION)
    (state IDLE)
    (refbox-state RUNNING)
  )
  (printout t "PLAYING EXPLORATION WITHOUT REFBOX!" crlf)
)
