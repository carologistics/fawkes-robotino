
;---------------------------------------------------------------------------
;  lock-actions.clp - CLIPS executive - lock action executors
;
;  Created: Tue Apr 24 21:32:19 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;                        Till Hofmann
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defrule lock-action-execute
" Execute a lock action by setting the state to EXECUTION-SUCCEEDED.
  This causes the locked domain fact to be asserted."
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name ?action&lock|one-time-lock)
                      (state PENDING) (executable TRUE)
                      (param-values ?name))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule unlock-action-execute
" Execute an unlock action by setting the state to EXECUTION-SUCCEEDED.
  This causes the locked domain fact to be retracted.
"
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name unlock)
                      (state PENDING) (executable TRUE)
                      (param-values ?name))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule location-lock-action-execute
" Execute a location-lock action by asserting the corresponding
  location-locked domain fact.
"
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name location-lock)
                      (state PENDING) (executable TRUE)
                      (param-values ?mps ?side))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))
  (assert (domain-fact (name location-locked) (param-values ?mps ?side)))
)

(defrule location-unlock-action-start
" Start execution of a location-unlock action by asserting the
  location-unlock-pending fact. After a constant time the location-locked
  fact will be retracted, giving the robot time to clear the location
"
  ?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name location-unlock)
                      (state PENDING) (executable TRUE)
                      (param-values ?mps ?side))
  (wm-fact (key refbox game-time) (values ?game-time $?))
  =>
  (modify ?pa (state EXECUTION-SUCCEEDED))  
  (assert (location-unlock-pending ?mps ?side ?game-time))
)

(defrule location-unlock-action-end
" Finish execution of a location-unlock action by retracting the
  location-locked domain fact after a constant time.
"
  (wm-fact (key refbox game-time) (values ?game-time $?))
  ?lock-fact <- (location-unlock-pending ?mps ?side ?old-game-time&:(< (+ ?old-game-time 2) ?game-time))
  ?df <- (domain-fact (name location-locked) (param-values ?mps ?side))
  =>
  (retract ?lock-fact)
  (retract ?df)
)
