
;---------------------------------------------------------------------------
;  lock-actions.clp - CLIPS executive - lock action executors
;
;  Created: Tue Apr 24 21:32:19 2018
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;                        Till Hofmann
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(defglobal
  ?*LOCK-ACTION-RETRY-PERIOD-SEC* = 2
  ?*LOCK-ACTION-TIMEOUT-SEC* = 20
  ?*UNLOCK-DISTANCE* = 0.5
)

(deftemplate lock-info
  (slot name (type SYMBOL) (default ?NONE))
  (slot goal-id (type SYMBOL))
  (slot plan-id (type SYMBOL))
  (slot action-id (type INTEGER))
  (slot status (type SYMBOL) (allowed-values REQUESTED WAITING)
    (default REQUESTED))
  (multislot start-time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (multislot last-try (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot last-error)
)

(defrule lock-actions-lock-start
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name ?action&lock|location-lock|one-time-lock)
                      (state PENDING) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (time $?now)
	=>
  (if (or (eq ?action lock) (eq ?action one-time-lock))  then
	  (bind ?lock-name (plan-action-arg name ?param-names ?param-values))
  else
    (bind ?lock-name
      (sym-cat (plan-action-arg location ?param-names ?param-values)
               -
               (plan-action-arg side ?param-names ?param-values)))
  )
	(mutex-try-lock-async ?lock-name)
  (printout warn "Trying to lock " ?lock-name crlf)
  (assert (lock-info (goal-id ?goal-id) (plan-id ?plan-id) (action-id ?id)
            (name ?lock-name) (start-time ?now) (last-try ?now)))
	(modify ?pa (state RUNNING))
)

(defrule lock-actions-lock-acquired
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name ?action-name&lock|location-lock|one-time-lock)
                      (param-values $?param-values)
                      (state RUNNING))
  ?mf <- (mutex (name ?name) (request LOCK) (response ACQUIRED))
  ?li <- (lock-info (name ?name) (goal-id ?goal-id) (plan-id ?plan-id)
                    (action-id ?id))
	=>
  (printout warn "Successfully locked " ?name crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(modify ?mf (request NONE) (response NONE))
  (if (eq ?action-name location-lock) then
    (assert (domain-fact (name location-locked) (param-values $?param-values)))
  )
  (retract ?li)
)

(defrule lock-actions-lock-rejected
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name lock|location-lock) (state RUNNING))
  ?mf <- (mutex (name ?name) (response REJECTED|ERROR)
                (error-msg ?error-msg))
  ?li <- (lock-info (name ?name) (goal-id ?goal-id) (plan-id ?plan-id)
                    (action-id ?id))
	=>
  (printout warn "Lock " ?name " was rejected " crlf)
	(modify ?mf (request NONE) (response NONE) (error-msg ""))
  (modify ?li (status WAITING) (last-error ?error-msg))
)

(defrule lock-actions-one-time-lock-rejected
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name one-time-lock) (state RUNNING))
  ?mf <- (mutex (name ?name) (response REJECTED|ERROR)
                (error-msg ?error-msg))
  ?li <- (lock-info (name ?name) (goal-id ?goal-id) (plan-id ?plan-id)
                    (action-id ?id))
	=>
  (printout warn "Lock " ?name " was rejected " crlf)
	(modify ?mf (request NONE) (response NONE) (error-msg ""))
  (retract ?li )
  (modify ?pa (state EXECUTION-FAILED) (error-msg ?error-msg))
)


(defrule lock-actions-lock-retry
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name lock|location-lock) (state RUNNING))
  ?li <- (lock-info (name ?name) (goal-id ?goal-id) (plan-id ?plan-id)
                    (action-id ?id) (status WAITING) (last-try $?last))
  (time $?now&:(timeout ?now ?last ?*LOCK-ACTION-RETRY-PERIOD-SEC*))
  =>
  (printout warn "Retrying to lock " ?name crlf)
  (mutex-try-lock-async ?name)
  (modify ?li (status REQUESTED) (last-try ?now))
)

(defrule lock-actions-lock-failed
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
                      (action-name lock|location-lock) (state RUNNING))
  ?li <- (lock-info (name ?name) (goal-id ?goal-id) (plan-id ?plan-id)
                    (action-id ?id) (status WAITING) (start-time $?start)
                    (last-error ?error-msg))
  (time $?now&:(timeout ?now ?start ?*LOCK-ACTION-TIMEOUT-SEC*))
  =>
  (printout warn "Failed to get lock " ?name " in " ?*LOCK-ACTION-TIMEOUT-SEC*
    "s, giving up" crlf)
	(modify ?pa (state EXECUTION-FAILED) (error-msg ?error-msg))
  (retract ?li)
)

(defrule lock-actions-unlock-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (state PENDING)
                      (action-name unlock) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	; The following performs a synchronous/blocking call
	;(bind ?rv (robmem-mutex-unlock (str-cat ?lock-name)))
	;(modify ?pa (state (if ?rv then EXECUTION-SUCCEEDED else EXECUTION-FAILED)))
	(mutex-unlock-async ?lock-name)
	(modify ?pa (state RUNNING))
)

(defrule lock-actions-unlock-done
  ?pa <- (plan-action (id ?id) (action-name unlock) (state RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
								(request UNLOCK) (response UNLOCKED))
	=>
	(printout t "Unlock of " ?name " successfull" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
  ;(assert (domain-fact (name location-locked) (param-values $?param-values)))
	(modify ?mf (request NONE) (response NONE))
)

(defrule lock-actions-unlock-location
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name location-unlock) (state PENDING)
                      (param-names $?param-names) (param-values $?param-values))
  ?mf <- (mutex (name ?name&:(eq ?name
                  (sym-cat (plan-action-arg location ?param-names ?param-values)
                  -
                  (plan-action-arg side ?param-names ?param-values))))
         )
  =>
  (bind ?lock-name (sym-cat
                    (plan-action-arg location ?param-names ?param-values)
                    -
                    (plan-action-arg side ?param-names ?param-values)))
  (printout warn "Eventually unlock " ?lock-name crlf)
	(modify ?pa (state RUNNING))
  ; Do not actually unlock, we use distance-based unlocking.
	;(mutex-unlock-async ?lock-name)
)

(defrule lock-actions-unlock-location-pending
  ?pa <- (plan-action (action-name location-unlock) (state RUNNING)
                      (param-names $?param-names)
                      (param-values $?param-values))
  =>
  (assert (location-unlock-pending
            (plan-action-arg location ?param-names ?param-values)
            (plan-action-arg side ?param-names ?param-values)))
  (modify ?pa (state EXECUTION-SUCCEEDED))
)

(defrule lock-actions-distance-based-unlocking
  (domain-object (name ?loc))
  (domain-obj-is-of-type ?loc mps)
  (domain-object (name ?side))
  (domain-obj-is-of-type ?side mps-side)
  (wm-fact (key cx identity) (value ?self))
  (location-unlock-pending ?loc ?side)
  ?mf <- (mutex (name ?lock-name&:(eq ?lock-name (sym-cat ?loc - ?side)))
                (state LOCKED) (request NONE) (locked-by ?self))
  (navgraph-node
    (name ?node&:(eq ?node
                     (str-cat ?loc (if (eq ?side INPUT) then -I else -O))))
    (pos $?mps-pose))
  (Position3DInterface (id "Pose") (translation $?pose))
  (test (> (distance-mf ?pose ?mps-pose) ?*UNLOCK-DISTANCE*))
  =>
  (printout warn "Unlocking " ?lock-name " based on distance ("
                 (distance-mf ?pose ?mps-pose) ")" crlf)
  (mutex-unlock-async ?lock-name)
)

(defrule lock-actions-unlock-location-done
  ?l <- (location-unlock-pending ?loc ?side)
  (mutex (name ?lock-name&:(eq ?lock-name (sym-cat ?loc - ?side)))
         (state OPEN) (request NONE))
  =>
  (retract ?l)
)
