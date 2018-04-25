
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
  ?*LOCK-ACTION-TIMEOUT-SEC* = 60
)

(deftemplate lock-info
  (slot name (type SYMBOL) (default ?NONE))
  (slot status (type SYMBOL) (allowed-values REQUESTED WAITING)
    (default REQUESTED))
  (multislot start-time (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (multislot last-try (type INTEGER) (cardinality 2 2) (default (create$ 0 0)))
  (slot last-error)
)

(defrule lock-actions-lock-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name lock) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
  (time $?now)
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	; The following performs a synchronous/blocking call
	;(bind ?rv (robmem-mutex-try-lock (str-cat ?lock-name)))
	;(modify ?pa (status (if ?rv then EXECUTION-SUCCEEDED else EXECUTION-FAILED)))
	(mutex-try-lock-async ?lock-name)
  (printout warn "Trying to lock " ?lock-name crlf)
  (assert (lock-info (name ?lock-name) (start-time ?now) (last-try ?now)))
	(modify ?pa (status RUNNING))
)

(defrule lock-actions-lock-acquired
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name lock) (status RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
								(request LOCK) (response ACQUIRED))
  ?li <- (lock-info (name ?name))
	=>
  (printout warn "Successfully locked " ?name crlf)
	(modify ?pa (status EXECUTION-SUCCEEDED))
	(modify ?mf (request NONE) (response NONE))
  (retract ?li)
)

(defrule lock-actions-lock-rejected
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name lock) (status RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
								(request LOCK) (response REJECTED|ERROR) (error-msg ?error-msg))
  ?li <- (lock-info (name ?name))
	=>
	;(modify ?pa (status EXECUTION-FAILED) (error-msg ?error-msg))
  (printout warn "Lock " ?name " was rejected " crlf)
	(modify ?mf (request NONE) (response NONE) (error-msg ""))
  (modify ?li (status WAITING) (last-error ?error-msg))
)

(defrule lock-actions-lock-retry
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name lock) (status RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
  ?li <- (lock-info (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
          (status WAITING) (last-try $?last))
  (time $?now&:(timeout ?now ?last ?*LOCK-ACTION-RETRY-PERIOD-SEC*))
  =>
  (printout warn "Retrying to lock " ?name crlf)
  (mutex-try-lock-async ?name)
  (modify ?li (status REQUESTED) (last-try ?now))
)

(defrule lock-actions-lock-failed
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name lock) (status RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
  ?li <- (lock-info (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
          (status WAITING) (start-time $?start) (last-error ?error-msg))
  (time $?now&:(timeout ?now ?start ?*LOCK-ACTION-TIMEOUT-SEC*))
  =>
  (printout warn "Failed to get lock " ?name " in " ?*LOCK-ACTION-TIMEOUT-SEC*
    "s, giving up" crlf)
	(modify ?pa (status EXECUTION-FAILED) (error-msg ?error-msg))
  (retract ?li)
)

(defrule lock-actions-unlock-start
	?pa <- (plan-action (plan-id ?plan-id) (id ?id) (status PENDING)
                      (action-name unlock) (executable TRUE)
                      (param-names $?param-names)
                      (param-values $?param-values))
	=>
	(bind ?lock-name (plan-action-arg name ?param-names ?param-values))
	; The following performs a synchronous/blocking call
	;(bind ?rv (robmem-mutex-unlock (str-cat ?lock-name)))
	;(modify ?pa (status (if ?rv then EXECUTION-SUCCEEDED else EXECUTION-FAILED)))
	(mutex-unlock-async ?lock-name)
	(modify ?pa (status RUNNING))
)

(defrule lock-actions-unlock-done
	?pa <- (plan-action (plan-id ?plan-id) (id ?id)
                      (action-name unlock) (status RUNNING)
                      (param-names $?param-names) (param-values $?param-values))
	?mf <- (mutex (name ?name&:(eq ?name (plan-action-arg name ?param-names ?param-values)))
								(request UNLOCK) (response UNLOCKED))
	=>
	(modify ?pa (status EXECUTION-SUCCEEDED))
	(modify ?mf (request NONE) (response NONE))
)
