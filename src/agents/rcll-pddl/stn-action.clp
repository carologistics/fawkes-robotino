;---------------------------------------------------------------------------
;  stn-action.clp - abstract task execution engine
;
;  Created: Tue 06 Jun 2017 12:42:48 CEST
;  Copyright  2017 Matthias Loebach
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------
; All tasks are proposed first in production.clp
; Task execution is decided in cooperation.clp
; Here the task is executed by switching through the steps of a task
; The steps themselves are executed/finished/aborted in steps.clp
;---------------------------------------------------------------------------

(defrule stn-action-finished
  (phase PRODUCTION)
  (stn-action (id ?id) (state finished))
  ?sa <- (stn-action (cond-actions $?cond&:(member$ ?id ?cond)))
  =>
  (synced-remove-from-multifield ?sa cond-actions ?id)
)

(defrule stn-action-lock-wait-cleanup
  (phase PRODUCTION)
  (not (wait-for-lock (res ?to)))
  ?lock <- (action-lock-wait ?id)
  =>
  (retract ?lock)
)
