;---------------------------------------------------------------------------
;  action-resources.clp - Add required resources to actions
;
;  Created:   Mon 14 Mar 13:25:41 CET 2022
;  Copyright  2022  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

(defrule action-resources-add-resources
	?action <- (plan-action (id ?action-id) (action-name ?action-name)
	                        (required-resources) (goal-id ?goal-id)
	                        (param-values $?params))
	(goal (id ?goal-id) (required-resources $?req-res))
 =>
	; These actions do not actually modify any resource, skip them.
	(bind ?ignore-actions
		(create$ enter-field go-wait location-lock location-unlock move move-node
		         send-beacon wait))
	(if (member$ ?action-name ?ignore-actions) then
		(return)
	)
	(bind ?action-res (create$))
	(foreach ?r ?req-res
		(bind ?actual-r ?r)
		(if (str-index "PROMISE-" (str-cat ?r)) then
			(bind ?actual-r (sym-cat (sub-string 9 (length$ ?r) (str-cat ?r))))
		)
		(if (member$ ?actual-r ?params) then
			(bind ?action-res (append$ ?action-res ?actual-r))
		)
	)
	; Check if we can construct an MPS side resource (e.g., C-CS1-INPUT) from the
	; parameters.
	(foreach ?param ?params
		(if (or (eq ?param INPUT) (eq ?param OUTPUT)) then
			(if (> ?param-index 1) then
				; Get the parameter directly preceeding ?param whether it is an MPS.
				(bind ?mps (nth$ (- ?param-index 1) ?params))
				(if (any-factp ((?wf wm-fact)) (eq (wm-key-arg ?wf:key m) ?mps)) then
					; It is an MPS, check whether the goal requires its INPUT/OUTPUT.
					(bind ?mps-res (sym-cat ?mps - ?param))
					(if (member$ ?mps-res ?req-res) then
						(bind ?action-res (append$ ?action-res ?mps-res))
	)))))
	(if (neq (length$ ?action-res) 0) then
		(printout t "Action " ?action-id " " ?action-name " of goal " ?goal-id
		            " requires the resources (" (implode$ ?action-res) ")" crlf)
		(modify ?action (required-resources ?action-res))
	)
)
