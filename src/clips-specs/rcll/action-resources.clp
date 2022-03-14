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
		(if (member$ ?r ?params) then
			(bind ?action-res (append$ ?action-res ?r))
		)
	)
	(if (neq (length$ ?action-res) 0) then
		(printout t "Action " ?action-id " " ?action-name " of goal " ?goal-id
		            " requires the resources (" (implode$ ?action-res) ")" crlf)
		(modify ?action (required-resources ?action-res))
	)
)
