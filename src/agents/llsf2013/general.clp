;---------------------------------------------------------------------------
;  general.clp - General rules for the agent
;
;  Created: Mon May 13 19:32:13 2013 (Aachen)
;  Copyright  2013  Frederik Zwilling
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------


;Determine the role of the agent
(defrule general-determine-role
  (confval (path "/clips-agent/llsf2013/agent-role") (value ?role))
  =>
  (assert (role (sym-cat ?role)))
  (printout t "I have the role " (str-cat ?role) crlf)
)
