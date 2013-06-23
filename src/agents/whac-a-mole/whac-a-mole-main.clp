;---------------------------------------------------------------------------
;  main.clp - Whack-a-mole CLIPS Agent global CLIPS variables
;
;  Created: Sat Jun 22 16:09:36 2013
;  Copyright  2013  Frederik Zwilling
;  Licensed under BSD license, cf. LICENSE file
;---------------------------------------------------------------------------

(printout t "Starting whac-a-mole agent..." crlf)

(load* (resolve-file llsf2013/priorities.clp))
(load* (resolve-file llsf2013/globals.clp))
(load* (resolve-file whac-a-mole/whac-a-mole-facts.clp))

(defrule load-config
  (init)
  =>
  (load-config "/clips-agent")
  (printout t "hallo" crlf)
)

(deffacts init (init))

(defrule initialize
  (declare (salience ?*PRIORITY-HIGH*))
  (agent-init)
  (protobuf-available)
  =>
  (load-config "/clips-agent")

  (blackboard-add-interface "Position3DInterface" "nearest_light_on")
  (blackboard-add-interface "Position3DInterface" "Pose")

  (load* (resolve-file llsf2013/utils.clp))
  (load* (resolve-file llsf2013/net.clp))
  (load* (resolve-file llsf2013/game.clp))
  (batch* (resolve-file whac-a-mole/whac-a-mole.clp))

  (reset)
)


