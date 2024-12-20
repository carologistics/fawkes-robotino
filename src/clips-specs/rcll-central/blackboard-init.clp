;---------------------------------------------------------------------------
;  util.clp - utilities needed for executive rcll agent
;
;  Created: Tue 19 Apr 2018 17:03:31 CET
;  Copyright  2018  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>, Daniel Habering
;  Licensed under GPLv2+ license, cf. LICENSE file in the doc directory.
;---------------------------------------------------------------------------

; This program is free software; you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation; either version 2 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU Library General Public License for more details.
;
; Read the full text in the LICENSE.GPL file in the doc directory.

(defrule blackboard-init-open-robot-pose-interface
  "Open the Pose blackboard interface to get the robot pose."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  (wm-fact (key central agent robot args? r ?robot))
  =>
  (blackboard-open-reading "Position3DInterface" (remote-if-id ?robot "Pose"))
)

(defrule blackboard-init-open-central-navgraph-interfaces
  "Open the central Navgraph blackboard interfaces."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  =>
  (blackboard-open "NavGraphWithMPSGeneratorInterface" "/navgraph-generator-mps")
  (blackboard-open "NavGraphGeneratorInterface" "/navgraph-generator")
)

(defrule blackboard-init-open-robot-exploration-interfaces
  "Open the central Navgraph blackboard interfaces."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  (wm-fact (key central agent robot args? r ?robot))
  =>
  (blackboard-open "MotorInterface"     (remote-if-id ?robot "Robotino"))
  (blackboard-open "Position3DInterface"(remote-if-id ?robot "Pose"))
  (blackboard-open "ExplorationInterface" (remote-if-id ?robot "exploration/info"))
)

(defrule blackboard-init-open-liveliness-check
  "Open the robot-specific Heartbeat blackboard interface."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  (wm-fact (key central agent robot args? r ?robot))
  =>
  (blackboard-open "HeartbeatInterface" (remote-if-id "heartbeat" ?robot))
)

(defrule blackboard-init-open-robot-navgraph-interfaces
  "Open the robot-specific Navgraph blackboard interfaces."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  (wm-fact (key central agent robot args? r ?robot))
  =>
  (blackboard-open "NavGraphWithMPSGeneratorInterface"
                   (remote-if-id ?robot "navgraph-generator-mps"))
  (blackboard-open "NavGraphGeneratorInterface"
                   (remote-if-id ?robot "navgraph-generator"))
  (blackboard-open "NavigatorInterface"
                   (remote-if-id ?robot "Navigator"))
)

(defrule blackboard-init-compute-navgraph-robot
	(wm-fact (key central agent robot args? r ?robot))
	(blackboard-interface (id ?id&:(str-index ?robot ?id))
	                      (type "NavGraphWithMPSGeneratorInterface"))
	(blackboard-interface (id ?id2&:(str-index ?robot ?id2))
	                      (type "NavGraphGeneratorInterface"))
 	=>
	(navgraph-init ?robot FALSE)
)

(defrule blackboard-init-compute-navgraph-central
	(blackboard-interface (id "/navgraph-generator-mps")
	                      (type "NavGraphWithMPSGeneratorInterface"))
	(blackboard-interface (id "/navgraph-generator")
	                    (type "NavGraphGeneratorInterface"))
  (wm-fact (key refbox version-info received))
	=>
	(navgraph-init central FALSE)
)

(defrule blackboard-init-open-skiller-interface
  "Open the skiller interface for a remote robot."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  (ff-feature-loaded skills)
  (wm-fact (key central agent robot args? r ?robot))
  (not (skiller-control (skiller ?skiller&:(eq ?skiller (remote-if-id ?robot "Skiller")))))
  =>
  (blackboard-open "SkillerInterface"
                   (remote-if-id ?robot "Skiller"))
  (assert (skiller-control (skiller (remote-if-id ?robot "Skiller"))))
)

(defrule blackboard-init-motor-interfaces
  "Open the motor interfaces of the robotinos."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  (wm-fact (key central agent robot args? r ?robot))
  =>
  (blackboard-open "MotorInterface" (remote-if-id ?robot "Robotino"))
)

; TODO: TagVisionInterface? LaserLineInterfaces
(defrule blackboard-init-unwatch
  "Unwatch IF related facts and rules."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  =>
  (unwatch rules blackboard-read)
  (unwatch facts Position3DInterface)
  (unwatch facts NavGraphWithMPSGeneratorInterface)
  (unwatch facts MotorInterface)
  (unwatch facts NavigatorInterface)
)
