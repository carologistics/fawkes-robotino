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
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_1"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_2"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_3"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_4"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_5"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_6"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_7"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_8"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_9"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_10"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_11"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_12"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_13"))
  (blackboard-open "BoxInterface" (remote-if-id ?robot "box_detect_14"))
  (blackboard-open "SwitchInterface" (remote-if-id ?robot "switch/box_detect_enabled"))
)

(defrule blackboard-init-open-laptop-navgraph-interfaces
  "Open the robot-specific Navgraph blackboard interfaces."
  (domain-facts-loaded)
  (ff-feature-loaded blackboard)
  (wm-fact (key central agent laptop args? r ?robot))
  =>
  (blackboard-open "NavGraphWithMPSGeneratorInterface"
                   (remote-if-id ?robot "navgraph-generator-mps"))
  (blackboard-open "NavGraphGeneratorInterface"
                   (remote-if-id ?robot "navgraph-generator"))
  (blackboard-open "NavigatorInterface"
                   (remote-if-id ?robot "Navigator"))
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

(defrule blackboard-init-compute-navgraph
  (or (wm-fact (key central agent robot args? r ?robot))
      (wm-fact (key central agent laptop args? r ?robot)))
  (blackboard-interface (id ?id&:(str-index ?robot ?id))
                        (type "NavGraphWithMPSGeneratorInterface"))
  (blackboard-interface (id ?id2&:(str-index ?robot ?id2))
                        (type "NavGraphGeneratorInterface"))
  (wm-fact (key config rcll use-static-navgraph) (type BOOL) (value FALSE))
  =>
  (navgraph-set-field-size ?robot)
  (navgraph-compute ?robot)
)

(defrule blackboard-init-compute-navgraph-from-refbox-value
  (or (wm-fact (key central agent robot args? r ?robot))
      (wm-fact (key central agent laptop args? r ?robot)))
  (blackboard-interface (id ?id&:(str-index ?robot ?id))
                        (type "NavGraphWithMPSGeneratorInterface"))
  (blackboard-interface (id ?id2&:(str-index ?robot ?id2))
                        (type "NavGraphGeneratorInterface"))
  (wm-fact (key config rcll use-static-navgraph) (type BOOL) (value FALSE))
  (wm-fact (key refbox field height) (value ?field-height))
  (wm-fact (key refbox field width) (value ?field-width))
  (wm-fact (key refbox field mirrored) (value ?mirrored))
  (test
    (and
      (neq ?field-height NOT-SET) (neq ?field-height DOES-NOT-EXIST)
      (neq ?field-width NOT-SET) (neq ?field-width DOES-NOT-EXIST)
      (neq ?mirrored NOT-SET) (neq ?mirrored DOES-NOT-EXIST)
    )
  )
  =>
  (bind ?p1_x (- 0 ?field-width))
  (bind ?p1_y 0)
  (bind ?p2_x ?field-width)
  (bind ?p2_y ?field-height)
  (if ?mirrored
    then
    (bind ?p2_x 0)
  )
  (navgraph-set-field-size-freely ?robot ?p1_x ?p1_y ?p2_x ?p2_y)
  (navgraph-compute ?robot)
)

(defrule blackboard-init-compute-navgraph-central
  (blackboard-interface (id "/navgraph-generator-mps")
                        (type "NavGraphWithMPSGeneratorInterface"))
  (blackboard-interface (id "/navgraph-generator")
                        (type "NavGraphGeneratorInterface"))
  =>
  (navgraph-compute FALSE)
)

(defrule blackboard-init-compute-navgraph-central-from-refbox-value
  (blackboard-interface (id "/navgraph-generator-mps")
                        (type "NavGraphWithMPSGeneratorInterface"))
  (blackboard-interface (id "/navgraph-generator")
                        (type "NavGraphGeneratorInterface"))
  (wm-fact (key refbox field height) (value ?field-height))
  (wm-fact (key refbox field width) (value ?field-width))
  (wm-fact (key refbox field mirrored) (value ?mirrored))
  (test
    (and
      (neq ?field-height NOT-SET) (neq ?field-height DOES-NOT-EXIST)
      (neq ?field-width NOT-SET) (neq ?field-width DOES-NOT-EXIST)
      (neq ?mirrored NOT-SET) (neq ?mirrored DOES-NOT-EXIST)
    )
  )
  =>
  (bind ?p1_x (- 0 ?field-width))
  (bind ?p1_y 0)
  (bind ?p2_x ?field-width)
  (bind ?p2_y ?field-height)
  (if ?mirrored
    then
    (bind ?p2_x 0)
  )
  (bind ?interface "NavGraphGeneratorInterface::navgraph-generator")
  (bind ?msg (blackboard-create-msg ?interface "SetBoundingBoxMessage"))
  (blackboard-set-msg-field ?msg "p1_x" ?p1_x)
  (blackboard-set-msg-field ?msg "p1_y" ?p1_y)
  (blackboard-set-msg-field ?msg "p2_x" ?p2_x)
  (blackboard-set-msg-field ?msg "p2_y" ?p2_y)
  (bind ?msg (blackboard-create-msg ?interface "SetBoundingBoxMessage"))
  (blackboard-send-msg ?msg)
  (navgraph-compute FALSE)
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
  "Open the Navgraph blackboard interfaces ."
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
