
----------------------------------------------------------------------------
--  robotino.lua - Robotini skill space initialization
--
--  Created: Wed May 30 11:51:30 2012
--  Copyright  2012  Tim Niemueller [www.niemueller.de]
----------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

require("fawkes.modinit")
module(..., fawkes.modinit.register_all);

print("Initializing Lua skill space for Robotino")

-- Generic skills
skillenv.use_skill("skills.generic.say")

-- Robotino specific skills
skillenv.use_skill("skills.robotino.relgoto")
skillenv.use_skill("skills.robotino.motor_move")
skillenv.use_skill("skills.robotino.global_motor_move")
skillenv.use_skill("skills.robotino.goto")
skillenv.use_skill("skills.robotino.ppgoto")
skillenv.use_skill("skills.robotino.ppgoto_waypoints")
skillenv.use_skill("skills.robotino.goto_waypoints")
skillenv.use_skill("skills.robotino.drive_into_field")
skillenv.use_skill("skills.robotino.drive_test")
skillenv.use_skill("skills.robotino.drive_to")
skillenv.use_skill("skills.robotino.enable_switch")
skillenv.use_skill("skills.robotino.explore_zone")
skillenv.use_skill("skills.robotino.gripper_commands")
skillenv.use_skill("skills.robotino.reset_gripper")
skillenv.use_skill("skills.robotino.detect_object")
skillenv.use_skill("skills.robotino.workpiece_pose")
skillenv.use_skill("skills.robotino.yolo_shelf_pick")
-- Skills for MPS interaction
skillenv.use_skill("skills.robotino.check_tag")
skillenv.use_skill("skills.robotino.mps_align")
skillenv.use_skill("skills.robotino.conveyor_align")
skillenv.use_skill("skills.robotino.approach_mps")
skillenv.use_skill("skills.robotino.shelf_pick")
skillenv.use_skill("skills.robotino.product_pick")
skillenv.use_skill("skills.robotino.product_put")
skillenv.use_skill("skills.robotino.drive_to_machine_point")
skillenv.use_skill("skills.robotino.bring_product_to")
skillenv.use_skill("skills.robotino.get_product_from")
skillenv.use_skill("skills.robotino.approach_test")
skillenv.use_skill("skills.robotino.discard")

-- ProtoBuf communication
skillenv.use_skill("skills.robotino.create_peer")
