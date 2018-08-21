
----------------------------------------------------------------------------
--  robotino.lua - RCLL2016 skill space initialization
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

print("Initializing Lua skill space for RCLL2016")

-- Robotino specific skills
skillenv.use_skill("skills.rcll2016.relgoto")
skillenv.use_skill("skills.rcll2016.motor_move")
skillenv.use_skill("skills.rcll2016.global_motor_move")
skillenv.use_skill("skills.rcll2016.goto")
skillenv.use_skill("skills.rcll2016.ppgoto")
skillenv.use_skill("skills.rcll2016.ppgoto_waypoints")
skillenv.use_skill("skills.rcll2016.goto_waypoints")
skillenv.use_skill("skills.rcll2016.drive_into_field")
skillenv.use_skill("skills.rcll2016.drive_test")
skillenv.use_skill("skills.rcll2016.align_laserlines")
skillenv.use_skill("skills.rcll2016.global_move_laserlines")
skillenv.use_skill("skills.rcll2016.drive_to")
skillenv.use_skill("skills.rcll2016.drive_to_global")
skillenv.use_skill("skills.rcll2016.drive_to_local")
skillenv.use_skill("skills.rcll2016.enable_switch")
skillenv.use_skill("skills.rcll2016.gripper_z_align")
skillenv.use_skill("skills.rcll2016.ax12gripper")
skillenv.use_skill("skills.rcll2016.align_tag")
skillenv.use_skill("skills.rcll2016.explore_zone")

-- Skills for MPS interaction
skillenv.use_skill("skills.rcll2016.check_tag")
skillenv.use_skill("skills.rcll2016.mps_align")
--skillenv.use_skill("skills.rcll2016.mps_detect_signal")
skillenv.use_skill("skills.rcll2016.conveyor_align")
skillenv.use_skill("skills.rcll2016.approach_mps")
skillenv.use_skill("skills.rcll2016.shelf_pick")
skillenv.use_skill("skills.rcll2016.slide_put")
skillenv.use_skill("skills.rcll2016.shelf_put")
skillenv.use_skill("skills.rcll2016.product_pick")
skillenv.use_skill("skills.rcll2016.product_put")
skillenv.use_skill("skills.rcll2016.bring_product_to")
skillenv.use_skill("skills.rcll2016.get_product_from")
skillenv.use_skill("skills.rcll2016.approach_test")
