
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

-- Robotino specific skills
skillenv.use_skill("skills.robotino.grab_puck")
skillenv.use_skill("skills.robotino.motor_move")
skillenv.use_skill("skills.robotino.relgoto")
skillenv.use_skill("skills.generic.ppgoto")
skillenv.use_skill("skills.robotino.goto")
skillenv.use_skill("skills.robotino.fetch_puck")
skillenv.use_skill("skills.robotino.take_puck_to")
skillenv.use_skill("skills.robotino.move_under_rfid")
skillenv.use_skill("skills.robotino.determine_signal")
skillenv.use_skill("skills.robotino.leave_area")
skillenv.use_skill("skills.robotino.deliver_puck")
skillenv.use_skill("skills.robotino.deposit_puck")
skillenv.use_skill("skills.robotino.leave_area_with_puck")
skillenv.use_skill("skills.robotino.leave_IS")
skillenv.use_skill("skills.robotino.get_s0")
skillenv.use_skill("skills.robotino.move_to_next_deliver")
skillenv.use_skill("skills.robotino.finish_puck_at")

