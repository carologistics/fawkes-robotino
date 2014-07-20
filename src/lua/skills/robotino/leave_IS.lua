
----------------------------------------------------------------------------
--  leave_area_with_puck.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
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

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "leave_IS"
fsm                = SkillHSM:new{name=name, start="TURN", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = {
   {v="pose", type="Position3DInterface",id = "Pose"}
}

documentation      = [==[Leaves area with puck by driving left and rotating]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local m_pos = require("machine_pos_module")


fsm:define_states{ export_to=_M,
   {"TURN", SkillJumpState, skills={{motor_move}}, final_to="SKILL_MOTOR_MOVE", fail_to="FAILED"},
   {"SKILL_MOTOR_MOVE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

function TURN:init()
   local q = fawkes.tf.Quaternion:new(pose:rotation(0),pose:rotation(1),pose:rotation(2),pose:rotation(3))
   local ori = fawkes.tf.get_yaw(q)
   if     self.fsm.vars.turn_direction == "left" then
      -- dann drehen wir auf 0° plus 20° drüber (nach links)
      self.skills[1].ori = math.abs(ori) + 0.35
   elseif self.fsm.vars.turn_direction == "right" then
      -- dann drehen wir auf 160° (nach rechts)
      self.skills[1].ori = math.abs(ori) - 3.49
   end
   self.skills[1].vel_rot = 1.1

   printf("ori=%f  skill_ori=%f", ori, self.skills[1].ori)
end

function SKILL_MOTOR_MOVE:init()
   self.skills[1].x = 0.4
end
