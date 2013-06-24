
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

-- function get_ori_diff()
--    local ori = 2*math.acos(pose:rotation(3)) 
--    local is_ori = m_pos.delivery_goto.Is.ori
--    local diff = 0
--    if ori > is_ori then 
--       if ori - is_ori < math.pi then
--          diff =  ori - is_ori 
--       else
--          diff =  - 2.0 * math.pi + ori - is_ori
--       end
--    else
--       if is_ori - ori < math.pi then
--          diff = ori - is_ori 
--       else
--          diff = 2.0 * math.pi - is_ori + ori;
--       end
--    end
--    return diff
-- end
-- function oriented_right()
--    local q = fawkes.tf.Quaternion:new(pose:rotation(0),pose:rotation(1),pose:rotation(2),pose:rotation(3))
--    local ori = fawkes.tf.get_yaw(q)
--    print(ori)
--    if  ori >= 0 and ori <= math.pi then
--       return true
--    end
-- end
function turned_to_zero()
   local q = fawkes.tf.Quaternion:new(pose:rotation(0),pose:rotation(1),pose:rotation(2),pose:rotation(3))
   local ori = fawkes.tf.get_yaw(q)
   if ori <= 0.05 and ori >= -0.05 then
      return true
   end
end

fsm:define_states{ export_to=_M,
   {"TURN", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"STOP", SkillJumpState, skills={{motor_move}},final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"TURN", "STOP", cond = turned_to_zero}
}
function TURN:init()
   local q = fawkes.tf.Quaternion:new(pose:rotation(0),pose:rotation(1),pose:rotation(2),pose:rotation(3))
   local ori = fawkes.tf.get_yaw(q)
   if ori < 0 then
      self.skills[1].ori = 2*math.pi - math.abs(ori)
   else
      self.skills[1].ori = ori
   end
   self.skills[1].vel_rot = 1.1
end
function STOP:init()
   self.skills[1].x=0
   self.skills[1].y=0
   self.skills[1].ori=0
end
