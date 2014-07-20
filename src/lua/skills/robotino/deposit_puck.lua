
----------------------------------------------------------------------------
--  goto.lua - generic global goto
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
name               = "deposit_puck"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = {
   {v = "sensor", type="RobotinoSensorInterface", id = "Robotino"},
}

documentation      = [==[deposits the used puck at the side of the traffic light]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local RIGHT_IR_ID = config:get_float("hardware/robotino/sensors/right_ir_id")
local LEFT_IR_ID  = config:get_float("hardware/robotino/sensors/left_ir_id")

function right_sensor_free()
   return sensor:analog_in(RIGHT_IR_ID) < 9
end

function left_sensor_free()
   return sensor:analog_in(LEFT_IR_ID) < 9
end

function deposit_right()
   return navgraph:node(fsm.vars.place):has_property("leave_right")
end

fsm:define_states{export_to=_M, closure={ no_puck=no_puck, navgraph=navgraph},
   {"INIT", JumpState},
   {"SKILL_DRIVE_LEFT", SkillJumpState, skills={{motor_move}}, final_to="SKILL_DRIVE_FORWARD",
      fail_to="FAILED"},
   {"SKILL_DRIVE_RIGHT", SkillJumpState, skills={{motor_move}}, final_to="SKILL_DRIVE_FORWARD",
      fail_to="FAILED"},
   {"SKILL_DRIVE_FORWARD", SkillJumpState, skills={{motor_move}}, final_to="SKILL_DRIVE_BACKWARD",
      fail_to="FAILED"},
   {"SKILL_DRIVE_BACKWARD", SkillJumpState, skills={{motor_move}}, final_to="FINAL",
      fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "SKILL_DRIVE_RIGHT", cond=deposit_right},
   {"INIT", "SKILL_DRIVE_LEFT", cond=true, desc="delivery"},
   {"SKILL_DRIVE_LEFT", "SKILL_DRIVE_FORWARD", cond=right_sensor_free},
   {"SKILL_DRIVE_RIGHT", "SKILL_DRIVE_FORWARD", cond=left_sensor_free},
}

function SKILL_DRIVE_LEFT:init()
   self.skills[1].y = 0.25
   self.skills[1].vel_trans = 0.2
end

function SKILL_DRIVE_RIGHT:init()
   self.skills[1].y = -0.25
   self.skills[1].vel_trans = 0.2
end

function SKILL_DRIVE_FORWARD:init()
   if self.fsm.vars.mtype == nil then
      self.skills[1].x = 0.15
   elseif self.fsm.vars.mtype == "deliver" then
      print("drive forward at delivery")
      self.skills[1].x = 0.08
   end 
end

function SKILL_DRIVE_BACKWARD:init()
   self.skills[1].x=-0.25 
end

