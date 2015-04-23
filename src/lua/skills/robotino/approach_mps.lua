
----------------------------------------------------------------------------
--  approach_mps.lua
--
--  Created Wed Apr 15
--  Copyright  2015  Johannes Rothe
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
name               = "approach_mps"
fsm                = SkillHSM:new{name=name, start="APPROACH", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = { 
   {v = "sensor", type="RobotinoSensorInterface"}
}

documentation      = [==[
                        The robot just drives forward until a sensor threshold is reached
                     ]==]


-- Initialize as skill module
skillenv.skill_module(_M)

local sensor_index = 0
local sensor_threshold = 0.085

fsm:define_states{ export_to=_M, closure={sensor=sensor, sensor_index=sensor_index, sensor_threshold=sensor_threshold},
   {"APPROACH", SkillJumpState, skills={{motor_move}},
      final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"APPROACH", "FINAL", cond="sensor:distance(sensor_index) <= sensor_threshold and sensor:distance(sensor_index) > 0"}
}

function APPROACH:init()
   self.skills[1].x = 1
   self.skills[1].vel_trans = 0.1
end
