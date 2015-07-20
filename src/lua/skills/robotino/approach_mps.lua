
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = { 
   {v = "sensor", type="RobotinoSensorInterface"},
}

documentation      = [==[
                        The robot just drives forward until a sensor threshold is reached
                        @param "offset_x" int The x offset of the conveyor belt (positive in robot direction)
                        @param "x" int The x distance to the MPS when finished
                     ]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")
local sensor_index = 0
local sensor_threshold = 0.06

if config:exists("/hardware/robotino/distance_front/index") then
   sensor_index = config:get_uint("/hardware/robotino/distance_front/index")
end

fsm:define_states{ export_to=_M, closure={sensor=sensor, sensor_index=sensor_index},
   {"INIT", JumpState},
   {"APPROACH_WITH_INFRARED", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "APPROACH_WITH_INFRARED", cond=true},
   {"APPROACH_WITH_INFRARED", "FINAL", cond="sensor:distance(sensor_index) <= vars.sensor_threshold and sensor:distance(sensor_index) > 0"}
}

function APPROACH_WITH_INFRARED:init()
   if self.fsm.vars.offset_x == nil then
      self.fsm.vars.offset_x = 0
   end
   printf("x_offset is set to: %f", self.fsm.vars.offset_x)
   self.fsm.vars.sensor_threshold = self.fsm.vars.x or sensor_threshold + self.fsm.vars.offset_x 
   --TODO !!!!!!!handle negative with motor_move and further driving when putting a product!!!!!!!!!!!!!!!!
   printf("sensor threshold is: %f", self.fsm.vars.sensor_threshold)
   self.skills[1].x = 1
   self.skills[1].vel_trans = 0.05
end
