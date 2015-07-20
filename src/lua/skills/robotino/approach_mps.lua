
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
                        @param "offset_x" int The x offset of the conveyor belt (negative in robot direction)
                        @param "x" int The x distance to the MPS when finished
                     ]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")
local sensor_index = 0
local sensor_threshold = 0.055

if config:exists("/hardware/robotino/distance_front/index") then
   sensor_index = config:get_uint("/hardware/robotino/distance_front/index")
end

fsm:define_states{ export_to=_M, closure={sensor=sensor, sensor_index=sensor_index},
   {"INIT", JumpState},
   {"APPROACH_WITH_INFRARED", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"CHECK", JumpState}, -- if the offset is negative drive the last bit with motor move because the sensor stop at 5mm
   {"MOTOR_MOVE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "APPROACH_WITH_INFRARED", cond=true},
   {"APPROACH_WITH_INFRARED", "CHECK", cond="sensor:distance(sensor_index) <= vars.sensor_threshold and sensor:distance(sensor_index) > 0"},
   {"CHECK", "MOTOR_MOVE", cond="vars.offset_x < 0"},
   {"CHECK", "FINAL", cond=true},
}

function INIT:init()
   if self.fsm.vars.offset_x == nil then --for backwards compatibility
      self.fsm.vars.offset_x = 0
   end
end

function APPROACH_WITH_INFRARED:init()
   printf("x_offset is set to: %f", self.fsm.vars.offset_x)
   if self.fsm.vars.offset_x >= 0 then
      self.fsm.vars.sensor_threshold = self.fsm.vars.x or sensor_threshold + self.fsm.vars.offset_x 
   else
      self.fsm.vars.sensor_threshold = sensor_threshold -- the threshold is negative so we adjust the rest in the upcoming motor_move state
   end
   --TODO !!!!!!!handle negative with motor_move and further driving when putting a product!!!!!!!!!!!!!!!!
   printf("sensor threshold is: %f", self.fsm.vars.sensor_threshold)
   self.skills[1].x = 1
   self.skills[1].vel_trans = 0.05
end

function MOTOR_MOVE:init()
   self.skills[1].x = -self.fsm.vars.offset_x --with the above condition this is negative so invert it
   self.skills[1].vel_trans = 0.03
   self.skills[1].tolerance = { x=0.001, y=0.002, ori=0.01 }
end
