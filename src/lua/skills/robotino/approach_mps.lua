
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
   {v = "conveyor_0", type="Position3DInterface"}
}

documentation      = [==[
                        The robot just drives forward until a sensor threshold is reached
                        @param "x" int The x distance to the MPS when finished
                        @param "vision" boolean if true use the conveyor vision
                                                instead of the infrared sensor
                     ]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("tf_module")
local sensor_index = 0
local sensor_threshold = 0.07
local MIN_VIS_HIST = 10

if config:exists("/hardware/robotino/distance_front/index") then
   sensor_index = config:get_uint("/hardware/robotino/distance_front/index")
end

fsm:define_states{ export_to=_M, closure={sensor=sensor, sensor_index=sensor_index, conveyor_0=conveyor_0, MIN_VIS_HIST=MIN_VIS_HIST},
   {"INIT", JumpState},
   {"APPROACH_WITH_INFRARED", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"SETTLE", JumpState},
   {"APPROACH_WITH_CAM", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "SETTLE", cond="vars.vision == true"},
   {"INIT", "FAILED", cond="not conveyor_0:has_writer()", desc="Conveyor vision disabled"},
   {"INIT", "APPROACH_WITH_INFRARED", cond=true},
   {"SETTLE", "APPROACH_WITH_CAM", cond="conveyor_0:visibility_history() >= MIN_VIS_HIST"},
   {"SETTLE", "FAILED", timeout=1},
   {"APPROACH_WITH_INFRARED", "FINAL", cond="sensor:distance(sensor_index) <= vars.sensor_threshold and sensor:distance(sensor_index) > 0"}
}

function APPROACH_WITH_INFRARED:init()
   self.fsm.vars.sensor_threshold = self.fsm.vars.x or 0.07
   self.skills[1].x = 1
   self.skills[1].vel_trans = 0.05
end

function APPROACH_WITH_CAM:init()
   self.skills[1].x = tfm.transform({x=conveyor_0:translation(0),y=0,ori=0}, "/cam_conveyor", "/gripper").x
   self.skills[1].vel_trans = 0.05
   self.skills[1].TOLERANCE = { x=0.002, y=0.002, ori=0.01 }
end
