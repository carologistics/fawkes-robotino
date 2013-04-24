
----------------------------------------------------------------------------
--  move_under_rfid.lua
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
name               = "move_under_rfid"
fsm                = SkillHSM:new{name=name, start="SEE_AMPEL", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = {
	{v = "sensor", type="RobotinoSensorInterface", id = "Robotino"},
	{v = "euclidean_cluster", type="Position3DInterface", id = "Euclidean Laser Cluster"},
	{v = "motor", type = "MotorInterface", id="Robotino" },	
   {v = "pose", type="Position3DInterface", id="Pose"}
}

documentation      = [==[Move under the RFID Reader/Writer]==]

local tfm = require("tf_module")

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require('tf_module')
local LASER_SIDEWARDS_CORRECTION = 0.1
local LASER_FORWARD_CORRECTION = 0.25
local LIGHT_SENSOR_DELAY_CORRECTION = 0.045

function ampel()
   return (fsm.vars.ampel_loc.distance > 0) and (fsm.vars.ampel_loc.distance < 1)
end

function rough_correct_done()
   if fsm.vars.correct_dir == -1 then
      return sensor:analog_in(4) < 1
   else
       return sensor:analog_in(0) < 1
   end
end

fsm:define_states{ export_to=_M, closure={ampel=ampel, sensor=sensor},
   {"SEE_AMPEL", JumpState},
   {"SKILL_TURN_ZERO",SkillJumpState, skills={{motor_move}}, final_to="STRAFE", fail_to="FAILED"},
   {"STRAFE", SkillJumpState, skills={{motor_move}}, final_to="APPROACH_AMPEL",
      fail_to="FAILED"},
   {"APPROACH_AMPEL", SkillJumpState, skills={{motor_move}},
      final_to="CHECK_POSITION", fail_to="FAILED"},
   {"CHECK_POSITION", JumpState},
   {"CORRECT_POSITION", SkillJumpState, skills={{motor_move}}, final_to="CORRECT_SENSOR_DELAY", fail_to="FAILED"},
   {"CORRECT_SENSOR_DELAY", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"SEE_AMPEL", "FAILED", cond="not ampel()", desc="No Ampel seen with laser"},
   {"SEE_AMPEL", "SKILL_TURN_ZERO", cond=ampel, desc="Ampel seen with laser"},
   {"CHECK_POSITION", "FINAL", cond="vars.correct_dir == 0"},
   {"CHECK_POSITION", "CORRECT_POSITION", cond="vars.correct_dir ~= 0"},
   {"CORRECT_POSITION", "CORRECT_SENSOR_DELAY", cond=rough_correct_done} 
}

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function CHECK_POSITION:init()
   if sensor:analog_in(0) < 1 then
      self.fsm.vars.correct_dir = -1
   elseif sensor:analog_in(4) < 1 then
      self.fsm.vars.correct_dir = 1
   else
      self.fsm.vars.correct_dir = 0
   end
end

function SEE_AMPEL:init()
   self.fsm.vars.ampel_loc = {}
   self.fsm.vars.ampel_loc.x = euclidean_cluster:translation(0)
   self.fsm.vars.ampel_loc.y = euclidean_cluster:translation(1)
   self.fsm.vars.ampel_loc.distance = math.sqrt(self.fsm.vars.ampel_loc.x^2,self.fsm.vars.ampel_loc.y^2)
   self.fsm.vars.ampel_loc.angle = math.atan(self.fsm.vars.ampel_loc.x,self.fsm.vars.ampel_loc.y)
   self.fsm.vars.ampel_map = tfm.transform({x=self.fsm.vars.ampel_loc.x, y=self.fsm.vars.ampel_loc.y, ori=self.fsm.vars.ampel_loc.angle}, "/base_link", "/map")
end

function SKILL_TURN_ZERO:init()
   local self_angle = 2*math.acos(pose:rotation(3))
   local floor_angle = math.floor(self_angle/(math.pi/2.0)) * (math.pi/2.0)
                                  -- round     (                                        )
   local turn_angle = floor_angle + math.floor(((self_angle - floor_angle)/(math.pi/2.0)) + 0.5) * (math.pi/2.0)
   local tgt_baselink = tfm.transform({x=0, y=0, ori=turn_angle}, "/map", "/base_link")
   self.skills[1].ori = tgt_baselink.ori
end

function STRAFE:init()
   local ampel_baselink = tfm.transform(self.fsm.vars.ampel_map, "/map", "/base_link")
   if ampel_baselink.y < 0 then
      corr = -1 * LASER_SIDEWARDS_CORRECTION
   else
      corr = LASER_SIDEWARDS_CORRECTION
   end
   self.skills[1].y = ampel_baselink.y + corr
end

function APPROACH_AMPEL:init()
   self.skills[1].x = self.fsm.vars.ampel_loc.x - LASER_FORWARD_CORRECTION
end

function CORRECT_POSITION:init()
   self.skills[1].y = self.fsm.vars.correct_dir * 0.3
end

function CORRECT_SENSOR_DELAY:init()
   self.skills[1].y = self.fsm.vars.correct_dir * -1 * LIGHT_SENSOR_DELAY_CORRECTION 
end

