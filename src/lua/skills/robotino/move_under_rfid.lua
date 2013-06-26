
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
   {v = "pose", type="Position3DInterface", id="Pose"},
   {v = "laserswitch", type="SwitchInterface", id="laser-cluster" },
   {v = "laser_cluster", type="LaserClusterInterface", id="laser-cluster" }
}

documentation      = [==[Move under the RFID Reader/Writer]==]

local tfm = require("tf_module")

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require('tf_module')
local LASER_FORWARD_CORRECTION = 0.2
local LIGHT_SENSOR_DELAY_CORRECTION = 0.045
local MIN_VIS_HIST = 15

function get_ampel()
   local ampel_loc = {}
   ampel_loc.x = euclidean_cluster:translation(0)
   ampel_loc.y = euclidean_cluster:translation(1)
   ampel_loc.distance = math.sqrt(ampel_loc.x^2, ampel_loc.y^2)
   ampel_loc.angle = math.atan(ampel_loc.x, ampel_loc.y)
   return ampel_loc
end

function ampel()
   local ampel = get_ampel()
   return (ampel.distance > 0)
      and (ampel.distance < 1)
      and (euclidean_cluster:visibility_history() > MIN_VIS_HIST)
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
   {"TURN", SkillJumpState, skills={{motor_move}}, final_to="APPROACH_AMPEL", fail_to="FAILED"},
   {"APPROACH_AMPEL", SkillJumpState, skills={{motor_move}},
      final_to="CHECK_POSITION", fail_to="FAILED"},
   {"CHECK_POSITION", JumpState},
   {"CORRECT_POSITION", SkillJumpState, skills={{motor_move}},
      final_to="CORRECT_SENSOR_DELAY", fail_to="FAILED"},
   {"CORRECT_SENSOR_DELAY", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"SEE_AMPEL", "FAILED", timeout=10, desc="No Ampel seen with laser"},
   {"SEE_AMPEL", "TURN", cond=ampel, desc="Ampel seen with laser"},
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
   laserswitch:msgq_enqueue_copy(laserswitch.EnableSwitchMessage:new())
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.0))
end

function TURN:init()
   local ampel = get_ampel()
   self.skills[1].ori = math.atan2(ampel.y, ampel.x)
end

function APPROACH_AMPEL:init()
   local ampel = get_ampel()
   self.skills[1].x = ampel.x - LASER_FORWARD_CORRECTION
   self.skills[1].y = ampel.y
end

function CORRECT_POSITION:init()
   self.skills[1].y = self.fsm.vars.correct_dir * 0.3
   self.skills[1].vel_trans = 0.05
end

function CORRECT_SENSOR_DELAY:init()
   self.skills[1].y = self.fsm.vars.correct_dir * -1 * LIGHT_SENSOR_DELAY_CORRECTION 
end

function FINAL:init()
   laserswitch:msgq_enqueue_copy(laserswitch.DisableSwitchMessage:new())
end
