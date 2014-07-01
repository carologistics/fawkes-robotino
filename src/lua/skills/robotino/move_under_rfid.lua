
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
   {v = "laser_cluster", type="LaserClusterInterface", id="laser-cluster" },
   {v = "light", type ="RobotinoLightInterface", id = "Light_State" },
   {v = "lightswitch", type ="SwitchInterface", id = "light_front_switch" }
}

documentation      = [==[Move under the RFID Reader/Writer]==]

local tfm = require("tf_module")

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require('tf_module')
local LASER_FORWARD_CORRECTION = 0.17
local LIGHT_SENSOR_DELAY_CORRECTION = 0.045
local MIN_VIS_HIST = 15
local LEFT_IR_ID = config:get_float("hardware/robotino/sensors/left_ir_id")
local RIGHT_IR_ID = config:get_float("hardware/robotino/sensors/right_ir_id")

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
   --analog_in(4) links
   --analog_in(5) rechts
   if fsm.vars.correct_dir == -1 then
      return sensor:analog_in(LEFT_IR_ID) > 9
   else
       return sensor:analog_in(RIGHT_IR_ID) > 9
   end
end

function producing()
   return light:green() == light.ON
      and light:yellow() == light.ON
      and light:red() == light.OFF
end

function sensors_fired()
   return sensor:analog_in(4) > 9 or sensor:analog_in(5) > 9
end

fsm:define_states{ export_to=_M, closure={ampel=ampel, sensor=sensor},
   {"SEE_AMPEL", JumpState},
   {"APPROACH_AMPEL", SkillJumpState, skills={{motor_move}},
      final_to="CHECK_POSITION", fail_to="FAILED"},
   {"CHECK_POSITION", JumpState},
   {"CORRECT_POSITION", SkillJumpState, skills={{motor_move}},
      final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"SEE_AMPEL", "FAILED", timeout=10, desc="No Ampel seen with laser"},
   {"SEE_AMPEL", "APPROACH_AMPEL", cond=ampel, desc="Ampel seen with laser"},
   {"CHECK_POSITION", "FINAL", cond="vars.correct_dir == 0"},
   {"CHECK_POSITION", "CORRECT_POSITION", cond="vars.correct_dir ~= 0"},
   {"CORRECT_POSITION", "FINAL", cond=rough_correct_done},
   {"APPROACH_AMPEL", "FINAL", cond=producing, desc="already there"},
   {"CORRECT_POSITION", "FINAL", precond=producing, desc="already there"} 
}

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function CHECK_POSITION:init()
   if sensor:analog_in(LEFT_IR_ID) > 9 then
      self.fsm.vars.correct_dir = 1
   elseif sensor:analog_in(RIGHT_IR_ID) > 9 then
      self.fsm.vars.correct_dir = -1
   else
      self.fsm.vars.correct_dir = 0
   end
end

function SEE_AMPEL:init()
   laserswitch:msgq_enqueue_copy(laserswitch.EnableSwitchMessage:new())
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.0))
end

function APPROACH_AMPEL:init()
   -- enable vision to see if we already have placed the puck under the rfid
   laserswitch:msgq_enqueue_copy(laserswitch.EnableSwitchMessage:new())
   lightswitch:msgq_enqueue_copy(lightswitch.EnableSwitchMessage:new())

   local ampel = get_ampel()
   self.skills[1].x = ampel.x - LASER_FORWARD_CORRECTION
   self.skills[1].y = ampel.y
   self.skills[1].vel_trans = 0.1
end

function CORRECT_POSITION:init()
   self.skills[1].y = self.fsm.vars.correct_dir * 0.3
   self.skills[1].vel_trans = 0.03
end

function FINAL:init()
   laserswitch:msgq_enqueue_copy(laserswitch.DisableSwitchMessage:new())
end
