
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
   {v = "euclidean_cluster", type="Position3DInterface", id = "/laser-cluster/ampel/1"},
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "pose", type="Position3DInterface", id="Pose"},
   {v = "laserswitch", type="SwitchInterface", id="/laser-cluster/ampel" },
   {v = "laser_cluster", type="LaserClusterInterface", id="/laser-cluster/ampel" },
   {v = "light", type ="RobotinoLightInterface", id = "/machine-signal/best" },
   {v = "lightswitch", type ="SwitchInterface", id = "/machine-signal" },
}

documentation      = [==[Move under the RFID Reader/Writer]==]

local tfm = require("tf_module")

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require('tf_module')
local LASER_FORWARD_CORRECTION = 0.2
local MIN_VIS_HIST = 10
local LEFT_IR_ID = config:get_float("hardware/robotino/sensors/left_ir_id")
local RIGHT_IR_ID = config:get_float("hardware/robotino/sensors/right_ir_id")

function get_ampel()
   local ampel_loc = false
   if fsm.vars.x and fsm.vars.y then
      ampel_loc = {}
      ampel_loc.x = fsm.vars.x
      ampel_loc.y = fsm.vars.y
   elseif euclidean_cluster:visibility_history() > MIN_VIS_HIST then
      ampel_loc = {}
      ampel_loc.x = euclidean_cluster:translation(0)
      ampel_loc.y = euclidean_cluster:translation(1)
   end

   if ampel_loc then
      ampel_loc.distance = math.sqrt(ampel_loc.x^2, ampel_loc.y^2)
      ampel_loc.angle = math.atan(ampel_loc.x, ampel_loc.y)
   end
   fsm.vars.ampel = ampel_loc
   return ampel_loc
end

function rough_correct_done()
   if fsm.vars.correct_dir == -1 then
      return sensor:analog_in(LEFT_IR_ID) > 9
   else
       return sensor:analog_in(RIGHT_IR_ID) > 9
   end
end

function producing()
   return light:is_ready() == true
      and light:green() == light.ON
      and light:yellow() == light.ON
      and light:red() == light.OFF
end

fsm:define_states{ export_to=_M, closure={ampel=ampel, sensor=sensor, get_ampel=get_ampel},
   {"SEE_AMPEL", JumpState},
   {"STRAFE_TO_AMPEL", SkillJumpState, skills={{motor_move_waypoints}}, final_to="SEE_AMPEL", fail_to="SEE_AMPEL" },
   {"APPROACH_AMPEL", SkillJumpState, skills={{motor_move}},
      final_to="CHECK_POSITION", fail_to="FAILED"},
   {"CHECK_POSITION", JumpState},
   {"CORRECT_POSITION", SkillJumpState, skills={{motor_move}},
      final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"SEE_AMPEL", "FAILED", timeout=10, desc="No Ampel seen with laser"},
   {"SEE_AMPEL", "STRAFE_TO_AMPEL", cond="get_ampel() and math.abs(vars.ampel.y) > 0.35" },
   {"SEE_AMPEL", "APPROACH_AMPEL", cond=get_ampel, desc="Ampel seen with laser"},
   {"CHECK_POSITION", "FINAL", cond="vars.correct_dir == 0"},
   {"CHECK_POSITION", "CORRECT_POSITION", cond="vars.correct_dir ~= 0"},
   {"CORRECT_POSITION", "FINAL", cond=rough_correct_done},
   {"APPROACH_AMPEL", "FINAL", cond=producing, desc="already there"},
   {"CORRECT_POSITION", "FINAL", precond=producing, desc="already there"} 
}

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
   lightswitch:msgq_enqueue_copy(lightswitch.EnableSwitchMessage:new())
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.0))
end

function STRAFE_TO_AMPEL:init()
   local turn_dir = 1
   if self.fsm.vars.ampel.y < 0 then
      turn_dir = -1
   end
   local goto_x = 0.3 * self.fsm.vars.ampel.x
   local goto_y = 0.8 * self.fsm.vars.ampel.y
   self.skills[1].waypoints = {
      {  x = goto_x,
         y = goto_y,
         ori = turn_dir * math.atan2(goto_y, goto_x) },
      {  ori = - turn_dir * math.atan2(goto_y, goto_x) }
   }
end

function APPROACH_AMPEL:init()
   -- enable vision to see if we already have placed the puck under the rfid
   laserswitch:msgq_enqueue_copy(laserswitch.EnableSwitchMessage:new())

   self.skills[1].x = self.fsm.vars.ampel.x - LASER_FORWARD_CORRECTION
   self.skills[1].y = self.fsm.vars.ampel.y
   self.skills[1].vel_trans = 0.25
end

function CORRECT_POSITION:init()
   self.skills[1].y = self.fsm.vars.correct_dir * 0.3
   self.skills[1].vel_trans = 0.02
end

function FINAL:init()
   laserswitch:msgq_enqueue_copy(laserswitch.DisableSwitchMessage:new())
   lightswitch:msgq_enqueue_copy(lightswitch.DisableSwitchMessage:new())
end

function FAILED:init()
   laserswitch:msgq_enqueue_copy(laserswitch.DisableSwitchMessage:new())
   lightswitch:msgq_enqueue_copy(lightswitch.DisableSwitchMessage:new())
end
