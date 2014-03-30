
----------------------------------------------------------------------------
--  get_produced.lua
--
--  Created: Mar 11 2014 
--  Copyright  2014 Johannes Rothe
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
name               = "get_produced"
fsm                = SkillHSM:new{name=name, start="GOTO_MACHINE", debug=true}
depends_skills     = {"motor_move", "ppgoto", "global_motor_move"}
depends_interfaces = {
  {v = "sensor", type="RobotinoSensorInterface", id = "Robotino"},
  {v = "euclidean_cluster", type="Position3DInterface", id = "Euclidean Laser Cluster"},
  {v = "laserswitch", type="SwitchInterface", id="laser-cluster" },
  { v="lightswitch", type="SwitchInterface", id="light_front_switch" },
  { v="light", type ="RobotinoLightInterface", id = "Light_State" },
  {v = "laser_cluster", type="LaserClusterInterface", id="laser-cluster" },
}

documentation      = [==[Get a produced puck from under the RFID]==]

-- Initialize as skill module
skillenv.skill_module(_M)
--fawkes.load_yaml_navgraph already searches in the cfg directory
graph = fawkes.load_yaml_navgraph("navgraph-llsf.yaml")

local LASER_FORWARD_CORRECTION = 0.17
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

function producing()
   return light:green() == light.ON
      and light:yellow() == light.ON
      and light:red() == light.OFF
end

fsm:define_states{ export_to=_M, closure={producing = producing},
   {"GOTO_MACHINE", SkillJumpState, skills={{ppgoto}}, final_to="ADJUST_POS", fail_to="FAILED"},
   {"ADJUST_POS", SkillJumpState, skills={{global_motor_move}}, final_to="SEE_AMPEL", fail_to="FAILED"},
   {"SEE_AMPEL", JumpState},
   {"CHECK_PRODUCE", JumpState},
   {"WAIT_PRODUCE", JumpState},
   {"TURN", SkillJumpState, skills = {{motor_move}}, final_to = "APPROACH_AMPEL", fail_to = "FAILED"},
   {"APPROACH_AMPEL", SkillJumpState, skills = {{motor_move}}, final_to = "CHECK_POSITION", fail_to = "FAILED"},
   {"CHECK_POSITION", JumpState},
   {"CORRECT_POSITION", SkillJumpState, skills = {{motor_move}}, final_to = "CORRECT_SENSOR_DELAY", fail_to = "FAILED"},
   {"CORRECT_SENSOR_DELAY", SkillJumpState, skills = {{motor_move}}, final_to = "SKILL_DRIVE_LEFT", fail_to = "FAILED"},
   {"SKILL_DRIVE_LEFT", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"SEE_AMPEL", "FAILED", timeout = 5, desc = "No Ampel seen with laser"},
   {"SEE_AMPEL", "CHECK_PRODUCE", cond = ampel, desc = "Ampel seen with laser"},
   {"CHECK_PRODUCE", "FAILED", timeout = 5, desc = "No Light detected"},
   {"CHECK_PRODUCE", "TURN", cond = "not producing()", desc = "The product is finished"},
   {"CHECK_PRODUCE", "WAIT_PRODUCE", cond = producing, desc = "Wait until the Product is finished"},
   {"WAIT_PRODUCE", "TURN", cond = "not producing()", desc = "The product is finished"},
   {"WAIT_PRODUCE", "FAILED", timeout = 62, desc = "Can't read the light properly"},
   {"CHECK_POSITION", "SKILL_DRIVE_LEFT", cond = "vars.correct_dir == 0"},
   {"CHECK_POSITION", "CORRECT_POSITION", cond = "vars.correct_dir ~= 0"},
   {"CORRECT_POSITION", "CORRECT_SENSOR_DELAY", cond = rough_correct_done},
}

function GOTO_MACHINE:init()
   self.skills[1].place = graph:closest_node_to(self.fsm.vars.place, "highway_exit"):name()
end

function ADJUST_POS:init()
   self.skills[1].place = self.fsm.vars.place
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
   --TODO enable vision to see if there is really a puck under the RFID

   local ampel = get_ampel()
   self.skills[1].x = ampel.x - LASER_FORWARD_CORRECTION
   self.skills[1].y = ampel.y
   self.skills[1].vel_trans = 0.1
end

function CORRECT_POSITION:init()
   self.skills[1].y = self.fsm.vars.correct_dir * 0.3
   self.skills[1].vel_trans = 0.03
end

function CORRECT_SENSOR_DELAY:init()
   self.skills[1].y = self.fsm.vars.correct_dir * -1 * LIGHT_SENSOR_DELAY_CORRECTION 
end

function SKILL_DRIVE_LEFT:init()
   if graph:node(self.fsm.vars.goto_name):has_property("leave_right") then
      self.skills[1].y = -0.4
      self.skills[1].vel_rot = 1
   else
      self.skills[1].y = 0.4
      self.skills[1].vel_rot = 1
   end
end

function FINAL:init()
   laserswitch:msgq_enqueue_copy(laserswitch.DisableSwitchMessage:new())
end
