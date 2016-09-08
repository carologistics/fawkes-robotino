------------------------------------------------------------------------
--  dock_charge.lua
--
--  Created Thu Jan 28 2016
--  Copyright  2016  Johannes Rothe
------------------------------------------------------------------------
--
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
name               = "dock_charge"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move","align_tag"}
depends_interfaces = { 
   {v = "sensor", type = "RobotinoSensorInterface", id="Robotino"},
   {v = "battery", type = "BatteryInterface", id="Robotino"},
}

documentation      = [==[
                        Aligns and then drives onto the Festo
                        charging station

                        @param tag_id ID of the tag at the 
                                      charging station
                     ]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local default_tag_id = 81
local left_sensor_index = 3
local right_sensor_index = 4
local backwards_distance = 0.14
local voltage_treshold = 26800
local try_dist = 0.03

function no_battery_writer()
   return not battery:has_writer()
end

function charging()
  --TODO check loading voltage
  printf(battery:voltage())
  if battery:voltage() > voltage_treshold then
     return true
  end
end

fsm:define_states{ export_to=_M,
   closure={charging=charging},
   {"INIT", JumpState},
   {"ALIGN_TAG", SkillJumpState, skills={{align_tag}}, final_to="DRIVE_BACKWARDS", fail_to="FAILED"},
   {"DRIVE_BACKWARDS", SkillJumpState, skills={{motor_move}}, final_to="SETTLE_VOLTAGE", fail_to="FAILED"},
   {"SETTLE_VOLTAGE", JumpState},
   {"CHECK_CHARGING", JumpState},
   {"TRY_RIGHT", SkillJumpState, skills={{motor_move}}, final_to="SETTLE_VOLTAGE", fail_to="FAILED"},
   {"TRY_LEFT", SkillJumpState, skills={{motor_move}}, final_to="SETTLE_VOLTAGE", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond=no_battery_writer, desc="No Writer for BatteryInterface"},
   {"INIT", "ALIGN_TAG", cond=true},
   {"SETTLE_VOLTAGE", "CHECK_CHARGING", timeout=3},
   {"CHECK_CHARGING", "TRY_RIGHT", cond="not charging() and vars.try_index == 1", desc="Not charging, try to drive a bit to the right"},
   {"CHECK_CHARGING", "TRY_LEFT", cond="not charging() and vars.try_index == 2", desc="Not charging, try to drive a bit to the left"},
   {"CHECK_CHARGING", "FAILED", cond="not charging() and vars.try_index == 3", desc="Not charging after all tries, something is wrong"},
   {"CHECK_CHARGING", "FINAL", cond=charging, desc="If not charging, do something"},
}

function INIT:init()
   self.fsm.vars.try_index = 1
end

function ALIGN_TAG:init()
   self.args["align_tag"] = {tag_id = self.fsm.vars.tag_id or default_tag_id, x = 0.5, y = 0, ori = 0}
end

function DRIVE_BACKWARDS:init()
   self.args["motor_move"] = {x = -backwards_distance, y = 0, ori = 0,
			                     vel_trans = 0.12,
									   tolerance = { x=0.05, y=0.05, ori=0.01 }}
end

function TRY_RIGHT:init()
   self.fsm.vars.try_index = self.fsm.vars.try_index + 1
   self.args["motor_move"] = {x = 0, y = -try_dist, ori = 0,
			                     vel_trans = 0.08,
									   tolerance = { x=0.02, y=0.02, ori=0.01 }}
end

function TRY_LEFT:init()
   self.fsm.vars.try_index = self.fsm.vars.try_index + 1
   self.args["motor_move"] = {x = 0, y = try_dist * 2, ori = 0,
			                     vel_trans = 0.08,
									   tolerance = { x=0.02, y=0.02, ori=0.01 }}
end
