
----------------------------------------------------------------------------
--  deliver_puck.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2014  Carologistics
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
name               = "deliver_puck"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"take_puck_to", "move_under_rfid", "leave_area", "motor_move", "deposit_puck", "global_motor_move" }
depends_interfaces = {
   { v="bb_open_gate", type="Position3DInterface", id="open_delivery_gate" },
   { v="light", type="RobotinoLightInterface", id="Light_State" },
   { v="sensor", type="RobotinoSensorInterface", id="Robotino" },
   { v="bb_laser_switch", type="SwitchInterface", id="laser-cluster" },
   { v="bb_laser_ctl", type="LaserClusterInterface", id="laser-cluster" },
   { v="pose", type="Position3DInterface", id="Pose" },
   { v = "light_switch", type="SwitchInterface", id="light_front_switch"},
   { v = "left_gate", type="RobotinoLightInterface", id="machine_signal_0"},
   { v = "middle_gate", type="RobotinoLightInterface", id="machine_signal_1"},
   { v = "right_gate", type="RobotinoLightInterface", id="machine_signal_2"},
   { v = "delivery_mode", type="SwitchInterface", id="machine_signal_delivery_mode"}
}

documentation     = [==[delivers already fetched puck to specified location]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local THRESHOLD_DISTANCE = 0.07
if config:exists("/skills/deliver_puck/front_sensor_dist") then
   -- you can find the config value in /cfg/host.yaml
   THRESHOLD_DISTANCE = config:get_float("/skills/deliver_puck/front_sensor_dist")
end

local SIGNAL_TIMEOUT = 4 -- seconds
local MAX_NUM_TRIES = 3
local MIN_VIS_HIST = 10

local tfm = require("tf_module")

local gates = { left_gate, middle_gate, right_gate }

function have_puck()
   local curDistance = sensor:distance(8)
   if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
      return true
   end
   return false
end

function orange_blinking()
   return light:yellow() == light.BLINKING and light:is_ready()
end

function is_green()
   return light:green() == light.ON and light:is_ready()
end


function is_red()
   return light:red() == light.ON and light:is_ready()
end


function feedback_ok()
   return light:is_ready()
      and light:green() == light.ON
      and light:yellow() == light.ON
      and light:red() == light.ON
end


function no_open_gate()
   if bb_open_gate:visibility_history() >= MIN_VIS_HIST then return false end
   count_not_open = 0
   for i,gate in ipairs(gates) do
      if gate:visibility_history() >= MIN_VIS_HIST then
         if gate:green() == gate.ON then
            return false
         end
         if gate:red() == gate.ON then
            count_not_open = count_not_open + 1
         end
      end
   end
   return count_not_open >= 2
end


function max_tries_reached()
   return fsm.vars.num_tries >= MAX_NUM_TRIES
end


function open_gate()
   if bb_open_gate:visibility_history() >= MIN_VIS_HIST then
      fsm.vars.open_gate = {}
      fsm.vars.open_gate.x = bb_open_gate:translation(0)
      fsm.vars.open_gate.y = bb_open_gate:translation(1)
   else
      fsm.vars.open_gate = false
   end
   return fsm.vars.open_gate
end


fsm:define_states{ export_to=_M,
   closure = {have_puck=have_puck, orange_blinking=orange_blinking,is_red=is_red,
              is_green=is_green, PLUGIN_LIGHT_TIMEOUT=PLUGIN_LIGHT_TIMEOUT,
              SETTLE_VISION_TIME=SETTLE_VISION_TIME, max_tries_reached=max_tries_reached},
   {"INIT", JumpState},

   {"TRY_LEFT", JumpState},
   {"DRIVE_RIGHT", SkillJumpState, skills={{global_motor_move}}, final_to="TRY_RIGHT",
      fail_to="TRY_RIGHT"},
   {"TRY_RIGHT", JumpState},
   {"DRIVE_LEFT", SkillJumpState, skills={{global_motor_move}}, final_to="TRY_LEFT",
      fail_to="TRY_LEFT"},

   {"MOVE_UNDER_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="CHECK_RESULT",
      fail_to="CHECK_RESULT"},
   
   {"CHECK_RESULT", JumpState},
   
   {"SKILL_DEPOSIT", SkillJumpState, skills={{deposit_puck}}, final_to="LEAVE_AREA", fail_to="FAILED"},
   {"LEAVE_AREA", SkillJumpState, skills={{leave_area}}, final_to="FINAL", fail_to="FAILED"},
}


fsm:add_transitions{
   {"INIT", "FAILED", cond="not have_puck()"},
   {"INIT", "TRY_LEFT", cond="vars.place == \"deliver1\" or vars.place == \"deliver2\""},
   {"INIT", "TRY_RIGHT", cond="vars.place == \"deliver1a\" or vars.place == \"deliver2a\""},
   
   {"TRY_LEFT", "DRIVE_RIGHT", cond=no_open_gate, timeout=SIGNAL_TIMEOUT},
   {"TRY_LEFT", "MOVE_UNDER_RFID", cond=open_gate},
   {"TRY_LEFT", "MOVE_UNDER_RFID", cond=max_tries_reached, desc="lose the puck before failing"},
   {"TRY_RIGHT", "DRIVE_LEFT", cond=no_open_gate, timeout=SIGNAL_TIMEOUT},
   {"TRY_RIGHT", "MOVE_UNDER_RFID", cond=open_gate},
   {"TRY_RIGHT", "MOVE_UNDER_RFID", cond=max_tries_reached, desc="lose the puck before failing"},
   
   {"CHECK_RESULT", "MOVE_UNDER_RFID", timeout=SIGNAL_TIMEOUT},
   {"CHECK_RESULT", "LEAVE_AREA", cond=feedback_ok, desc="all lights on"},
   {"CHECK_RESULT", "SKILL_DEPOSIT", cond=orange_blinking},
   {"CHECK_RESULT", "SKILL_DEPOSIT", timeout=SIGNAL_TIMEOUT},
}

function INIT:init()
   --turn machine_signal on and into delivery mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.EnableSwitchMessage:new())
   bb_laser_switch:msgq_enqueue_copy(bb_laser_switch.EnableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.EnableSwitchMessage:new())
   msg = bb_laser_ctl.SetSelectionModeMessage:new()
   msg:set_selection_mode(bb_laser_ctl.SELMODE_MIN_ANGLE)
   bb_laser_ctl:msgq_enqueue_copy(msg)
   self.fsm.vars.num_tries = 1
end

function DRIVE_LEFT:init()
   self.skills[1].place = self.fsm.vars.place
   self.fsm.vars.num_tries = self.fsm.vars.num_tries + 1
end

function MOVE_UNDER_RFID:init()
   if self.fsm.vars.open_gate then
      self.skills[1].x = self.fsm.vars.open_gate.x
      self.skills[1].y = self.fsm.vars.open_gate.y
   end
end

function DRIVE_RIGHT:init()
   if self.fsm.vars.place == "deliver1" then
      self.skills[1].place = "deliver1a"
   else
      self.skills[1].place = "deliver2a"
   end
end

function SKILL_DEPOSIT:init()
   self.skills[1].mtype = "deliver"
end

--[[
function GET_RID_OF_PUCK:init()
   self.skills[1].x = -0.2
   self.skills[1].ori = 0.45 -- 20°
   self.skills[1].vel_trans = 0.8
end
]]--

function FINAL:init()
   --turn machine_signal off and into normal mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.DisableSwitchMessage:new())
   bb_laser_switch:msgq_enqueue_copy(bb_laser_switch.DisableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.DisableSwitchMessage:new())
   msg = bb_laser_ctl.SetSelectionModeMessage:new()
   msg:set_selection_mode(bb_laser_ctl.SELMODE_MIN_DIST)
   bb_laser_ctl:msgq_enqueue_copy(msg)
   printf("FINAL: VISION DISABLED")
end

function FAILED:init()
   --turn machine_signal off and into normal mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.DisableSwitchMessage:new())
   bb_laser_switch:msgq_enqueue_copy(bb_laser_switch.DisableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.DisableSwitchMessage:new())
   msg = bb_laser_ctl.SetSelectionModeMessage:new()
   msg:set_selection_mode(bb_laser_ctl.SELMODE_MIN_DIST)
   bb_laser_ctl:msgq_enqueue_copy(msg)
   printf("FAILED: VISION DISABLED")
end

