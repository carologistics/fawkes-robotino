
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
depends_skills     = {
   "take_puck_to", "move_under_rfid", "motor_move",
   "motor_move_waypoints", "deposit_puck", "global_move_laserlines"
}

depends_interfaces = {
   { v="bb_open_gate", type="Position3DInterface", id="/machine-signal/open-delivery-gate" },
   { v="light", type="RobotinoLightInterface", id="/machine-signal/best" },
   { v="sensor", type="RobotinoSensorInterface", id="Robotino" },
   { v="bb_laser_switch", type="SwitchInterface", id="/laser-cluster/ampel" },
   { v="bb_laser_ctl", type="LaserClusterInterface", id="/laser-cluster/ampel" },
   { v="pose", type="Position3DInterface", id="Pose" },
   { v = "light_switch", type="SwitchInterface", id="/machine-signal"},
   { v = "left_gate", type="RobotinoLightInterface", id="/machine-signal/0"},
   { v = "middle_gate", type="RobotinoLightInterface", id="/machine-signal/1"},
   { v = "right_gate", type="RobotinoLightInterface", id="/machine-signal/2"},
   { v = "delivery_mode", type="SwitchInterface", id="/machine-signal/delivery-mode"}
}

documentation     = [==[delivers already fetched puck to specified location]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local LOSTPUCK_DIST = 0.07
local PUCK_SENSOR_INDEX = 8
if config:exists("/hardware/robotino/puck_sensor/trigger_dist") then
   LOSTPUCK_DIST = config:get_float("/hardware/robotino/puck_sensor/trigger_dist")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/trigger_dist FOUND! Using default value\n");
end
if config:exists("/hardware/robotino/puck_sensor/index") then
   -- you can find the config value in /cfg/host.yaml
   PUCK_SENSOR_INDEX = config:get_uint("/hardware/robotino/puck_sensor/index")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/index FOUND! Using default value\n");
end


local SIGNAL_TIMEOUT = 5 -- seconds
local MAX_NUM_TRIES = 3
local MIN_VIS_HIST = 10

local tfm = require("tf_module")

local gates = { left_gate, middle_gate, right_gate }

function have_puck()
   local curDistance = sensor:distance(PUCK_SENSOR_INDEX)
   if (curDistance > 0) and (curDistance <= LOSTPUCK_DIST) then
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

function only_green()
   return light:is_ready()
      and light:red() == light.OFF
      and light:yellow() == light.OFF
      and light:green() == light.ON
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
              SETTLE_VISION_TIME=SETTLE_VISION_TIME, max_tries_reached=max_tries_reached,
              MAX_NUM_TRIES=MAX_NUM_TRIES },
   {"INIT", JumpState},

   {"SKILL_TAKE_PUCK", SkillJumpState, skills={{take_puck_to}}, final_to="POSITION_FIRST",
      fail_to="FAILED" },
   {"POSITION_FIRST", SkillJumpState, skills={{global_move_laserlines}}, final_to="TRY_FIRST",
      fail_to="SKILL_TAKE_PUCK"},
   {"TRY_FIRST", JumpState},

   {"DRIVE_SECOND", SkillJumpState, skills={{global_move_laserlines}}, final_to="TRY_SECOND",
      fail_to="TRY_SECOND"},
   {"TRY_SECOND", JumpState},

   {"MOVE_UNDER_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="CHECK_RESULT",
      fail_to="CHECK_RESULT"},
   
   {"CHECK_RESULT", JumpState},
   
   {"LEAVE_AREA", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FINAL"},
   {"REMOVE_KEBAB", SkillJumpState, skills={{deposit_puck}}, final_to="FAILED", fail_to="FAILED"},
}


fsm:add_transitions{
   {"INIT", "FAILED", cond="not have_puck()"},
   {"INIT", "SKILL_TAKE_PUCK", cond=have_puck},

   {"TRY_FIRST", "DRIVE_SECOND", cond=no_open_gate, timeout=SIGNAL_TIMEOUT},
   {"TRY_FIRST", "MOVE_UNDER_RFID", cond=open_gate},
   {"TRY_FIRST", "MOVE_UNDER_RFID", cond=max_tries_reached, desc="lose the puck before failing"},
   {"TRY_SECOND", "SKILL_TAKE_PUCK", cond=no_open_gate, timeout=SIGNAL_TIMEOUT},
   {"TRY_SECOND", "MOVE_UNDER_RFID", cond=open_gate},
   {"TRY_SECOND", "MOVE_UNDER_RFID", cond=max_tries_reached, desc="lose the puck before failing"},
   
   {"CHECK_RESULT", "LEAVE_AREA", cond=feedback_ok, desc="RYG"},
   {"CHECK_RESULT", "REMOVE_KEBAB", cond=orange_blinking, timeout=6, desc="Y blink"},
}

function enable_vision()
   --turn machine_signal on and into delivery mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.EnableSwitchMessage:new())
   bb_laser_switch:msgq_enqueue_copy(bb_laser_switch.EnableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.EnableSwitchMessage:new())
   msg = bb_laser_ctl.SetSelectionModeMessage:new()
   msg:set_selection_mode(bb_laser_ctl.SELMODE_MIN_ANGLE)
   bb_laser_ctl:msgq_enqueue_copy(msg)
end

function INIT:init()
   self.fsm.vars.num_tries = 1
   self.fsm.vars.num_rfid_tries = 0
end

function SKILL_TAKE_PUCK:init()
   self.skills[1].place = self.fsm.vars.place
end

function POSITION_FIRST:init()
   enable_vision()
   self.skills[1].place = self.fsm.vars.place
   self.fsm.vars.num_tries = self.fsm.vars.num_tries + 1
end

function MOVE_UNDER_RFID:init()
   self.fsm.vars.num_rfid_tries = self.fsm.vars.num_rfid_tries + 1
   if self.fsm.vars.open_gate then
      self.skills[1].x = self.fsm.vars.open_gate.x
      self.skills[1].y = self.fsm.vars.open_gate.y
   end
end

function DRIVE_SECOND:init()
   if self.fsm.vars.place == "deliver1" then
      self.skills[1].place = "deliver1a"
   elseif self.fsm.vars.place == "deliver1a" then
      self.skills[1].place = "deliver1"
   elseif self.fsm.vars.place == "deliver2" then
      self.skills[1].place = "deliver2a"
   else
      self.skills[1].place = "deliver2"
   end
end

function REMOVE_KEBAB:init()
   self.skills[1].delivery = true
end

function cleanup()
   -- turn machine_signal off and into normal mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.DisableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.DisableSwitchMessage:new())
   
   -- set laser-cluster selection mode to distance
   msg = bb_laser_ctl.SetSelectionModeMessage:new()
   msg:set_selection_mode(bb_laser_ctl.SELMODE_MIN_DIST)
   bb_laser_ctl:msgq_enqueue_copy(msg)
   
   -- switch off laser-cluster
   bb_laser_switch:msgq_enqueue_copy(bb_laser_switch.DisableSwitchMessage:new())
   
end


function FINAL:init()
   cleanup()
end

function FAILED:init()
   cleanup()
end

