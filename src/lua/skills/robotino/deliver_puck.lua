
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
   { v="light", type="RobotinoLightInterface", id="Light_State" },
   { v="sensor", type="RobotinoSensorInterface", id="Robotino" },
   { v="laser", type="SwitchInterface", id="laser-cluster" },
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

local PLUGIN_LIGHT_TIMEOUT = 4 -- seconds
local SETTLE_VISION_TIME = 1 --seconds
local MAX_NUM_TRIES = 3

local tfm = require("tf_module")

function have_puck()
local curDistance = sensor:distance(8)
   if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
      return true
   end
   return false
end

function left_gate_open()
   return left_gate:green() == left_gate.ON and left_gate:visibility_history() > 10 --TODO and left_gate:is_ready()
end

function middle_gate_open()
   return middle_gate:green() == middle_gate.ON and middle_gate:visibility_history() > 10 --TODO and middle_gate:is_ready()
end

function right_gate_open()
   return right_gate:green() == right_gate.ON and right_gate:visibility_history() > 10 --TODO and right_gate:is_ready()
end

function orange_blinking()
   return light:yellow() == light.BLINKING and light:visibility_history() > 10 --TODO light:is_ready()
end

function is_green()
   printf("IS GREEN FUNKTION")
   return light:green() == light.ON and light:visibility_history() > 5 --TODO and light:is_ready()
end

function is_red()
   return light:red() == light.ON and light:visibility_history() > 5 --TODO and light:is_ready()
end
function feedback_ok()
   return light:is_ready()
      and light:green() == light.ON
      and light:yellow() == light.ON
      and light:red() == light.ON
end

function max_tries_reached()
   return fsm.vars.num_tries >= MAX_NUM_TRIES
end

fsm:define_states{ export_to=_M,
   closure = {have_puck=have_puck, orange_blinking=orange_blinking,is_red=is_red,
              is_green=is_green, PLUGIN_LIGHT_TIMEOUT=PLUGIN_LIGHT_TIMEOUT,
              SETTLE_VISION_TIME=SETTLE_VISION_TIME, max_tries_reached=max_tries_reached},
   {"INIT", JumpState},
   {"DECIDE_GATE", JumpState},
   {"DRIVE_LEFT", SkillJumpState, skills={{global_motor_move}}, final_to="SETTLE_VISION",
      fail_to="DECIDE_GATE"},
   {"DRIVE_RIGHT", SkillJumpState, skills={{global_motor_move}}, final_to="SETTLE_VISION",
      fail_to="DECIDE_GATE"},
   {"DRIVE_MIDDLE", SkillJumpState, skills={{global_motor_move}}, final_to="SETTLE_VISION",
      fail_to="DECIDE_GATE"},
   {"CHECK_GATE_AGAIN", JumpState},
   {"SETTLE_VISION", JumpState},
   {"RESTART", SkillJumpState, skills={{take_puck_to}}, final_to="GLOBAL_MOTOR_MOVE",
      fail_to="FAILED"},
   {"GLOBAL_MOTOR_MOVE", SkillJumpState, skills={{global_motor_move}}, final_to="DECIDE_GATE",
      fail_to="DECIDE_GATE"},
   {"MOVE_UNDER_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="WAIT_FOR_SIGNAL",
      fail_to="WAIT_FOR_SIGNAL"},
   {"WAIT_FOR_SIGNAL", JumpState},
   {"CHECK_RESULT", JumpState},
   {"SKILL_DEPOSIT", SkillJumpState, skills={{deposit_puck}}, final_to="LEAVE_AREA", fail_to="FAILED"},
   {"LEAVE_AREA", SkillJumpState, skills={{leave_area}}, final_to="FINAL", fail_to="FAILED"},
   {"GET_RID_OF_PUCK", SkillJumpState, skills={{motor_move}}, final_to="FAILED",
      fail_to="FAILED"},
}


fsm:add_transitions{
   {"INIT", "DECIDE_GATE", cond="have_puck and not max_tries_reached()"},
   {"INIT", "FAILED", cond="not have_puck()"},
   {"DECIDE_GATE", "DRIVE_LEFT", cond=left_gate_open, desc="left gate open"},
   {"DECIDE_GATE", "DRIVE_RIGHT", cond=right_gate_open, desc="right gate open"},
   {"DECIDE_GATE", "DRIVE_MIDDLE", cond=middle_gate_open, desc="middle gate open"},
   {"SETTLE_VISION", "CHECK_GATE_AGAIN", timeout=SETTLE_VISION_TIME, desc="Let the vision settle"},
   {"CHECK_GATE_AGAIN", "MOVE_UNDER_RFID", cond="not is_red()", desc="gate is still open"},
   {"CHECK_GATE_AGAIN", "RESTART", cond=is_red, desc="gate just got closed, restarting"},
   {"RESTART", "MOVE_UNDER_RFID", cond=max_tries_reached, desc="lose the puck before failing"},
   {"WAIT_FOR_SIGNAL", "CHECK_RESULT", timeout=2, desc="wait for the deliver registry"},
   {"CHECK_RESULT", "MOVE_UNDER_RFID", timeout=PLUGIN_LIGHT_TIMEOUT},
   {"CHECK_RESULT", "LEAVE_AREA", cond=feedback_ok, desc="all lights on"},
   {"CHECK_RESULT", "SKILL_DEPOSIT", cond=orange_blinking},
}

function INIT:init()
   --turn machine_signal on and into delivery mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.EnableSwitchMessage:new())
   laser:msgq_enqueue_copy(laser.EnableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.EnableSwitchMessage:new())
   self.fsm.vars.num_tries = 1
   printf("INIT: VISION ENABLED")
end

function DRIVE_LEFT:init()
   if self.fsm.vars.place == "deliver1" then
      self.skills[1].place = "D13"
   else
      self.skills[1].place = "D21"
   end
end

function DRIVE_MIDDLE:init()
   if self.fsm.vars.place == "deliver1" then
      self.skills[1].place = "D12"
   else
      self.skills[1].place = "D22"
   end
end

function DRIVE_RIGHT:init()
   if self.fsm.vars.place == "deliver1" then
      self.skills[1].place = "D11"
   else
      self.skills[1].place = "D23"
   end
end

function CHECK_GATE_AGAIN:init()
   --turn machine_signal on and into delivery mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.EnableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.EnableSwitchMessage:new())
   printf("CHECK_GATE_AGAIN: VISION ENABLED")
end

function RESTART:init()
   self.fsm.vars.num_tries = self.fsm.vars.num_tries + 1
   self.skills[1].place = self.fsm.vars.place
end

function GLOBAL_MOTOR_MOVE:init()
   self.skills[1].place = self.fsm.vars.place
end

function SKILL_DEPOSIT:init()
   self.skills[1].mtype = "deliver"
end

function GET_RID_OF_PUCK:init()
   self.skills[1].x = -0.2
   self.skills[1].ori = 0.45 -- 20°
   self.skills[1].vel_trans = 0.8
end

function FINAL:init()
   --turn machine_signal off and into normal mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.DisableSwitchMessage:new())
   laser:msgq_enqueue_copy(laser.DisableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.DisableSwitchMessage:new())
   printf("FINAL: VISION DISABLED")
end

function FAILED:init()
   --turn machine_signal off and into normal mode
   delivery_mode:msgq_enqueue_copy(delivery_mode.DisableSwitchMessage:new())
   laser:msgq_enqueue_copy(laser.DisableSwitchMessage:new())
   light_switch:msgq_enqueue_copy(light_switch.DisableSwitchMessage:new())
   printf("FAILED: VISION DISABLED")
end

