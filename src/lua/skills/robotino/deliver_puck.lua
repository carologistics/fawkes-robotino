
----------------------------------------------------------------------------
--  chase_puck.lua
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
name               = "deliver_puck"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"take_puck_to", "move_under_rfid", "watch_signal", "leave_area", "motor_move", "ppgoto", "deposit_puck", "global_motor_move" }
depends_interfaces = {
   { v="light", type="RobotinoLightInterface", id="Light_State" },
   { v="sensor", type="RobotinoSensorInterface", id="Robotino" },
   { v="laser", type="SwitchInterface", id="laser-cluster" },
   { v="pose", type="Position3DInterface", id="Pose" },
   { v = "light_switch", type="SwitchInterface", id="light_front_switch"},
   { v = "left_gate", type="RobotinoLightInterface", id="machine_signal_0"},
   { v = "middle_gate", type="RobotinoLightInterface", id="machine_signal_1"},
   { v = "right_gate", type="RobotinoLightInterface", id="machine_signal_2"}
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
--local MOVES = { {y=-0.37}, {y=-0.39}, {y=0.7} }
--local MAX_TRIES = 3
local MAX_RFID_TRIES = 2
local MAX_ORI_ERR = 0.15
local PLUGIN_LIGHT_TIMEOUT = 2.5 -- seconds

local tfm = require("tf_module")

function have_puck()
local curDistance = sensor:distance(8)
   if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
      return true
   end
   return false
end

function left_gate_open()
   return left_gate:green() == left_gate.ON and left_gate:visibility_history() > 20 --TODO and left_gate:is_ready()
end

function middle_gate_open()
   return middle_gate:green() == middle_gate.ON and middle_gate:visibility_history() > 20 --TODO and middle_gate:is_ready()
end

function right_gate_open()
   return right_gate:green() == right_gate.ON and right_gate:visibility_history() > 20 --TODO and right_gate:is_ready()
end

function orange_blinking()
   return light:yellow() == light.BLINKING and light:is_ready()
end

function is_green()
   return light:green() == light.ON and light:visibility_history() > 20 --TODO and light:is_ready()
end

function feedback_ok()
   return light:is_ready()
      and light:green() == light.ON
      and light:yellow() == light.ON
      and light:red() == light.ON
end

fsm:define_states{ export_to=_M,
   closure = {have_puck=have_puck, ampel_green=ampel_green, MAX_TRIES=MAX_TRIES, pose_ok=pose_ok,
      MOVES=MOVES, orange_blinking=orange_blinking, is_green=is_green, 
      light=light, PLUGIN_LIGHT_TIMEOUT=PLUGIN_LIGHT_TIMEOUT, MAX_RFID_TRIES=MAX_RFID_TRIES},
   {"INIT", JumpState},
   {"CHECK_POSE", SkillJumpState, skills={{global_motor_move}}, final_to="DECIDE_GATE", fail_to="DECIDE_GATE" },
   {"DECIDE_GATE", JumpState},
   --{"DECIDE_NEXT_MOVE", JumpState},
   --{"MOVE_TO_NEXT", SkillJumpState, skills={{motor_move}}, final_to="TRY_GATE",
   --   fail_to="FAILED"},
   {"DRIVE_LEFT", SkillJumpState, skills={{motor_move}}, final_to="SETTLE_VISION",
      fail_to="FAILED"},
   {"DRIVE_RIGHT", SkillJumpState, skills={{motor_move}}, final_to="SETTLE_VISION",
      fail_to="FAILED"},
   {"DRIVE_FORWARD", SkillJumpState, skills={{motor_move}}, final_to="SETTLE_VISION",
      fail_to="FAILED"},
   {"CHECK_GATE_AGAIN", JumpState},
   {"SETTLE_VISION", JumpState},
   {"RESTART", SkillJumpState, skills={{take_puck_to}}, final_to="INIT",
      fail_to="FAILED"},
   {"MOVE_UNDER_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="WAIT_FOR_SIGNAL",
      fail_to="WAIT_FOR_SIGNAL"},
   {"WAIT_FOR_SIGNAL", JumpState},
   {"CHECK_RESULT", JumpState},
   {"SKILL_DEPOSIT", SkillJumpState, skills={{deposit_puck}}, final_to="LEAVE_AREA", fail_to="FAILED"},
   {"LEAVE_AREA", SkillJumpState, skills={{leave_area}}, final_to="FINAL", fail_to="FAILED"},
}


fsm:add_transitions{
   {"INIT", "CHECK_POSE", cond=have_puck},
   {"INIT", "FAILED", cond="not have_puck()"},
   {"DECIDE_GATE", "DRIVE_LEFT", cond=left_gate_open, desc="left gate open"},
   {"DECIDE_GATE", "DRIVE_RIGHT", cond=right_gate_open, desc="right gate open"},
   {"DECIDE_GATE", "DRIVE_FORWARD", cond=middle_gate_open, desc="middle gate open"},
   {"SETTLE_VISION", "CHECK_GATE_AGAIN", timeout=1},
   {"CHECK_GATE_AGAIN", "MOVE_UNDER_RFID", cond=is_green, desc="gate is still open"},
   {"CHECK_GATE_AGAIN", "RESTART", cond="not is_green()", desc="gate just got closed, restarting"},
   --{"DECIDE_NEXT_MOVE", "MOVE_UNDER_RFID", cond="vars.num_tries >= MAX_TRIES and vars.cur_gate_idx >= #MOVES"}, --blind guess, doesnt harm
   --{"DECIDE_NEXT_MOVE", "RESTART", cond="vars.cur_gate_idx >= #MOVES"},
   --{"DECIDE_NEXT_MOVE", "MOVE_TO_NEXT", cond="vars.cur_gate_idx < #MOVES"},
   {"WAIT_FOR_SIGNAL", "CHECK_RESULT", timeout=2}, -- wait for deliver
   --{"CHECK_RESULT", "SKILL_DEPOSIT", cond="vars.rfid_tries >= MAX_RFID_TRIES"},
   {"CHECK_RESULT", "MOVE_UNDER_RFID", timeout=PLUGIN_LIGHT_TIMEOUT},
   {"CHECK_RESULT", "LEAVE_AREA", cond=feedback_ok},
   {"CHECK_RESULT", "SKILL_DEPOSIT", cond=orange_blinking},
   --{"CHECK_RESULT", "MOVE_UNDER_RFID", cond="light:is_ready()"}
}

function INIT:init()
   --self.fsm.vars.num_tries = 0
   --self.fsm.vars.rfid_tries = 0
   --self.fsm.vars.cur_gate_idx = 1

   laser:msgq_enqueue_copy(laser.EnableSwitchMessage:new())
end

function CHECK_POSE:init()
   --self.fsm.vars.num_tries = self.fsm.vars.num_tries + 1
   self.skills[1].ori = 0
end

function DECIDE_GATE:init()
   --TODO turn machine_signal on and into delivery mode
   --light_switch:msgq_enqueue_copy(light_switch.EnableSwitchMessage:new())
end

function DRIVE_LEFT:init()
   self.skills[1].y = 0.4
   self.skills[1].x = 0.4
   --self.fsm.vars.cur_gate_idx = self.fsm.vars.cur_gate_idx + 1
end

function DRIVE_RIGHT:init()
   self.skills[1].y = -0.4
   self.skills[1].x = 0.4
   --self.fsm.vars.cur_gate_idx = self.fsm.vars.cur_gate_idx + 1
end

function DRIVE_FORWARD:init()
   self.skills[1].y = 0
   self.skills[1].x = 0.4
   --self.fsm.vars.cur_gate_idx = self.fsm.vars.cur_gate_idx + 1
end

function RESTART:init()
   self.skills[1].place = self.fsm.vars.place
end

--function MOVE_UNDER_RFID:init()
   --self.fsm.vars.rfid_tries = self.fsm.vars.rfid_tries + 1
--   printf("RFID Tries: %f", self.fsm.vars.rfid_tries)
--end

function CHECK_RESULT:init()
   light_switch:msgq_enqueue_copy(light_switch.EnableSwitchMessage:new())
end

function SKILL_DEPOSIT:init()
   self.skills[1].mtype = "deliver"
end

function FINAL:init()
   laser:msgq_enqueue_copy(laser.DisableSwitchMessage:new())
end

function FAILED:init()
   laser:msgq_enqueue_copy(laser.DisableSwitchMessage:new())
end

