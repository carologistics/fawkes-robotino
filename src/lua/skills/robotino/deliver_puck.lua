
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
depends_skills     = {"take_puck_to", "move_under_rfid", "watch_signal", "leave_area", "motor_move", "relgoto", "deposit_puck" }
depends_interfaces = {
   { v="light", type="RobotinoLightInterface", id="Light_State" },
   { v="sensor", type="RobotinoSensorInterface", id="Robotino" },
   { v="laser", type="SwitchInterface", id="laser-cluster" },
   { v="pose", type="Position3DInterface", id="Pose" },
   { v = "lightFrontSwitch", type="SwitchInterface", id="light_front_switch"}
}

documentation     = [==[delivers already fetched puck to specified location]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
local THRESHOLD_DISTANCE = 0.05
-- you can find the config value in /cfg/host.yaml
local THRESHOLD_DISTANCE = config:get_float("/skills/deliver_puck/front_sensor_dist")
local DELIVERY_GATES = { "D1", "D2", "D3" }
local MOVES = { {y=-0.37}, {y=-0.43}, {y=0.7} }
local MAX_TRIES = 2
local MAX_ORI_ERR = 0.15

local tfm = require("tf_module")

function have_puck()
local curDistance = sensor:distance(8)
   if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
      return true
   end
   return false
end

function try_again()
 return (not ampel_green()) and (vars.cur_gate_idx >= #dg) and (vars.numtries<MAX_TRIES)
end

function ampel_green()
   return light:green() == light.ON and light:visibility_history() > 15
end

function ampel_red()
   return light:red() == light.ON and light:visibility_history() > 15
end

function orange_blinking()
   return light:yellow() == light.BLINKING and light:visibility_history() > 15
end

function pose_ok()
   print(math.abs(2*math.acos(pose:rotation(3))))
   return math.abs(2*math.acos(pose:rotation(3))) <= MAX_ORI_ERR
end

fsm:define_states{ export_to=_M,
   closure = {have_puck=have_puck, ampel_green=ampel_green, MAX_TRIES=MAX_TRIES, pose_ok=pose_ok,
      MOVES=MOVES, orange_blinking=orange_blinking},
   {"INIT", JumpState}, -- initial state
   {"CHECK_POSE", JumpState},
   {"CORRECT_TURN", SkillJumpState, skills={{relgoto}}, final_to="TRY_GATE",
      fail_to="FAILED"},
   {"TRY_GATE", JumpState},
   {"DECIDE_RESTART", JumpState},
   {"MOVE_TO_NEXT", SkillJumpState, skills={{motor_move}}, final_to="TRY_GATE",
      fail_to="FAILED"},
   {"RESTART", SkillJumpState, skills={{take_puck_to}}, final_to="CHECK_POSE",
      fail_to="FAILED"},
   {"MOVE_UNDER_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="CHECK_ORANGE_BLINKING",
      fail_to="FAILED"},
   {"CHECK_ORANGE_BLINKING", JumpState},
   {"LEAVE_AREA", SkillJumpState, skills={{leave_area}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_DEPOSIT", SkillJumpState, skills={{deposit_puck}}, final_to="FAILED", fail_to="FAILED"} -- just leave the puck not harming the other delivery processes
}
   

fsm:add_transitions{
   {"INIT", "FAILED", cond="not have_puck()", desc="No puck seen by Infrared"},
   {"INIT", "CHECK_POSE", cond=have_puck},
   {"CHECK_POSE", "CORRECT_TURN", cond="not pose_ok()"},
   {"CHECK_POSE", "TRY_GATE", cond=pose_ok},
   {"TRY_GATE", "MOVE_UNDER_RFID", cond=ampel_green, desc="green"},
   {"TRY_GATE", "DECIDE_RESTART", cond=ampel_red, desc="red"},
   {"TRY_GATE", "DECIDE_RESTART", timeout=4},
   {"DECIDE_RESTART", "MOVE_UNDER_RFID", cond="vars.numtries > MAX_TRIES and vars.cur_gate_idx >= #MOVES"}, --blind guess, doesnt harm
   {"DECIDE_RESTART", "RESTART", cond="vars.cur_gate_idx >= #MOVES"},
   {"DECIDE_RESTART", "MOVE_TO_NEXT", cond="vars.cur_gate_idx < #MOVES"},
   {"CHECK_ORANGE_BLINKING", "SKILL_DEPOSIT", cond=orange_blinking},
   {"CHECK_ORANGE_BLINKING", "LEAVE_AREA", cond="not orange_blinking()"}
}

function INIT:init()
   self.fsm.vars.numtries = 1
   self.fsm.vars.cur_gate_idx = 1
   laser:msgq_enqueue_copy(laser.EnableSwitchMessage:new())
end

function TRY_GATE:init()
   lightFrontSwitch:msgq_enqueue_copy(lightFrontSwitch.EnableSwitchMessage:new())
end

function MOVE_TO_NEXT:init()
   self.skills[1].ori = MOVES[self.fsm.vars.cur_gate_idx].ori or 0
   self.skills[1].x = MOVES[self.fsm.vars.cur_gate_idx].x or 0
   self.skills[1].y = MOVES[self.fsm.vars.cur_gate_idx].y or 0

   self.fsm.vars.cur_gate_idx = self.fsm.vars.cur_gate_idx + 1
end

function CORRECT_TURN:init()
   local t_bl = tfm.transform({x=0, y=0, ori=0}, "/map", "/base_link")
   print(t_bl.ori)
   self.skills[1].ori = t_bl.ori
end

function RESTART:init()
   self.fsm.vars.numtries = self.fsm.vars.numtries + 1
   self.fsm.vars.cur_gate_idx = 1
   self.skills[1].place = "deliver"
end

function MOVE_UNDER_RFID:init()
   self.skills[1].place = DELIVERY_GATES[self.fsm.vars.cur_gate_idx]
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

