
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
fsm                = SkillHSM:new{name=name, start="CHECK_PUCK", debug=true}
depends_skills     = {"take_puck_to", "move_under_rfid", "watch_signal", "leave_area", "motor_move"}
depends_interfaces = {
   { v="light", type="RobotinoLightInterface", id="Light determined" },
   { v="sensor", type="RobotinoSensorInterface", id="Robotino" }
}

documentation     = [==[delivers already fetched puck to specified location]==]
-- Constants
local THRESHOLD_DISTANCE = 0.07
local DELIVERY_GATES = { "D2", "D1", "D3" }
local ANGLES = { 0, 0.17*math.pi, -0.34*math.pi}

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require('tf_module')

function have_puck()
local curDistance = sensor:distance(8)
   if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
      return true
   end
   return false
end

function ampel_green()
   return light:green() == light.ON
end

fsm:define_states{ export_to=_M,
   closure = {have_puck=have_puck, dg=DELIVERY_GATES, ampel_green=ampel_green},
   {"CHECK_PUCK", JumpState},
   {"SKILL_DETERMINE_SIGNAL", SkillJumpState, skills={{watch_signal}},
      final_to="DECIDE_DELIVER", fail_to="FAILED"},
   {"DECIDE_DELIVER", JumpState},
   {"TURN_TO_NEXT", SkillJumpState, skills={{motor_move}}, final_to="SKILL_DETERMINE_SIGNAL",
      fail_to="FAILED"},
   {"TAKE_PUCK_WAIT", JumpState},
   {"TAKE_PUCK_TO", SkillJumpState, skills={{take_puck_to}}, final_to="MOVE_UNDER_RFID", fail_to="FAILED"},
   {"MOVE_UNDER_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="LEAVE_AREA",
      fail_to="FAILED"},
   {"LEAVE_AREA", SkillJumpState, skills={{leave_area}}, final_to="FINAL", fail_to="FAILED"}
}
   

fsm:add_transitions{
   {"CHECK_PUCK", "FAILED", cond="not have_puck()", desc="No puck seen by Infrared"},
   {"CHECK_PUCK", "SKILL_DETERMINE_SIGNAL", cond=have_puck},
   {"DECIDE_DELIVER", "MOVE_UNDER_RFID", cond="ampel_green() and vars.cur_gate_idx == 1", desc="Gate 2 green"},
   {"DECIDE_DELIVER", "TAKE_PUCK_WAIT", cond="ampel_green() and vars.cur_gate_idx ~= 1", desc="Gate 1 or 3 green"},
   {"TAKE_PUCK_WAIT", "TAKE_PUCK_TO", timeout=1},
   {"DECIDE_DELIVER", "TURN_TO_NEXT", cond="vars.cur_gate_idx < #dg"},
   {"DECIDE_DELIVER", "FAILED", cond="(not ampel_green()) and (vars.cur_gate_idx >= #dg)"}
}

function CHECK_PUCK:init()
   self.fsm.vars.cur_gate_idx = 1
end

function TURN_TO_NEXT:init()
   self.fsm.vars.cur_gate_idx = self.fsm.vars.cur_gate_idx + 1
   --turn_bl = tfm.transform({x=0, y=0, ori=ANGLES[self.fsm.vars.cur_gate_idx]}, "/map", "/base_link")
   turn_bl = {}
   turn_bl.ori = ANGLES[self.fsm.vars.cur_gate_idx]
   print(turn_bl.ori)
   self.skills[1].ori = turn_bl.ori
end

function TAKE_PUCK_TO:init()
   self.skills[1].place = DELIVERY_GATES[self.fsm.vars.cur_gate_idx]
end

function MOVE_UNDER_RFID:init()
   self.skills[1].place = DELIVERY_GATES[self.fsm.vars.cur_gate_idx]
end
