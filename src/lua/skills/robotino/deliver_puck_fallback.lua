
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
name               = "deliver_puck_fallback"
fsm                = SkillHSM:new{name=name, start="CHECK_PUCK", debug=true}
depends_skills     = {"take_puck_to", "move_under_rfid", "determine_signal", "leave_area"}
depends_interfaces = {
   {v = "sensor", type="RobotinoSensorInterface"},
   { v="light", type="RobotinoAmpelInterface", id="light" } 
}

documentation     = [==[delivers already fetched puck to specified location]==]
-- Constants
local THRESHOLD_DISTANCE = 0.08
local DELIVERY_GATES = { "D1", "D2", "D3" }

-- Initialize as skill module
skillenv.skill_module(_M)

function have_puck()
local curDistance = sensor:distance(8)
   if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
      return true
   end
   return false
end

function ampel_green()
   return light:state() == light.GREEN
end

fsm:define_states{ export_to=_M,
   closure = {motor=motor, have_puck=have_puck, idx=fsm.vars.cur_gate_idx,
      dg=DELIVERY_GATES, ampel_green=ampel_green},
   {"CHECK_PUCK", JumpState},
   {"MOVE_UNDER_FIRST_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="SKILL_DETERMINE_SIGNAL",
      fail_to="FAILED"},
   {"SKILL_DETERMINE_SIGNAL", SkillJumpState, skills={{determine_signal}},
      final_to="DECIDE_DELIVER", fail_to="FAILED"},
   {"DECIDE_DELIVER", JumpState},
   {"MOVE_TO_NEXT", SkillJumpState, skills={{take_puck_to}}, final_to="SKILL_DETERMINE_SIGNAL",
      fail_to="FAILED"},
   {"MOVE_UNDER_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="LEAVE_AREA",
      fail_to="FAILED"},
   {"LEAVE_AREA", SkillJumpState, skills={{leave_area}}, final_to="FINAL", fail_to="FAILED"}
}
   

fsm:add_transitions{
   {"CHECK_PUCK", "FAILED", cond="not have_puck()", desc="No puck seen by Infrared"},
   {"CHECK_PUCK", "MOVE_UNDER_FIRST_RFID", cond=have_puck},
   {"DECIDE_DELIVER", "MOVE_UNDER_RFID", cond=ampel_green},
   {"DECIDE_DELIVER", "MOVE_TO_NEXT", cond="idx < #dg"},
   {"DECIDE_DELIVER", "FAILED", cond="(not ampel_green()) and (idx >= #dg)"}
}

function CHECK_PUCK:init()
   self.fsm.vars.cur_gate_idx = 1
end

function SKILL_DETERMINE_SIGNAL:init()
   self.skills[1].mode = "DELIVER"
end

function MOVE_TO_NEXT:init()
   self.fsm.vars.cur_gate_idx = self.fsm.vars.cur_gate_idx + 1
   self.skills[1].goto_name = DELIVERY_GATES[self.fsm.vars.cur_gate_idx]
end
