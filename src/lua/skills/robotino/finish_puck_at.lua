
----------------------------------------------------------------------------
--  take_puck_to.lua
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
name               = "finish_puck_at"
fsm                = SkillHSM:new{name=name, start="SKILL_TAKE_PUCK", debug=true}
depends_skills     = { "take_puck_to", "determine_signal", "deposit_puck", "leave_area", "deliver" }
depends_interfaces = { }

documentation      = [==[Go to target without losing teh puck]==]

-- Initialize as skill module
skillenv.skill_module(...)

fsm:add_transitions{
	{ "SKILL_TAKE_PUCK", "DECIDE_ENDSKILL", skill=take_puck_to, fail_to="FAILED" },
	{ "DECIDE_ENDSKILL", "SKILL_RFID", cond=end_rfid, desc="move under rfid" },
	{ "DECIDE_ENDSKILL", "SKILL_DELIVER", cond=end_deliver, desc="deliver" },
	{ "SKILL_RFID", "SKILL_DETERMINE_SIGNAL", skill=move_under_rfid, fail_to="FAILED" },
	{ "SKILL_DETERMINE_SIGNAL", "SKILL_DEPOSIT", skill=determine_signal, fail_to="FAILED" },
	{ "SKILL_DEPOSIT", "SKILL_LEAVE", skill=deposit_puck, fail_to="FAILED" },
	{ "SKILL_LEAVE", "FINAL", skill=leave_area, fail_to="FAILED" },
	{ "SKILL_DELIVER", "FINAL", skill=deliver, fail_to="FAILED" }
}

function SKILL_TAKE_PUCK:init()
	self.args = { goto_name = self.fsm.vars.goto_name }
end


