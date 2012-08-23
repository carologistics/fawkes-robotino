
----------------------------------------------------------------------------
--  deliver.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
----------------------------------------------------------------------------

--  This program is free software; you can reistribute it and/or modify
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
name               = "deliver"
fsm                = SkillHSM:new{name=name, start="MOVE_UNDER_D1", debug=true}
depends_skills     = {"move_under_rfid","determine_signal","leave_area","move_to_next_deliver"}
depends_interfaces = {
	{ v="light",type ="RobotinoAmpelInterface", id ="light" } 
}

documentation     = [==[delivers puck to active delivery goal]==]
-- Constants

-- Initialize as skill module
skillenv.skill_module(...)

function signal_green()
	return (light:state() ==light.GREEN)
end


fsm:add_transitions{
	closure={motor=motor},
  {"MOVE_UNDER_D1","CHECK_D1", skill=move_under_rfid, fail_to="FAILED"},
  {"CHECK_D1","IS_CHECKED_D1", skill= determine_signal, fail_to="FAILED"},
  {"IS_CHECKED_D1","DELIVERED", cond = signal_green, desc ="1st green"},
  {"IS_CHECKED_D1","MOVE_TO_D2", cond = true},
  {"MOVE_TO_D2", "MOVE_UNDER_D2", skill=move_to_next_deliver, fail_to="FAILED"},
  {"MOVE_UNDER_D2","CHECK_D2", skill=move_under_rfid, fail_to="FAILED"},
  {"CHECK_D2","IS_CHECKED_D2", skill= determine_signal, fail_to="FAILED"},
  {"IS_CHECKED_D2","DELIVERED", cond = signal_green, desc = "2nd green"},
  {"IS_CHECKED_D2","MOVE_TO_D3", cond = true},
  {"MOVE_TO_D3", "MOVE_UNDER_D3", skill=move_to_next_deliver, fail_to="FAILED"},
  {"MOVE_UNDER_D3","CHECK_D3", skill=move_under_rfid, fail_to="FAILED"},
  {"CHECK_D3","IS_CHECKED_D3", skill= determine_signal, fail_to="FAILED"},
  {"IS_CHECKED_D3","DELIVERED", cond = signal_green, fail_to="FAILED",desc = "3rd green"},
  {"IS_CHECKED_D3","FAILED",cond = true,desc = "alle ampeln aus?"},
  {"DELIVERED","FINAL", skill=leave_area, fail_to="FAILED"}
}

