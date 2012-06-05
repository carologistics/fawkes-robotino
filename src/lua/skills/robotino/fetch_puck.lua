
----------------------------------------------------------------------------
--  fetch_puck.lua
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
name               = "fetch_puck"
fsm                = SkillHSM:new{name=name, start="SEE_PUCK", debug=false}
depends_skills     = { "grab_puck" }
depends_interfaces = {
--	 {v = "omnipuck", type="Position3DInterface"},
--	 {v = "navigator", type="NavigatorInterface"}
}

documentation      = [==[Move to puck pickup position]==]
local puck_loc

-- Initialize as skill module
skillenv.skill_module(...)

function no_puck()
	-- TODO: stub
	return false
end

function puck()
	-- TODO: stub
	return true
end

function no_puck_in_front()
	-- TODO: stub
	return false
end

function puck_in_front()
	-- TODO: stub
	return true
end

function final_position_reached()
	-- TODO: stub
	-- return navigator:final()
end

function navigation_error()
	-- TODO: stub
	-- return not navigator:error_code() == 0
end

fsm:add_transitions{
	closure={motor=motor},
	{"SEE_PUCK", "FAILED", cond=no_puck, desc="No puck seen by OmniVision"},
	{"SEE_PUCK", "APPROACH_PUCK", cond=puck},
	{"APPROACH_PUCK", "ARRIVED", cond=final_position_reached},
	{"APPROACH_PUCK", "FAILED", cond=navigation_error, desc="Target not reachable"},
	{"ARRIVED", "SEE_PUCK", cond=no_puck_in_front, desc="Puck gone after approach"},
	{"ARRIVED", "SKILL_GRAB_PUCK", cond=puck_in_front},
	{"SKILL_GRAB_PUCK", "FINAL", skill=grab_puck, fail_to="SEE_PUCK"}
}

function SEE_PUCK:init()
	-- TODO: stub
	-- store relative puck coordinates
	-- puck_loc.x = omnipuck:translation(0)
	-- puck_loc.y = omnipuck:translation(1)
	-- puck_loc.w = omnipuck:translation(3)
end

function APPROACH_PUCK:init()
	-- TODO: stub
	-- drive to puck using navigator
end

