
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
depends_skills     = { "grab_puck", "relgoto", "motor_move" }
depends_interfaces = {
	 {v = "omnipuck", type="Position3DInterface", id="Omnipuck1"},
	 {v = "pose", type="Position3DInterface", id="Pose"},
	 {v = "navigator", type="NavigatorInterface", id="Navigator"}
}

documentation      = [==[Move to puck pickup position]==]
local TIMEOUT = 5
local ORI_OFFSET = 0.15

-- Initialize as skill module
skillenv.skill_module(...)

function get_puck_loc()
	if omnipuck:visibility_history() >= 5 then
		fsm.vars.puck_loc = {
			x = omnipuck.translation(0)
			y = omnipuck.translation(1)
		}
		return true
	end
	return false
end

function no_puck()
	if os.time() - fsm.vars.start_time > TIMEOUT and not fsm.vars.puck_loc then
		return true
	end
	return false
end

function puck()
	return get_puck_loc()
end

function puck_in_front()
	if get_puck_loc() and math.abs(math.atan2(fsm.vars.puck_loc.y, fsm.vars.puck_loc.x)) < ORI_OFFSET then
		return true
	end
	return false
end

fsm:add_transitions{
	closure={motor=motor},
	{"SEE_PUCK", "FAILED", cond=no_puck, desc="No puck seen by OmniVision"},
	{"SEE_PUCK", "APPROACH_PUCK", cond=puck},
	{"APPROACH_PUCK", "ARRIVED", skill=relgoto, fail_to="SEE_PUCK"},
	{"APPROACH_PUCK", "FAILED", cond=navigation_error, desc="Target not reachable"},
	{"ARRIVED", "SEE_PUCK", cond=no_puck, desc="Puck gone after approach"},
	{"ARRIVED", "SKILL_GRAB_PUCK", cond=puck_in_front},
	{"SKILL_GRAB_PUCK", "FINAL", skill=grab_puck, fail_to="FAILED"}
}

function SEE_PUCK:init()
	self.fsm.vars.start_time = os.time()
end

function APPROACH_PUCK:init()
	local ori = 2*math.acos(pose:rotation(3))
	if math.abs(ori) < ORI_OFFSET
		or math.abs(ori - math.pi) < ORI_OFFSET then
		self.args.rel_y = self.fsm.puck_loc.y
	elseif math.abs(ori - 0.5*math.pi) < ORI_OFFSET
		or math.abs(ori - 1.5*math.pi) < ORI_OFFSET then
		self.args.rel_x = self.fsm.puck_loc.x
	end
end

function ARRIVED:init()
	self.fsm.vars.start_time = os.time()
end

function SKILL_GRAB_PUCK:init()
	self.args.puck_loc = self.fsm.vars.puck_loc
end


