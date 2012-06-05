
----------------------------------------------------------------------------
--  approach_puck.lua
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
name               = "approach_puck"
fsm                = SkillHSM:new{name=name, start="SEE_PUCK", debug=false}
depends_skills     = {}
depends_interfaces = {
	{v = "OmniPuck", type="Position3DInterface"},
	{v = "navigator", type="NavigatorInterface"}
}

documentation      = [==[Find reachable puck pickup position]==]

-- Constants
local pickup_dist = 0.3
local margin = 0.05

-- Initialize as skill module
skillenv.skill_module(...)

function puck()
        return OmniPuck:visibility_history() > -2
end

function no_puck()
        return not puck()
end

function pickup_pos_reached()
	return navigator:is_final()
end

function pickup_pos_not_reached()
	return navigator:is_final() and navigator:dest_dist() > margin
end

function puck_in_front()
	local cam_x = OmniPuck:translation(0)
	local cam_y = OmniPuck:translation(1)
	local puck_angle = math.atan2(cam_y, cam_x)
	return puck_angle > -0.01 or puck_angle < 0.01
end

function no_puck_in_front()
	return not puck_in_front()
end

fsm:add_transitions{
	{"SEE_PUCK", "GOTO_PICKUP_POS", cond=puck},
	{"SEE_PUCK", "FAILED", cond=no_puck},
	{"GOTO_PICKUP_POS", "TURN_TO_PUCK", cond=pickup_pos_reached},
	{"GOTO_PICKUP_POS", "SEE_PUCK", cond=pickup_pos_not_reached},
	{"TURN_TO_PUCK", "FINAL", cond=puck_in_front},
	{"TURN_TO_PUCK", "TURN_TO_PUCK", cond=no_puck_in_front},
	{"TURN_TO_PUCK", "FAILED", cond=no_puck}
--	{"SKILL_GRAB_PUCK", "FINAL", skill=grab_puck, fail_to="FAILED"}
}

function SEE_PUCK:init()
	local cam_x = OmniPuck:translation(0)
	local cam_y = OmniPuck:translation(1)
	local cam_d = math.sqrt(cam_x^2 + cam_y^2)
	local arg_x = math.sqrt( (cam_d - pickup_dist)^2 / (1 + (cam_y/cam_x)^2))
	local arg_y = cam_y/cam_x * arg_x
	local arg_ori = math.atan2(cam_y, cam_x)

	self.fsm.vars.puck_pickup_pos = { x=arg_x, y=arg_y, ori=arg_ori } 
end

function GOTO_PICKUP_POS:init()
	local x, y, ori = self.fsm.vars.puck_pickup_pos.x,
	        self.fsm.vars.puck_pickup_pos.y,
	        self.fsm.vars.puck_pickup_pos.ori
	printf("Sending CartesianGotoMessage(%f, %f, %f)", x, y, ori)
	local m = navigator.CartesianGotoMessage:new(x, y, ori)
	navigator:msgq_enqueue_copy(m)
end

function TURN_TO_PUCK:init()
	local cam_x = OmniPuck:translation(0)
	local cam_y = OmniPuck:translation(1)
	local m = navigator.CartesianGotoMessage:new(0, 0, math.atan2(cam_y, cam_x))
	navigator:msgq_enqueue_copy(m)
end

