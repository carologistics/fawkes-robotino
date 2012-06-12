
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
name               = "relgoto"
fsm                = SkillHSM:new{name=name, start="CHECK_INPUT", debug=true}
depends_skills     = { }
depends_interfaces = {
	{v = "navigator", type="NavigatorInterface", id="Navigator"}
}

documentation      = [==[Move to puck pickup position]==]

-- Initialize as skill module
skillenv.skill_module(...)

function can_navigate()
	return navigator:has_writer()
end

function cannot_navigate()
	return not can_navigate()
end

function target_reached()
	return navigator:is_final()
end

function target_not_reached()
	return false
end

fsm:add_transitions{
	{ "CHECK_INPUT", "FAILED", cond=cannot_navigate },
	{ "CHECK_INPUT", "MOVING", cond=can_navigate },
	{ "MOVING", "FINAL", cond=target_reached },
	{ "MOVING", "FAILED", cond=target_not_reached }
}

function MOVING:init()
	send_navmsg(self.fsm.vars.rel_x, self.fsm.vars.rel_y, self.fsm.vars.rel_ori)
end

function send_navmsg(x, y, ori)
	local msg = navigator.CartesianGotoMessage:new(x, y, ori)
	navigator:msgq_enqueue_copy(msg)
end


