
----------------------------------------------------------------------------
--  goto.lua - 
--
--  Created: Thu Aug 14 14:32:47 2008
--  modified by Victor Mataré
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
name               = "goto"
fsm                = SkillHSM:new{name=name, start="DO_RELGOTO"}
depends_skills     = { "relgoto" }
depends_interfaces = {
	{v = "pose", type="Position3DInterface", id="Pose"}
}

documentation      = [==[Move to a known (named) location.
@param goto_name Name of the place we want to go to.
@

Available names:
================
M1-M10 = m1-m10:              Machines 1-10,
R1, R2:                       Recycling gates 1 and 2,
T = TEST = Test:              Test station,
EGI = ExpressGoodInsertion:   Express good insertion area,
D1-D3 = Delivery1-Delivery3:  Delivery gates 1-3.
]==]

local MAX_TRANSERR = 0.05
local MAX_ROTERR = 0.1

-- Initialize as skill module
skillenv.skill_module(...)

local machine_pos = require 'machine_pos_module'
local tf_mod = require 'tf_module'

fsm:add_transitions{
	{"DO_RELGOTO", "FINAL", skill=relgoto, fail_to="FAILED"},
}


function DO_RELGOTO:init()
	local x, y, ori
	if self.fsm.vars.goto_name then
		local name = self.fsm.vars.goto_name
		x = machine_pos.delivery_goto[name].x
		y = machine_pos.delivery_goto[name].y
		ori = machine_pos.delivery_goto[name].ori
	else
		x = self.fsm.vars.goto_x or pose:translation(0)
		y = self.fsm.vars.goto_y or pose:translation(1)
		ori = self.fsm.vars.goto_ori or 2*math.acos(pose:translation(3))
	end

	local rel_pos = tf_mod.transform({x = x, y = y, ori = ori}, "/map", "/base_link")

	self.args = {
		rel_x = rel_pos.x,
		rel_y = rel_pos.y,
		rel_ori = rel_pos.ori
	}
end


