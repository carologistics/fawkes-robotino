
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
fsm                = SkillHSM:new{name=name, start="START_RELGOTO"}
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

local MAX_TRANSERR = 0.16
local MAX_ROTERR = 0.16
local MAX_POS_MISS = 4
-- Initialize as skill module
skillenv.skill_module(_M)

local machine_pos = require 'machine_pos_module'
local tf_mod = require 'tf_module'


function pose_ok(self)
	return (math.abs(self.fsm.vars.goto_x - pose:translation(0)) <= MAX_TRANSERR
	 and math.abs(self.fsm.vars.goto_y - pose:translation(1)) <= MAX_TRANSERR
	 and math.abs(self.fsm.vars.goto_ori - 2*math.acos(pose:rotation(3))) <= MAX_ROTERR)
end

function pose_not_ok(self)
	return not pose_ok(self)
end

function missed_to_often(self)
 return (self.fsm.vars.num_pos_missed>= MAX_POS_MISS)
end

function not_missed_to_often(self)
 return not missed_to_often(self)
end

fsm:define_states{ export_to=_M,
   {"START_RELGOTO", JumpState},
   {"DO_RELGOTO", SkillJumpState, skills=relgoto, final_to="WAIT_POSE", fail_to="FAILED"},
   {"WAIT_POSE", JumpState},
   {"CHECK_POSE", JumpState},
   {"MISSED", JumpState}
}


fsm:add_transitions{
  {"START_RELGOTO","DO_RELGOTO",cond=true},
	{"WAIT_POSE", "CHECK_POSE", timeout=3.0},
	{"CHECK_POSE", "FINAL", cond=pose_ok, desc="Pose reached" },
	{"CHECK_POSE", "MISSED", cond=pose_not_ok, },
  {"MISSED", "DO_RELGOTO", cond=not_missed_to_often},
  {"MISSED", "FAILED", cond=missed_to_often, desc="Posed missed"}
}

function START_RELGOTO:init()
  self.fsm.vars.num_pos_missed=0
end

function MISSED:init()
  self.fsm.vars.num_pos_missed=self.fsm.vars.num_pos_missed+1
end

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

	self.fsm.vars.goto_x = x
	self.fsm.vars.goto_y = y
	self.fsm.vars.goto_ori = ori

	local rel_pos = tf_mod.transform({x = x, y = y, ori = ori}, "/map", "/base_link")

	self.args = {
		rel_x = rel_pos.x,
		rel_y = rel_pos.y,
		rel_ori = rel_pos.ori
	}
end


