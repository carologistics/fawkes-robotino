
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
name               = "take_puck_to"
fsm                = SkillHSM:new{name=name, start="SKILL_GOTO", debug=false}
--depends_skills     = { "goto", "relgoto" }
depends_skills     = { "relgoto" }
depends_interfaces = {
	{ v = "sensor", type = "RobotinoSensorInterface", id = "Robotino" },
	{ v = "omnivisionSwitch", type = "SwitchInterface", id = "omnivisionSwitch" },
	{ v = "omnipuck1", type = "Position3DInterface", id = "OmniPuck1" },
	{ v = "omnipuck2", type = "Position3DInterface", id = "OmniPuck2" },
	{ v = "omnipuck3", type = "Position3DInterface", id = "OmniPuck3" },
	{ v = "omnipuck4", type = "Position3DInterface", id = "OmniPuck4" },
	{ v = "omnipuck5", type = "Position3DInterface", id = "OmniPuck5" },
}

documentation      = [==[Go to target without losing teh puck]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Constants
local AVG_LEN = 10
local FIND_TIMEOUT = 5
local omnipucks = { omnipuck1 omnipuck2 omnipuck3 omnipuck4 omnipuck5 }

-- Imports
local pm = require 'puck_loc_module'
local machine_pos = require 'machine_pos_module'
local tf = require 'tf_module'

function lost_puck()
	local val = fsm.vars.avg_val
	local idx = fsm.vars.avg_idx

	val[idx] = sensor:distance(0)
	idx = idx + 1
	if idx > AVG_LEN then idx = 1 end

	local sum = 0
	local count = 0
	for i,v in ipairs(val) do
		sum = sum + v
		count = count + 1
	end
	local avg = sum / count
	printf("moving avg: %f", avg)

	fsm.vars.avg_val = val
	fsm.vars.avg_idx = idx

	if avg > 0.075 or avg == 0 then return true end
	return false
end

fsm:add_transitions{
	{ "SKILL_GOTO", "FINAL", skill=relgoto, fail_to="FAILED" },
	{ "SKILL_GOTO", "LOST_PUCK", cond=lost_puck },
	{ "LOST_PUCK", "TURN_ON_OMNIVISION", skill=relgoto, fail_to="FAILED" },
	{ "TURN_ON_OMNIVISION", "LOCATE_PUCK", cond=true },
}

function SKILL_GOTO:init()
	self.args = {
		rel_x = self.fsm.vars.rel_x,
		rel_y = self.fsm.vars.rel_y,
		rel_ori = self.fsm.vars.rel_ori
	}
	self.fsm.vars.avg_idx = 1
	self.fsm.vars.avg_val = {}
end

function LOST_PUCK:init()
	self.args = {
		rel_x = 0,
		rel_y = 0,
		rel_ori = 0
	}
end

function TURN_ON_OMNIVISION:init()
	local msg = omnivisionSwitch.EnableSwitchMessage:new()
	omnivisionSwitch:msgq_enqueue_copy(msg)
end

function LOCATE_PUCK:init()
	local p = find_best_puck()
	if p then
		
	end
end

function find_best_puck()
	local p_rel, p_loc, d
	local min_d = 1000

	for i,op in ipairs(omnipucks) do
		p_rel = pm.puck_loc(op)

		if p_rel then

			p_loc = tf.transform(
				{
					x = p_rel.x,
					y = p_rel.y,
					ori = math.atan2(p_rel.y, p_rel.x)
				}, "/base_link", "/map")

			if p_loc.x == 0 or p_loc.y == 0 then break end

			for k,field in pairs(machine_pos.fields) do
				if not ( (x > field.x and x < field.x + field.size)
				 and (y > field.y and y < field.y + field.size) ) then
					return p_loc
				end
			end
		end
	end

	return nil
end

