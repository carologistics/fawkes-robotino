
----------------------------------------------------------------------------
--  move_under_rfid.lua
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
name               = "move_under_rfid"
fsm                = SkillHSM:new{name=name, start="SEE_AMPEL", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = {
	{v = "Machine_0", type="Position3DInterface",id = "Machine_0"}
}

documentation      = [==[Move under the RFID Reader/Writer]==]

-- Initialize as skill module
skillenv.skill_module(...)

function ampel()
	distance = math.sqrt((Machine_0:translation(0))^2+(Machine_0:translation(1))^2)
	return (distance > 0)
end

function no_ampel()
	return not ampel()
end



fsm:add_transitions{
	closure={motor=motor},
	{"SEE_AMPEL", "FAILED", cond=no_ampel, desc="No Ampel seen with laser"},
	{"SEE_AMPEL", "SKILL_TURN_TO_AMPEL", cond=ampel, desc="Ampel seen with laser"},
	{"SKILL_TURN_TO_AMPEL", "SKILL_APPROACH_AMPEL_CLOSER", skill=motor_move, fail_to="FAILED"},
	{"SKILL_APPROACH_AMPEL_CLOSER", "SKILL_TURN_TO_AMPEL_CLOSER", skill=motor_move, fail_to="FAILED"},
	{"SKILL_TURN_TO_AMPEL_CLOSER", "SKILL_APPROACH_AMPEL", skill=motor_move, fail_to="FAILED"},
	{"SKILL_APPROACH_AMPEL", "FINAL", skill=motor_move, fail_to="FAILED"}
}

function SEE_AMPEL:init()
	-- TODO: stub
	-- store relative puck coordinates
	self.fsm.vars.ampel_loc = {}
	self.fsm.vars.ampel_loc.x = Machine_0:translation(0)
	self.fsm.vars.ampel_loc.y = Machine_0:translation(1)
	self.fsm.vars.ampel_loc.angle = math.atan(self.fsm.vars.ampel_loc.y/self.fsm.vars.ampel_loc.x)
end
function SKILL_TURN_TO_AMPEL:init()
	self.args = {x=0,y=0,ori=self.fsm.vars.ampel_loc.angle}
end
function SKILL_APPROACH_AMPEL_CLOSER:init()
	self.args = {x=math.sqrt((Machine_0:translation(0))^2+(Machine_0:translation(1))^2)-(0.283+0.1),y=0,ori=0}
end
function SKILL_TURN_TO_AMPEL_CLOSER:init()
	self.args = {x=0,y=0,ori=math.atan(Machine_0:translation(1)/Machine_0:translation(0))}
end
function SKILL_APPROACH_AMPEL:init()
	self.args = {x=math.sqrt((Machine_0:translation(0))^2+(Machine_0:translation(1))^2)-(0.278)}
end
