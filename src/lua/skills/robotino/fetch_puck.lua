
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
fsm                = SkillHSM:new{name=name, start="TURN_ON_OMNIVISION", debug=false}
depends_skills     = { "grab_puck", "relgoto", "motor_move" }
depends_interfaces = {
	 {v = "omnivisionSwitch", type="SwitchInterface", id="omnivisionSwitch"},
	 {v = "omnipuck", type="Position3DInterface", id="OmniPuck1"},
     {v = "sensor", type="RobotinoSensorInterface"}
}

documentation      = [==[Move to puck pickup position]==]
local TIMEOUT = 5
local ORI_OFFSET = 0.15
local THRESHOLD_DISTANCE = 0.1

-- Initialize as skill module
skillenv.skill_module(...)

local pm = require 'puck_loc_module'

function get_puck_loc()
	fsm.vars.puck_loc = pm.get_puck_loc(omnipuck)
	return fsm.vars.puck_loc 
end

function no_puck()
	if os.time() - fsm.vars.start_time > TIMEOUT and not fsm.vars.puck_loc then
		return true
	end
	return false
end

function puck()
	if get_puck_loc() then return true end
	return false
end

function puck_in_front()
	if get_puck_loc() and math.abs(math.atan2(fsm.vars.puck_loc.y, fsm.vars.puck_loc.x)) < ORI_OFFSET then
		return true
	end
	return false
end

function have_puck()
    local curDistance = sensor:distance(0)
    if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
        printf("near: " .. curDistance)
        return true
    end
    return false
end

function dont_have_puck()
	return not have_puck()
end

fsm:add_transitions{
	{"TURN_ON_OMNIVISION", "WAIT_FOR_VISION", cond=true },
	{"WAIT_FOR_VISION", "SEE_PUCK", wait_sec=0.333 },
	{"SEE_PUCK", "FAILED", cond=no_puck, desc="No puck found by OmniVision"},
	{"SEE_PUCK", "TURN_TO_PUCK", cond=puck, desc="Found a puck"},
	{"TURN_TO_PUCK", "ARRIVED", skill=motor_move, fail_to="SEE_PUCK"},
	{"ARRIVED", "SEE_PUCK", cond=no_puck, desc="Puck gone after approach"},
	{"ARRIVED", "GRAB_PUCK", cond=puck_in_front},
	{"GRAB_PUCK", "MOVE_DONE", skill=motor_move, fail_to="FAILED" },
	{"MOVE_DONE", "SEE_PUCK", cond=dont_have_puck },
	{"MOVE_DONE", "TURN_OFF_OMNIVISION", cond=have_puck },
	{"TURN_OFF_OMNIVISION", "FINAL", cond=true }
}

function TURN_ON_OMNIVISION:init()
	local msg = omnivisionSwitch.EnableSwitchMessage:new()
	omnivisionSwitch:msgq_enqueue_copy(msg)
	self.fsm.vars.start_time = os.time()
end

function TURN_TO_PUCK:init()
	self.args = { x=0, y=0,
		ori=math.atan2(self.fsm.vars.puck_loc.y, self.fsm.vars.puck_loc.x)
	}
end

function GRAB_PUCK:init()
	self.args = {
		x = self.fsm.vars.puck_loc.x,
		y = self.fsm.vars.puck_loc.y,
		ori = 0
	}
end

function TURN_OFF_OMNIVISION:init()
    local msg = omnivisionSwitch.EnableSwitchMessage:new()
    omnivisionSwitch:msgq_enqueue_copy(msg)	
end




