
----------------------------------------------------------------------------
--  approach_puk.lua
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
name               = "take_puck"
fsm                = SkillHSM:new{name=name, start="SEE_PUCK", debug=false}
depends_skills     = { "grab_puck" }
depends_interfaces = {
	 {v = "motor", type = "MotorInterface"}
--	 {v = "omnipuck", type="Position3DInterface"}
}

documentation      = [==[Move to puck pickup position]==]

-- Constants
local THRESHOLD_DISTANCE = 0.09

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

fsm:add_transitions{
	closure={motor=motor},
	{"SEE_PUCK", "FAILED", cond=no_puck, desc="No puck seen by OmniVision", precond=true},
	{"SEE_PUCK", "APPROACH_PUCK", cond=puck},
	{"APPROACH_PUCK", "SEE_PUCK", cond=no_puck_in_front, desc="Puck gone after approach"},
	{"APPROACH_PUCK", "SKILL_TAKEPUCK", cond=puck_in_front},
	{"SKILL_TAKEPUCK", "FINAL", skill=grab_puck, fail_to="SEE_PUCK"}
}

function send_transrot(vx, vy, omega)
	local oc  = motor:controller()
	local ocn = motor:controller_thread_name()
	motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
	motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
	motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

