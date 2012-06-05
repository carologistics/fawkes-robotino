
----------------------------------------------------------------------------
--  goto.lua - generic global goto
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
name               = "grab_puck"
fsm                = SkillHSM:new{name=name, start="DRIVE", debug=false}
depends_skills     = nil
depends_interfaces = {
    {v = "motor", type = "MotorInterface"},
    {v = "sensor", type="RobotinoSensorInterface"}
}

documentation      = [==[Move forward till puck is inside arm]==]

-- Constants
local THRESHOLD_DISTANCE = 0.1

-- Initialize as skill module
skillenv.skill_module(...)

function no_writer(state)
   return not motor:has_writer()
end


function puck_is_away()
	local curDistance = sensor:distance(0)
	if not fsm.vars.checked_away then
		fsm.vars.checked_away = true
		if (curDistance > THRESHOLD_DISTANCE) or (cur_distance == 0) then
			printf("away: " .. curDistance)
			return true
		end
	end
	return false
end

function puck_is_near()
	local curDistance = sensor:distance(0)
	if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
		printf("near: " .. curDistance)
		return true
	end
	return false
end

fsm:add_transitions{
   closure={motor=motor},
   {"DRIVE", "FAILED", cond=no_writer, desc="No writer for motor", precond=true},
   {"DRIVE", "STOP", cond=puck_is_near},
   {"DRIVE", "DRIVE", cond=puck_is_away, desc="away"},
   {"STOP", "FINAL", cond=true}
}

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function DRIVE:init()
	send_transrot(0.2,0,0)
end

function STOP:init()
	send_transrot(0,0,0)
end

function DRIVE:loop()
	self.fsm.vars.checked_away = false
end

