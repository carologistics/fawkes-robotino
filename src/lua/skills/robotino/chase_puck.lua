
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
name               = "chase_puck"
fsm                = SkillHSM:new{name=name, start="SEE_PUCK", debug=true}
depends_skills     = { "grab_puck" }
depends_interfaces = {
	{v = "OmniPuck1", type="Position3DInterface",id = "OmniPuck1"},
	{v = "sensor", type="RobotinoSensorInterface"},
	{v = "motor", type="MotorInterface"}
}

documentation      = [==[Move to puck pickup position]==]

-- Initialize as skill module
skillenv.skill_module(...)

function no_puck()
	print("OmniPuck1_visibiltiy_historty: ".. (OmniPuck1:visibility_history()))

	
	return OmniPuck1:visibility_history() < 10
end

function puck()
	return OmniPuck1:visibility_history() >=10
	
end

function no_puck_in_front()
	-- TODO: stub
	return false
end

function puck_in_front()
	-- TODO: stub
	return true
end

function final_position_reached()
	-- TODO: stub
	-- return navigator:final()
end

function gyro_angle_turned()
	
	local  a = correct_angle(sensor:gyro_angle()) - correct_angle(fsm.vars.gyro_start_angle)
	
	return correct_angle(a)
end

function gyro_rot_reached()
	printf("observed puck_angle=" .. fsm.vars.puck_loc.angle .. " angle turned um=" .. gyro_angle_turned())
	return math.abs( fsm.vars.puck_loc.angle  - gyro_angle_turned()) < 0.08
end

function gyro_not_yet_reached()
	return not math.abs( fsm.vars.puck_loc.angle  - gyro_angle_turned()) < 0.08
end



function puck_in_range()

	return sensor:distance(0) > 0


end

fsm:add_transitions{
	closure={motor=motor},
	{"SEE_PUCK", "FAILED", cond=no_puck, desc="No puck seen by OmniVision"},
	{"SEE_PUCK", "TURN_TO_PUCK", cond=puck},
	{"TURN_TO_PUCK", "APPROACH_PUCK", cond=gyro_rot_reached},
	{"TURN_TO_PUCK", "TURN_TO_PUCK", cond=gyro_not_yet_reached},
	
	
	{"APPROACH_PUCK", "SKILL_GRAB_PUCK", cond=puck_in_range},
	{"SKILL_GRAB_PUCK", "FINAL", skill=grab_puck, fail_to="FAILED"}
}

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function correct_angle(a)
	a = a % (2 * math.pi)
	if a < 0 then
		return 2 * math.pi + a
	end
	return a
end

function SEE_PUCK:init()
	-- TODO: stub
	-- store relative puck coordinates
	self.fsm.vars.puck_loc = {}
	self.fsm.vars.gyro_start_angle = correct_angle(sensor:gyro_angle())

if  Omnipuck1 == nil then
print("nil")
else
print("not nil")
end	
	
	
	self.fsm.vars.puck_loc.x = OmniPuck1:translation(0)
	self.fsm.vars.puck_loc.y = OmniPuck1:translation(1)
	print(OmniPuck1:translation(0))
	print(OmniPuck1:translation(1))
	self.fsm.vars.puck_loc.angle = 
		correct_angle(math.atan2(
			self.fsm.vars.puck_loc.y, self.fsm.vars.puck_loc.x))
		
end

function APPROACH_PUCK:init()
	send_transrot(0.2, 0, 0)
end

function TURN_TO_PUCK:init()
	send_transrot(0, 0, 0)
	if self.fsm.vars.puck_loc.angle > math.pi then
		send_transrot(0, 0, -0.05)
	else
		send_transrot(0, 0, 0.05)
	end
end


