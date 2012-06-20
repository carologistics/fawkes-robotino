
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
	{v = "Machine_0", type="Position3DInterface",id = "Machine_0"},
	{v = "sensor", type="RobotinoSensorInterface"},
	{v = "motor", type = "MotorInterface", id="Robotino" }	
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

function see_ampel()
	if sensor:analog_in(0) > 8 or sensor:analog_in(1) >8 then
		return true
	end
end
function position_left()
	if sensor:analog_in(0) < 1 then
		return true
	end
end
function position_right()
	if sensor:analog_in(1) < 1 then
		return true
	end
end
function left_ok() -- if coming from left, when the left sensor jumps
	if sensor:analog_in(0) > 8 then
		send_transrot(0,0,0)
		return true
	end
end
function right_ok()
	if sensor:analog_in(1) > 8 then
		send_transrot(0,0,0)
		return true
	end
end
function left_and_right_ok()
	if sensor:analog_in(0) > 8 and sensor:analog_in(1) > 8 then
		send_transrot(0,0,0)
		return true
	end
end
function is_left()
	angle = math.atan(Machine_0:translation(1)/Machine_0:translation(0))
	if angle <= -0.05 then
		return true
	end
end	
function is_right()
	angle = math.atan(Machine_0:translation(1)/Machine_0:translation(0))
	if angle >= 0.05  then
		return true
	end
end
function angle_ok()
	angle = math.atan(Machine_0:translation(1)/Machine_0:translation(0))
	if angle < 0.05 and angle > -0.3 then
		send_transrot(0,0,0)
		return true
	end
end
fsm:add_transitions{
	closure={motor=motor},
	{"SEE_AMPEL", "FAILED", cond=no_ampel, desc="No Ampel seen with laser"},
	{"SEE_AMPEL", "DESC_CHECK_IF_FRONT", cond=ampel, desc="Ampel seen with laser"},
	{"DESC_CHECK_IF_FRONT", "CORRECT_LEFT", cond=is_left},
	{"DESC_CHECK_IF_FRONT", "CORRECT_RIGHT", cond=is_right},
	{"DESC_CHECK_IF_FRONT", "APPROACH_AMPEL_CLOSER", cond=angle_ok},
	{"CORRECT_LEFT", "APPROACH_AMPEL_CLOSER", skill=motor_move,fail_to="FAILED"},
	{"CORRECT_LEFT", "APPROACH_AMPEL_CLOSER", cond=angle_ok},
	{"CORRECT_RIGHT", "APPROACH_AMPEL_CLOSER", skill=motor_move,fail_to="FAILED"},
	{"CORRECT_RIGHT", "APPROACH_AMPEL_CLOSER", cond=angle_ok},
	--{"SKILL_TURN_TO_AMPEL", "APPROACH_AMPEL_CLOSER", skill=motor_move, fail_to="FAILED"},
	{"APPROACH_AMPEL_CLOSER", "SKILL_APPROACH_AMPEL", skill=motor_move,fail_to="FAILED"},
	{"APPROACH_AMPEL_CLOSER", "SKILL_APPROACH_AMPEL", cond=see_ampel},
	--{"SKILL_TURN_TO_AMPEL_CLOSER", "SKILL_APPROACH_AMPEL", skill=motor_move, fail_to="FAILED"},
	--{"SKILL_APPROACH_AMPEL", "CHECK_POSITION", skill=motor_move, fail_to="FAILED"},
	{"SKILL_APPROACH_AMPEL", "CHECK_POSITION", skill=motor_move, fail_to="FAILED"},
	{"CHECK_POSITION", "LEFT_TOO_FAR", cond=position_left},
	{"CHECK_POSITION", "RIGHT_TOO_FAR", cond=position_right},
	{"CHECK_POSITION", "FINAL", cond=left_and_right_ok},
	{"LEFT_TOO_FAR", "FINAL", skill=motor_move,fail_to="FAILED"},
	{"LEFT_TOO_FAR", "FINAL", cond=left_ok},
	{"RIGHT_TOO_FAR", "FINAL", skill=motor_move,fail_to="FAILED"},
	{"RIGHT_TOO_FAR", "FINAL", cond=right_ok},
}
function send_transrot(vx, vy, omega)
	local oc  = motor:controller()
	local ocn = motor:controller_thread_name()
	motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
	motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
	motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end
function SEE_AMPEL:init()
	self.fsm.vars.ampel_loc = {}
	self.fsm.vars.ampel_loc.x = Machine_0:translation(0)
	self.fsm.vars.ampel_loc.y = Machine_0:translation(1)
	self.fsm.vars.ampel_loc.angle = math.atan(self.fsm.vars.ampel_loc.y/self.fsm.vars.ampel_loc.x)
end
function DESC_CHECK_IF_FRONT:init()
end
function CORRECT_LEFT:init()
	self.args = {x=0,y=-1,ori=0}
end
function CORRECT_RIGHT:init()
	self.args = {x=0,y=1,ori=0}
end
--function SKILL_TURN_TO_AMPEL:init()
--	self.args = {x=0,y=0,ori=self.fsm.vars.ampel_loc.angle}
--end
function APPROACH_AMPEL_CLOSER:init()
	self.args = {x=1,y=0,ori=0}
end
--function SKILL_TURN_TO_AMPEL_CLOSER:init()
--	self.args = {x=0,y=0,ori=math.atan(Machine_0:translation(1)/Machine_0:translation(0))}
--end
function SKILL_APPROACH_AMPEL:init()
	self.args = {x=0.13,y=0,ori=0}
end
function CHECK_POSITION:init()
end
function RIGHT_TOO_FAR:init()
	self.args = {x=0,y=1,ori=0}
end
function LEFT_TOO_FAR:init()
	self.args = {x=0,y=-1,ori=0}
end
