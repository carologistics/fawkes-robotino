
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
	{v = "sensor", type="RobotinoSensorInterface", id = "Robotino"},
	{v = "Closest_Machine", type="Position3DInterface", id = "Closest_Machine"},
	{v = "motor", type = "MotorInterface", id="Robotino" },	
    {v = "pose", type="Position3DInterface", id="Pose"}
}

documentation      = [==[Move under the RFID Reader/Writer]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require('tf_module')
local TURN_CORRECTION = 0.13 --angle in rad
local LASER_SIDEWARDS_CORRECTION = 0.1 
local LASER_FORWARD_CORRECTION = 0.20
local LIGHT_SENSOR_DELAY_CORRECTION = 0.04
function ampel(self)
   return (self.fsm.vars.ampel_loc.distance > 0) and (self.fsm.vars.ampel_loc.distance < 1)
end

function no_ampel(self)
	return not ampel(self)
end

function see_ampel(self)
	if self.fsm.vars.ampel_loc.distance <= 0.3  then
		return true
	end
end
function position_left()
	if sensor:analog_in(0) < 1 then
		return true
	end
end
function position_right()
	if sensor:analog_in(4) < 1 then
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
	if sensor:analog_in(4) > 8 then
		send_transrot(0,0,0)
		return true
	end
end
function left_and_right_ok()
	if sensor:analog_in(0) > 8 and sensor:analog_in(4) > 8 then
		send_transrot(0,0,0)
		return true
	end
end
function is_left(self)
	if self.fsm.vars.ampel_loc.angle <= -0.07 then
		return true
	end
end	
function is_right(self)
	if self.fsm.vars.ampel_loc.angle >= 0.07  then
		return true
	end
end
function angle_reached(self)
   if ((2*math.acos(pose:rotation(3))) + TURN_CORRECTION) >=  (math.ceil(self.fsm.vars.loc_angle/(math.pi/2.0)) * (math.pi/2.0)) then
	return true
   --- falls 0 durchlaufen wird:
   elseif (2*math.acos(pose:rotation(3))) <= 0.07 then
        return true
      end
end
function angle_ok(self)
   self.fsm.vars.ampel_loc.x = Closest_Machine:translation(0)
   self.fsm.vars.ampel_loc.y = Closest_Machine:translation(1)
   self.fsm.vars.ampel_loc.distance = math.sqrt(self.fsm.vars.ampel_loc.x^2,self.fsm.vars.ampel_loc.y^2)
   self.fsm.vars.ampel_loc.angle = math.atan(self.fsm.vars.ampel_loc.x,self.fsm.vars.ampel_loc.y)
   if self.fsm.vars.ampel_loc.angle < 0.07 and self.fsm.vars.ampel_loc.angle > -0.07 then
      send_transrot(0,0,0)
      return true
   end
end

fsm:define_states{ export_to=_M,
   {"SEE_AMPEL", JumpState},
   {"SKILL_TURN_ZERO",SkillJumpState, skills={{motor_move}}, final_to="DESC_CHECK_IF_FRONT", fail_to="SEE_AMPEL"},
   {"DESC_CHECK_IF_FRONT", JumpState},
   {"CORRECT_LEFT", SkillJumpState, skills={{motor_move}}, final_to="APPROACH_AMPEL_CLOSER",
      fail_to="FAILED"},
   {"CORRECT_RIGHT", SkillJumpState, skills={{motor_move}}, final_to="APPROACH_AMPEL_CLOSER",
      fail_to="FAILED"},
   {"APPROACH_AMPEL_CLOSER", SkillJumpState, skills={{motor_move}},
      final_to="CHECK_POSITION", fail_to="FAILED"},
   {"CHECK_POSITION", JumpState},
   {"LEFT_TOO_FAR", SkillJumpState, skills={{motor_move}}, final_to="CORRECT_DELAY_RIGHT", fail_to="FAILED"},
   {"RIGHT_TOO_FAR", SkillJumpState, skills={{motor_move}}, final_to="CORRECT_DELAY_LEFT", fail_to="FAILED"},
   {"CORRECT_DELAY_RIGHT", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},--both are needed because of a delay of the sensor
   {"CORRECT_DELAY_LEFT", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"SEE_AMPEL", "FAILED", cond=no_ampel, desc="No Ampel seen with laser"},
   {"SEE_AMPEL", "SKILL_TURN_ZERO", cond=ampel, desc="Ampel seen with laser"},
   {"SKILL_TURN_ZERO", "DESC_CHECK_IF_FRONT", cond=angle_reached},
   {"DESC_CHECK_IF_FRONT", "CORRECT_LEFT", cond=is_left},
   {"DESC_CHECK_IF_FRONT", "CORRECT_RIGHT", cond=is_right},
   {"DESC_CHECK_IF_FRONT", "APPROACH_AMPEL_CLOSER", cond=angle_ok},
   {"CHECK_POSITION", "LEFT_TOO_FAR", cond=position_left},
   {"CHECK_POSITION", "RIGHT_TOO_FAR", cond=position_right},
   {"CHECK_POSITION", "FINAL", cond=left_and_right_ok},
   {"LEFT_TOO_FAR", "CORRECT_DELAY_RIGHT", cond=left_ok},
   {"RIGHT_TOO_FAR", "CORRECT_DELAY_LEFT", cond=right_ok},
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
   self.fsm.vars.ampel_loc.x = Closest_Machine:translation(0)
   self.fsm.vars.ampel_loc.y = Closest_Machine:translation(1)
   self.fsm.vars.ampel_loc.distance = math.sqrt(self.fsm.vars.ampel_loc.x^2,self.fsm.vars.ampel_loc.y^2)
   self.fsm.vars.ampel_loc.angle = math.atan(self.fsm.vars.ampel_loc.x,self.fsm.vars.ampel_loc.y)
end
function SKILL_TURN_ZERO:init()
   self.fsm.vars.loc_angle = 2*math.acos(pose:rotation(3))
   self.fsm.vars.turn_angle = math.ceil(self.fsm.vars.loc_angle/(math.pi/2.0)) * (math.pi/2.0)
   self.skills[1].ori=self.fsm.vars.turn_angle
end
function DESC_CHECK_IF_FRONT:init()
   self.fsm.vars.ampel_loc.x = Closest_Machine:translation(0)
   self.fsm.vars.ampel_loc.y = Closest_Machine:translation(1)
   self.fsm.vars.ampel_loc.distance = math.sqrt(self.fsm.vars.ampel_loc.x^2,self.fsm.vars.ampel_loc.y^2)
   self.fsm.vars.ampel_loc.angle = math.atan(self.fsm.vars.ampel_loc.x,self.fsm.vars.ampel_loc.y)
   print(self.fsm.vars.ampel_loc.angle)
end
function CORRECT_LEFT:init()
   self.skills[1].x=0 
   self.skills[1].y=self.fsm.vars.ampel_loc.y 
   self.skills[1].ori=0
end
function CORRECT_RIGHT:init()
   self.skills[1].x=0 
   self.skills[1].y=self.fsm.vars.ampel_loc.y - LASER_SIDEWARDS_CORRECTION
   self.skills[1].ori=0
end
function APPROACH_AMPEL_CLOSER:init()
   self.skills[1].x=self.fsm.vars.ampel_loc.x - LASER_FORWARD_CORRECTION
   self.skills[1].y=0 
   self.skills[1].ori=0
end
function CHECK_POSITION:init()
end
function RIGHT_TOO_FAR:init()
   self.skills[1].x=0 
   self.skills[1].y=0.3 
   self.skills[1].ori=0
end
function LEFT_TOO_FAR:init()
   self.skills[1].x=0 
   self.skills[1].y=-0.3 
   self.skills[1].ori=0
end
function CORRECT_DELAY_RIGHT:init()
   self.skills[1].x=0 
   self.skills[1].y=LIGHT_SENSOR_DELAY_CORRECTION 
   self.skills[1].ori=0
end
function CORRECT_DELAY_LEFT:init()
   self.skills[1].x=0 
   self.skills[1].y=-LIGHT_SENSOR_DELAY_CORRECTION
   self.skills[1].ori=0
end
