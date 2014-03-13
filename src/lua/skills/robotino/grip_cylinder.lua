
----------------------------------------------------------------------------
--  grip_cylinder.lua
--
--  Created: Thu March 13 15:34:23 2014
--  Copyright  2014  Sebastian Reuter
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
name               = "grip_cylinder"
fsm                = SkillHSM:new{name=name, start="APPROACH", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = {
   {v = "motor", type = "MotorInterface"},
   {v = "gripper", type = "GripperInterface"}
}

documentation      = [==[Grip cylinder skill.
Drives a short distances forward,
grips a cylinder,
drives a short distance backward
and releases the cylinder.

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function no_motor_writer(state)
    return not motor:has_writer()
end

function no_gripper_writer(state)
    return not gripper:has_writer()
end

-- States
fsm:define_states{
   export_to=_M,
   {"APPROACH", SkillJumpState, skills={{motor_move}}, final_to="GRIP", fail_to="FAILED"},
   {"LEAVE", SkillJumpState, skills={{motor_move}}, final_to="RELEASE", fail_to="RELEASE"},
   {"GRIP", JumpState},
   {"RELEASE", JumpState}
}

-- Transitions
fsm:add_transitions{
   {"APPROACH", "FAILED", precond=no_motor_writer, desc="No writer for interface"},
   {"GRIP", "FAILED", precond=no_gripper_writer, desc="No writer for interface"},
   {"GRIP", "LEAVE", cond=true},
   {"RELEASE", "FINAL", cond=true}
}

function APPROACH:init()
   self.skills[1].x=0.2
   self.skills[1].y=0
   self.skills[1].ori=0
end

function LEAVE:init()
   self.skills[1].x=-0.2
   self.skills[1].y=0
   self.skills[1].ori=0
end

function GRIP:init()
   gripper:msgq_enqueue_copy( gripper.CloseGripperMessage:new() )
end

function RELEASE:init()
   gripper:msgq_enqueue_copy( gripper.OpenGripperMessage:new() )
end
