
----------------------------------------------------------------------------
--  move_to_next_deliver.lua
--
--  Created: Thu Aug 14 14:32:47 2012
--  Copyright  2012 Daniel Ewert, Andre Burghof
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
name               = "move_to_next_deliver"
fsm                = SkillHSM:new{name=name, start="ROTATE_RIGHT", debug=true}
depends_skills     = {"motor_move"}

documentation     = [==[moves blindly to the next light of the delivery zone]==]
-- Constants

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"ROTATE_RIGHT", SkillJumpState, final_to="STRAFE_RIGHT", skills=motor_move, fail_to="FAILED"},
   {"STRAFE_RIGHT", SkillJumpState, final_to="ROTATE_LEFT", skills=motor_move, fail_to="FAILED"},
   {"ROTATE_LEFT", SkillJumpState, final_to="FINAL", skills=motor_move, fail_to="FAILED"}
}

function ROTATE_RIGHT:init()
   self.args = {x=0,y=0,ori=-(math.pi/9)}
end

function STRAFE_RIGHT:init()
   self.args = {x=0,y=-0.35,ori=0}
end

function ROTATE_LEFT:init()
 self.args = {x=0,y=-0,ori=(math.pi/9)}
end

