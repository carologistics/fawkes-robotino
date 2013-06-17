
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
name               = "deposit_puck"
fsm                = SkillHSM:new{name=name, start="DESC_DIRECTION", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = {
   -- TODO {v = "worldModel", type="WorldModel"}
}

documentation      = [==[deposits the used puck at the side of the traffic light]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function no_puck()
   --return worldModel:numberOfPucks=0
   return false
end


function one_left()
   --pucksLeft = worldModel:getNumberOfPucks(right,machine)
   --return (pucksLeft == 1)
   return true
end

fsm:define_states{export_to=_M,
   {"DESC_DIRECTION", JumpState},
   {"SKILL_DRIVE_LEFT", SkillJumpState, skills={{motor_move}}, final_to="SKILL_DRIVE_FORWARD",
      fail_to="FAILED"},
   {"SKILL_DRIVE_RIGHT", SkillJumpState, skills={{motor_move}}, final_to="SKILL_DRIVE_FORWARD",
      fail_to="FAILED"},
   {"SKILL_DRIVE_FORWARD", SkillJumpState, skills={{motor_move}}, final_to="SKILL_DRIVE_BACKWARD",
      fail_to="FAILED"},
   {"SKILL_DRIVE_BACKWARD", SkillJumpState, skills={{motor_move}}, final_to="FINAL",
      fail_to="FAILED"}
}

fsm:add_transitions{
   {"DESC_DIRECTION", "SKILL_DRIVE_LEFT", cond=no_puck, desc="No Puck in storage"},
   {"DESC_DIRECTION", "SKILL_DRIVE_RIGHT", cond=one_left},
}

function SKILL_DRIVE_LEFT:init() 
   self.skills[1].x=0 
   self.skills[1].y=0.23 
   self.skills[1].ori=0
end

function SKILL_DRIVE_RIGHT:init()
   if self.fsm.vars.mtype == nil then
      self.skills[1].y = -0.23 
   elseif self.fsm.vars.mtype == "deliver" then
      print("drive right at delivery")
      self.skills[1].y = -0.21
   end 
end

function SKILL_DRIVE_FORWARD:init()
   if self.fsm.vars.mtype == nil then
      self.skills[1].x = 0.1 
   elseif self.fsm.vars.mtype == "deliver" then
      print("drive forward at delivery")
      self.skills[1].x = 0.08
   end 
end

function SKILL_DRIVE_BACKWARD:init()
   self.skills[1].x=-0.25 
 --TODO UPDATE WORLD MODEL WENN FINAL
end

