----------------------------------------------------------------------------
--  store_puck.lua
--
--  Created: Sun Jun 29 15:33:49 2014
--  Copyright  2014 Frederik Zwilling
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
name               = "store_puck"
fsm                = SkillHSM:new{name=name, start="GOTO", debug=false}
depends_skills     = {"ppgoto", "motor_move", "global_motor_move"}
depends_interfaces = {
   {v = "ppnavi", type = "NavigatorInterface"},
   {v = "motor", type = "MotorInterface", id="Robotino"},
   {v = "puck_0", type="Position3DInterface", id="puck_0"}
}

documentation      = [==[
      Store a puck at the given place to pick it up later

      Parameters:
        place: name of the storage place
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

function puck_visible()
   return puck_0:visibility_history() >= 1
end

fsm:define_states{ export_to=_M,
   {"GOTO", SkillJumpState, skills={{ppgoto}}, final_to="ADJUST_POS", fail_to="FAILED"},
   {"ADJUST_POS", SkillJumpState, skills={{global_motor_move}}, final_to="LAY_DOWN", fail_to="FAILED"},
   {"LAY_DOWN", SkillJumpState, skills={{motor_move}}, final_to="BACK_UP", fail_to="FAILED"},
   {"BACK_UP", SkillJumpState, skills={{motor_move}}, final_to="LEAVE", fail_to="FAILED"},
   {"LEAVE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
}

function GOTO:init()
   if self.fsm.vars.place == nil then
      printf("Called store_puck without parameter place!")
   end
   self.skills[1].place = self.fsm.vars.place
end

function ADJUST_POS:init()
   self.skills[1].place = self.fsm.vars.place
end

function LAY_DOWN:init()
   self.skills[1].x = 0.2
   self.skills[1].vel_trans = 0.1
end

function BACK_UP:init()
   self.skills[1].x = -0.2
   self.skills[1].vel_trans = 0.1
end

function LEAVE:init()
   self.skills[1].x = -0.1
   self.skills[1].ori = 3.14
end
