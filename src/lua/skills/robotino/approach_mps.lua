----------------------------------------------------------------------------
--  approach_mps.lua
--
--  Created Sun Jun 26 11:44:40 2016
--  Copyright  2016  Frederik Zwilling
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
name               = "approach_mps"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = { 
   {v = "if_front_dist", type = "Position3DInterface", id="front_dist"}

}

documentation      = [==[
                        The robot just drives forward according to the current distance to the mps and the desired position
                        @param "x" int The x distance (laser-mps) to the MPS when finished
                     ]==]


-- Initialize as skill module
skillenv.skill_module(_M)

local x_to_drive = 0

function mps_is_near()
   if if_front_dist:visibility_history() > 0 and x_to_drive < 1.0 then
      return true
   else
      printf("mps_approach failed: visibility history is %f, dist to object in front is %f. I don't drive with this visibility history or this far without collision avoidance", if_front_dist:visibility_history(), x_to_drive)
      return false
   end
end


fsm:define_states{ export_to=_M, closure={mps_is_near=mps_is_near},
   {"INIT", JumpState},
   {"APPROACH", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "APPROACH", cond=mps_is_near},
   {"INIT", "FAILED", timeout=0.5}
}

function INIT:init()
   x_to_drive = if_front_dist:translation(0) - self.fsm.vars.x
end

function APPROACH:init()
   printf("x is set to: %f", self.fsm.vars.x)
   x_to_drive = if_front_dist:translation(0) - self.fsm.vars.x
   self.args["motor_move"] = {x = self.fsm.vars.x, vel_trans = 0.2, frame="front_dist"}
end

function FAILED:init()
   printf("mps_approach failed (maybe because of subskill): visibility history is %f, dist to object in front is %f.", if_front_dist:visibility_history(), if_front_dist:translation(0))
end
