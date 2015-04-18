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
fsm                = SkillHSM:new{name=name, start="DRIVETO", debug=false}
depends_skills     = {"take_puck_to", "motor_move", "global_motor_move", "drive_to"}
depends_interfaces = {
   {v = "ppnavi", type = "NavigatorInterface"},
   {v = "motor", type = "MotorInterface", id="Robotino"},
   {v = "puck_0", type="Position3DInterface", id="puck_0"},
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"},
   {v = "line5", type="LaserLineInterface", id="/laser-lines/5"},
   {v = "line6", type="LaserLineInterface", id="/laser-lines/6"},
   {v = "line7", type="LaserLineInterface", id="/laser-lines/7"},
   {v = "line8", type="LaserLineInterface", id="/laser-lines/8"}

}

documentation      = [==[
      Store a puck at the given place to pick it up later

      Parameters:
        place: name of the storage place
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
MIN_VIS_HIST = 10

local lines = {
   line1,
   line2,
   line3,
   line4,
   line5,
   line6,
   line7,
   line8
}

local candidates = {}

function get_closest_line()
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST then
         table.insert(candidates, o)
      end
   end

   local best_line
   for _,o in ipairs(candidates) do
      local min_ori = 10
      -- get the closest line to the given angle
      if o:bearing() < min_ori then
         min_ori = o:bearing()
         best_line = o
      end
   end
   
   return best_line
end

function visible_and_writer()
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST and o:has_writer() then
         return true
      end
   end
   return false
end

function puck_visible()
   return puck_0:visibility_history() >= 1
end

fsm:define_states{ export_to=_M,closure={visible_and_writer=visible_and_writer},
   {"DRIVETO", SkillJumpState, skills={{drive_to}}, final_to="DECIDE", fail_to="FAILED"},
   {"DECIDE", JumpState},
   {"LAY_DOWN", SkillJumpState, skills={{motor_move}}, final_to="BACK_UP", fail_to="FAILED"},
   {"BACKUP_LAY_DOWN", SkillJumpState, skills={{motor_move}}, final_to="BACK_UP", fail_to="FAILED"},
   {"BACK_UP", SkillJumpState, skills={{motor_move}}, final_to="LEAVE", fail_to="FAILED"},
   {"LEAVE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"DECIDE", "LAY_DOWN", cond=visible_and_writer},
   {"DECIDE", "BACKUP_LAY_DOWN", cond="not visible_and_writer()"}
}

function DRIVETO:init()
   if self.fsm.vars.place == nil then
      printf("Called store_puck without parameter place!")
   end
   self.skills[1].puck = true
   self.skills[1].place = self.fsm.vars.place
end

function LAY_DOWN:init()
   local best_line = get_closest_line()
   self.skills[1].x = best_line:point_on_line(0) - 0.25
end

function BACKUP_LAY_DOWN:init()
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
