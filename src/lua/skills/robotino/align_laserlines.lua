
----------------------------------------------------------------------------
--  align_laserlines.lua - Align to a line detected by laserlines plugin
--
--  Created: Thu Jul 3
--  Copyright  2014  Bahram Maleki-Fard
--                   Tim Niemueller
--                   Johannes Rothe
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
name               = "align_laserlines"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = {
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"}
}

documentation      = [==[Align to straight line in laser data.]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
MIN_VIS_HIST = 10

-- Variables
local lines = {
   line1,
   line2,
   line3,
   line4,
}

local candidates = {}
local closest_line

-- Functions
function visible()
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST then
         return true
      end
   end
   return false
end


-- States
fsm:define_states{
   export_to=_M,
   closure={visible=visible},
   {"INIT",     JumpState},
   {"ALIGN", SkillJumpState, skills={{"motor_move"}}, final_to="FINAL", fail_to="FAILED"}
}

-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", precond="not visible()", desc="no writer"},
   {"INIT", "ALIGN", cond=true, desc="initialized"}
}

function INIT:init()
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST then
         table.insert(candidates, o)
      end
   end

   local min_ori = 10
   for _,o in ipairs(candidates) do
      local ori = math.abs(o:bearing())
      if ori < min_ori then
         min_ori = ori
         closest_line = o
      end
   end
end

function ALIGN:init()
   self.args["motor_move"] = {x=0, y=0, ori=closest_line:bearing()}
end
