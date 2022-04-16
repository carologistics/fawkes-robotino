----------------------------------------------------------------------------
--  vs_grasping_challenge.lua
--
--  Created Sun April 10
--  Copyright  2022  Matteo Tschesche
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
name               = "vs_grasping_challenge"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"goto", "manipulate_wp"}
depends_interfaces = {}

documentation      = [==[
This skill is used to evaluate the visual servoing approach against the point
cloud based method. It requieres a MPS as input and starts picking the
workpiece from its input. Afterwards it puts the workpiece on the output. This
precedure is repeated 2 times.

Parameters:
      @param mps  the name of the MPS (e.g. C-CS1, see navgraph)
]==]

local mps_end_pos = {["C-RS1"] = {x = -0.5, y = 4.5, ori = 3.14},
                     ["C-CS1"] = {x = -4.5, y = 1.5, ori = 1.57},
                     ["C-BS"] = {x = -0.5, y = 0.5, ori = 1.57}}

-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

function finished()
  return fsm.vars.nr_puts == 3
end

fsm:define_states{ export_to=_M, closure={},
   {"INIT",    JumpState},
   {"PICK",    SkillJumpState, skills={{manipulate_wp}}, final_to="PUT" ,fail_to="FAILED"},
   {"PUT",     SkillJumpState, skills={{manipulate_wp}}, final_to="REPEAT" ,fail_to="FAILED"},
   {"REPEAT",  JumpState},
   {"END_POS", SkillJumpState, skills={{goto}}, final_to="FINAL", fail_to="FINAL"},
}

fsm:add_transitions{
   {"INIT", "PICK",      true},
   {"REPEAT", "END_POS", cond=finished, desc="Go to start position"},
   {"REPEAT", "PICK",    true, desc="Repeat"},
}

function INIT:init()
  fsm.vars.time_start = fawkes.Time:new():in_msec()
  print_info("[VS] Evaluating grasping challenge for mps " .. fsm.vars.mps)
  fsm.vars.nr_puts = 0
end

function PICK:init()
  self.args["manipulate_wp"] = {target = "WORKPIECE",
                                mps = fsm.vars.mps,
                                side = "OUTPUT"}
end

function PUT:init()
  fsm.vars.nr_puts = fsm.vars.nr_puts + 1
  self.args["manipulate_wp"] = {target = "CONVEYOR",
                                mps = fsm.vars.mps,
                                side = "INPUT"}
end

function END_POS:init()
  self.args["goto"] = {x = mps_end_pos[fsm.vars.mps].x,
                       y = mps_end_pos[fsm.vars.mps].y,
                       ori = mps_end_pos[fsm.vars.mps].ori}
end

function END_POS:exit()
  local now = fawkes.Time:new():in_msec()
  print_info("[VS] Execution time for mps " .. fsm.vars.mps .. ": " .. now - fsm.vars.time_start)
end
