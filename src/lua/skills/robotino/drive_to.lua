
----------------------------------------------------------------------------
--  drive_to.lua
--
--  Created: Sat Jul 12 13:25:47 2014
--  Copyright  2008       Tim Niemueller [www.niemueller.de]
--             2014-2015  Tobias Neumann
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
name               = "drive_to"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "ppgoto","global_motor_move" }
depends_interfaces = { }

documentation      = [==[Drive to point with colisoin avoidance and last part with global_motor_move

Parameters:
      place:        Where to drive to
      ori:          alternativ orientation
      just_ori:     if true => after ppgoto finished, global_motor_move is just called with ori (for exploration)
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
  closure={navgraph=navgraph},
  {"INIT",                     JumpState},
  {"SKILL_PPGOTO",             SkillJumpState, skills={{ppgoto}},            final_to="TIMEOUT", fail_to="FAILED"},
  {"TIMEOUT",                  JumpState},
  {"SKILL_GLOBAL_MOTOR_MOVE",  SkillJumpState, skills={{global_motor_move}}, final_to="FINAL",   fail_to="FINAL"},
}

fsm:add_transitions{
  { "INIT",    "FAILED",                   cond="not navgraph",             desc="navgraph not available" },
  { "INIT",    "FAILED",                   cond="not vars.node:is_valid()", desc="point invalid" },
  { "INIT",    "SKILL_PPGOTO",             cond=true },
  { "TIMEOUT", "SKILL_GLOBAL_MOTOR_MOVE",  timeout=0.5 },
}

function INIT:init()
  self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function SKILL_PPGOTO:init()
   self.skills[1].place = self.fsm.vars.place
   self.skills[1].ori   = self.fsm.vars.ori
end

function SKILL_GLOBAL_MOTOR_MOVE:init()
  
  -- x and y are just set if we not just want to use ori
  if not self.fsm.vars.just_ori then
    self.skills[1].x = self.fsm.vars.node:x()
    self.skills[1].y = self.fsm.vars.node:y()
  end

  -- ori is set via a paramater, if non is given its set by the node
  if self.fsm.vars.ori == nil then
    if self.fsm.vars.node:has_property("orientation") then
      self.skills[1].ori  = self.fsm.vars.node:property_as_float("orientation");
    end
  else
    self.skills[1].ori    = self.fsm.vars.ori
  end

  printf("Drive to: call global_motor_move with: x(" .. tostring(self.skills[1].x) .. ") y(" .. tostring(self.skills[1].y)  ..") ori(" .. tostring(self.skills[1].ori) .. ")")
end
