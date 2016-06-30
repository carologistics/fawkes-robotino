
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = { "goto","global_motor_move" }
depends_interfaces = { }

documentation      = [==[Drive to point with colisoin avoidance and last part with global_motor_move

Parameters:
      place:        Where to drive to
      x:            x coordinate to drive to (if place is not set)
      y:            y coordinate to drive to (if place is not set)
      ori:          alternativ orientation
      just_ori:     if true => after goto finished, global_motor_move is just called with ori (for exploration)
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function node_is_valid(self)
  if self.fsm.vars.point_set then
    return self.fsm.vars.point_valid
  end
  return true
end

fsm:define_states{ export_to=_M,
  closure={navgraph=navgraph, node_is_valid=node_is_valid},
  {"INIT",                     JumpState},
  {"SKILL_PPGOTO",             SkillJumpState, skills={{goto}},            final_to="TIMEOUT", fail_to="FORCE_SET_JUST_ORI"},
  {"FORCE_SET_JUST_ORI",       JumpState},
  {"TIMEOUT",                  JumpState},
  {"SKILL_GLOBAL_MOTOR_MOVE",  SkillJumpState, skills={{global_motor_move}}, final_to="FINAL",   fail_to="FINAL"},
}

fsm:add_transitions{
  { "INIT",    "FAILED",                   cond="not navgraph", desc="navgraph not available" },
  { "INIT",    "FAILED",                   cond="not node_is_valid(self)",  desc="point invalid" },
  { "INIT",    "SKILL_PPGOTO",             cond=true },
  { "FORCE_SET_JUST_ORI", "TIMEOUT",       cond=true },
  { "TIMEOUT", "SKILL_GLOBAL_MOTOR_MOVE",  timeout=0.5 },
}

function INIT:init()
  self.fsm.vars.point_set   = false
  self.fsm.vars.point_valid = false
  if self.fsm.vars.place ~= nil then
    self.fsm.vars.point_set = true
    self.fsm.vars.node = navgraph:node(self.fsm.vars.place)

    if self.fsm.vars.node:is_valid() then
      self.fsm.vars.point_valid = true
      self.fsm.vars.x = self.fsm.vars.node:x()
      self.fsm.vars.y = self.fsm.vars.node:y()
      if self.fsm.vars.ori == nil and self.fsm.vars.node:has_property("orientation") then
        self.fsm.vars.ori = self.fsm.vars.node:property_as_float("orientation");
      end
    end
  end
end

function SKILL_PPGOTO:init()
	 self.args["goto"] = {x = self.fsm.vars.x, y = self.fsm.vars.y, ori = self.fsm.vars.ori}
end

function FORCE_SET_JUST_ORI:init()
  self.fsm.vars.just_ori = true
  printf("Drive to global: force just ori")
end


function SKILL_GLOBAL_MOTOR_MOVE:init()
  if not self.fsm.vars.just_ori then
		 self.args["global_motor_move"] = {x = self.fsm.vars.x, y = self.fsm.vars.y}
  end
  self.args["global_motor_move"].ori = self.fsm.vars.ori
  
  printf("Drive to: call global_motor_move with: x(" ..
						tostring(self.args["global_motor_move"].x) .. ") y(" ..
						tostring(self.args["global_motor_move"].y)  ..") ori(" ..
						tostring(self.args["global_motor_move"].ori) .. ")")
end
