----------------------------------------------------------------------------
--  grasping.lua
--
--  Created Wed Apr 15
--  Copyright  2015  Sebastian Eltester
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
name               = "grasping"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"get_product_from", "bring_product_to", "goto"}
depends_interfaces = {
}

documentation      = [==[
Skill to do a grasping challenge

Parameters:
	@param place MPS to use
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M, closure={},
   {"INIT", JumpState},
   {"GET", SkillJumpState, skills={{get_product_from}},final_to="GO_TO_BRING", fail_to="GET"},
   {"GO_TO_BRING", SkillJumpState, skills={{goto}},final_to="BRING", fail_to="GO_TO"},
   {"BRING", SkillJumpState, skills={{bring_product_to}}, final_to="GO_TO_GET",fail_to="BRING"},
   {"GO_TO_GET", SkillJumpState, skills={{goto}},final_to="GET", fail_to="GO_TO"},
}

fsm:add_transitions{
   {"INIT", "GET", cond=true},
   {"BRING", "FINAL", cond="vars.enough"},
}


function INIT:init()
  -- Override values if host specific config value is set
  self.fsm.vars.enough = false
  self.fsm.vars.iterator_count = 0
end

function GET:init()
   if self.fsm.vars.error then
      self.fsm.vars.iterator_count = 0
   else
      self.fsm.vars.iterator_count = self.fsm.vars.iterator_count + 1
   end
  self.args["get_product_from"].place = self.fsm.vars.place
  self.args["get_product_from"].side = "output"
end

function BRING:init()
  self.args["bring_product_to"].place= self.fsm.vars.place
  self.args["bring_product_to"].side= "input"
   if self.fsm.vars.error then
      self.fsm.vars.iterator_count = 0
   else
      self.fsm.vars.iterator_count = self.fsm.vars.iterator_count + 1
   end
   if self.fsm.vars.iterator_count > 6 then
      self.fsm.vars.enough = true
   end
end

function GO_TO_BRING:init()
   self.args["goto"].place = self.fsm.vars.place .. "-I"
end

function GO_TO_GET:init()
   self.args["goto"].place = self.fsm.vars.place .. "-O"
end
