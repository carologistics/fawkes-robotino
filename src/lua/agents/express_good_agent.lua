
----------------------------------------------------------------------------
--  express_good_agent.lua - Express Good Agent for RoboCup Logistics League
--
--  Created: Mon Jun 18 17:37:44 2012 (RoboCup 2012, Mexico City)
--  Copyright  2012  Jens Wittmeyer
--             2012  Tim Niemueller
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
module(..., agentenv.module_init)

-- Crucial agent information
name               = "express_good_agent"
fsm                = AgentHSM:new{name=name, debug=true, start="INIT"}
depends_skills     = {"goto", "move_under_rfid", "determine_signal",
                      "deliver_puck", "leave_area_with_puck", "fetch_puck",
                      "leave_area", "take_puck_to"}
depends_interfaces = {
   { v="worldmodel", type="RobotinoWorldModelInterface", id = "Model fll merged"},
   { v="omni_switch",type = "SwitchInterface" , id = "omnivisionSwitch"}, 
   { v="omni_puck_1",type = "Position3DInterface", id = "OmniPuck1"},
   { v="omni_puck_2",type = "Position3DInterface", id = "OmniPuck1"},
   { v="omni_puck_3",type = "Position3DInterface", id = "OmniPuck1"},
   { v="omni_puck_4",type = "Position3DInterface", id = "OmniPuck1"},
   { v="omni_puck_5",type = "Position3DInterface", id = "OmniPuck1"},
   { v="light",type ="RobotinoAmpelInterface", id = "light" } 
   { v="start",type ="SwitchInterface", id = "GameStart" } 
}

documentation      = [==[Express Agent]==]

-- Initialize as agent module
agentenv.agent_module(...)

local PUCK_CLOSE_DIST = 0.25

local tfm = require("tf_module")
local mpos = require("machine_pos_module")
local puck_loc = require("puck_loc_module")

local pucks = { omni_puck_1, omni_puck_2, omni_puck_3, omni_puck_4, omni_puck_5 }

function egi_puck()
   for i,p in ipairs(pucks) do

      local ploc = puck_loc.get_puck_loc(p)
      if ploc then
         -- transform position into map coordinates to check if it is close
         -- to the egc input area
         local map_pos = tfm.transform(ploc, p:frame(), "/map")

         local dx = map_pos.x - mpos.fields.EGI.x
         local dy = map_pos.y - mpos.fields.EGI.y
         if math.sqrt(dx*dx + dy*dy) <= PUCK_CLOSE_DIST then
            return ploc
         else
            printf("Puck at (%f,%f) is not close to (%f,%f)",
                   map_pos.x, map_pos.y, mpos.fields.EGI.x, mpos.fields.EGI.y)
         end
      end
   end

   return nil
end

function choose_next_m1(self)
   local m1_machines = {}
   for i = 1, 10 do
      if worldmodel:machine_types(i) == worldmodel.M1 then
         local m_name = m .. i;
         table.insert(m1_machines, m_name)
         if not self.fsm.vars.visited[m_name] then
            return m_name
         end
      end
   end

   -- non found, clear visited and try again
   self.fsm.vars.visited = {}
   if #m1_machines > 0 then
      return m1_machines[1]
   end

   -- really none, too bad
   return nil
end

function have_m1_exp()
   return worldmodel:get_express_machine() ~= 0
end

function no_change()
   return light:state() == light.NO_CHANGE
end
function green()
   return light:state() == light.GREEN
end


-- Setup FSM
fsm:add_transitions{
   closure={egi_puck=egi_puck, have_m1_exp=have_m1_exp, light=light, start=start},

   {"INIT", "MOVE_TO_EGC", cond="start:is_enabled()", desc="initialized/restarted"},
   {"MOVE_TO_EGC", "WAIT_PUCK", fail_to="MOVE_TO_EGC",
      skills={{"goto", {goto_name = "EGI"}}}},
   {"WAIT_PUCK", "FETCH_PUCK", cond="egi_puck() ~= nil and have_m1_exp()"},
   {"FETCH_PUCK" , "GOTO_M_EXP", fail_to="INIT", skills={{"fetch_puck"}} },
   {"GOTO_M_EXP","MOVE_UNDER_RFID", fail_to="INIT",
      skills={{"take_puck_to"}}},
   --[[
   {"CHECK_OUT_OF_ORDER_SKILL", "CHECK_OUT_OF_ORDER", fail_to="INIT",
      skills={{"determine_signal", {mode = "OUTOFORDER"}}} },
   {"CHECK_OUT_OF_ORDER", "MOVE_UNDER_RFID", cond="light:state() ~= light.RED"},
   {"CHECK_OUT_OF_ORDER", "GOTO_M1", cond="light:state() == light.RED"},
   {"GOTO_M1", "CHECK_OUT_OF_ORDER", skills={{"take_puck_to"}}},
   {"GOTO_M1", "CHECK_OUT_OF_ORDER", fail_to="INIT", skills={{"goto"}}},
   --]]
   {"MOVE_UNDER_RFID", "LEAVE_AREA_WITH_PUCK", fail_to="INIT",
      skills={{"move_under_rfid"}} },
   --{"CHECK_LIGHT", "DECIDE" , fail_to="INIT",
   --   skills={{"determine_signal", { mode = "EXP"}}} }, 
   --{"DECIDE", "LEAVE_AREA_WITH_PUCK", cond="light:state() == light.GREEN"},
   --{"DECIDE", "MOVE_TO_EGC", cond=true}, -- else case
   {"LEAVE_AREA_WITH_PUCK", "GOTO_DEL", fail_to="INIT",
      skills={{"leave_area_with_puck"}} },
   {"GOTO_DEL", "DELIVER", fail_to="INIT",
      skills={{"take_puck_to", {goto_name = "D1"}}} },
   {"DELIVER", "INIT", fail_to="INIT", skills = {{"deliver_puck"}} }
}

function WAIT_PUCK:init()
   if omnivisionSwitch:has_writer() then
      local m = omnivisionSwitch.EnableMessage:new()
      omnivisionSwitch:msgq_enqueue_copy(m)
   end
end

function WAIT_PUCK:exit()
   if omnivisionSwitch:has_writer() then
      local m = omnivisionSwitch.DisableMessage:new()
      omnivisionSwitch:msgq_enqueue_copy(m)
   end
end

function GOTO_M_EXP:init()
   if not self.fsm.vars.visited then
      self.fsm.vars.visited = {}
   end
   local m_name = "m" .. worldmodel:get_express_machine();

   self.fsm.vars.visited[m_name] = true
   self.skills[1].args = {goto_name = m_name}
end
