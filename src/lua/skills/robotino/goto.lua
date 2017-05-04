
----------------------------------------------------------------------------
--  goto.lua - 
--
--  Created: Thu Aug 14 14:32:47 2008
--  modified by Victor Mataré
--              2015  Tobias Neumann
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
name               = "goto"
fsm                = SkillHSM:new{name=name, start="CHECK_INPUT"}
depends_skills     = { }
depends_interfaces = {
--   {v = "pose", type="Position3DInterface", id="Pose"},
   {v = "navigator", type="NavigatorInterface", id="Navigator"},
}

documentation      = [==[Move to a known location via place or x, y, ori.
if place is set, this will be used and x, y and ori will be ignored

@param place  Name of the place we want to go to.
@param x      x we want to drive to
@param y      y we want to drive to
@param ori    ori we want to drive to

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tf_mod = require 'tf_module'

-- Tunables
--local REGION_TRANS=0.2

function check_navgraph(self)
  return self.fsm.vars.place ~= nil and not navgraph
end

function target_reached()
   if navigator:msgid() == fsm.vars.goto_msgid then
      if navigator:is_final() and navigator:error_code() ~= 0 then
         return false
      end
      return navigator:is_final()
   end
   return false
end

function can_navigate()
   return navigator:has_writer()
end

function target_unreachable()
   if navigator:msgid() == fsm.vars.goto_msgid then
      if navigator:is_final() and navigator:error_code() ~= 0 then
         return true
      end
   end
   return false
end

fsm:define_states{ export_to=_M,
  closure={check_navgraph=check_navgraph, reached_target_region=reached_target_region, },
  {"CHECK_INPUT",   JumpState},
  {"INIT",          JumpState},
  {"MOVING",        JumpState},
  {"TIMEOUT",       JumpState},
}

fsm:add_transitions{
  {"CHECK_INPUT", "INIT", cond=can_navigate },
  {"CHECK_INPUT", "FAILED", cond="not can_navigate()", desc="Navigator not running" },
  {"INIT",  "FAILED",         precond=check_navgraph, desc="no navgraph"},
  {"INIT",  "FAILED",         cond="not vars.target_valid",                 desc="target invalid"},
  {"INIT",  "MOVING",         cond=true},
  {"MOVING", "TIMEOUT",       timeout=2}, -- Give the interface some time to update
  {"TIMEOUT", "FINAL",         cond=target_reached, desc="Target reached"},
  {"TIMEOUT", "FAILED",        cond=target_unreachable, desc="Target unreachable"},
}

function CHECK_INPUT:init()
      self.fsm.vars.x   = self.fsm.vars.x   or self.fsm.vars.rel_x or 0
      self.fsm.vars.y   = self.fsm.vars.y   or self.fsm.vars.rel_y or 0
      self.fsm.vars.ori = self.fsm.vars.ori or self.fsm.vars.rel_ori or math.nan
end

function INIT:init()
  self.fsm.vars.target_valid = true

  if self.fsm.vars.place ~= nil then
    if string.match(self.fsm.vars.place, "[MC][-]Z[1-7][1-8]") then
      -- place argument is a zone, e.g. M-Z21
      self.fsm.vars.zone = self.fsm.vars.place
      self.fsm.vars.x = tonumber(string.sub(self.fsm.vars.place, 4, 4)) - 0.5
      self.fsm.vars.y = tonumber(string.sub(self.fsm.vars.place, 5, 5)) - 0.5
      if string.sub(self.fsm.vars.place, 1, 1) == "M" then
        self.fsm.vars.x = 0 - self.fsm.vars.x
      end
    else
      -- place argument is a navgraph point
      local node = navgraph:node(self.fsm.vars.place)
      if node:is_valid() then
        self.fsm.vars.x = node:x()
        self.fsm.vars.y = node:y()
        if node:has_property("orientation") then
          self.fsm.vars.ori = node:property_as_float("orientation");
        else
          self.fsm.vars.ori = nil   -- if orientation is not set, we don't care
        end
      else
        self.fsm.vars.target_valid = false
      end
    end
  end 

  local frame_msg = navigator.SetTargetFrameMessage:new( "/map" )
  navigator:msgq_enqueue(frame_msg)

  self.fsm.vars.region_trans = self.fsm.vars.region_trans or REGION_TRANS
end

function MOVING:init()
   self.fsm.vars.msgid_timeout = os.time() + 1

   local msg = navigator.CartesianGotoMessage:new(
      self.fsm.vars.x,
      self.fsm.vars.y,
      self.fsm.vars.ori)
   fsm.vars.goto_msgid = navigator:msgq_enqueue(msg)
end

function TIMEOUT:exit()
    local frame_msg = navigator.SetTargetFrameMessage:new( "/base_link" )
    navigator:msgq_enqueue(frame_msg)
end
