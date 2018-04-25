
----------------------------------------------------------------------------
--  goto.lua -
--
--  Created: Thu Aug 14 14:32:47 2008
--  modified by Victor Mataré, David Schmidt
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
fsm                = SkillHSM:new{name=name, start="CHECK_NAVIGATOR", debug=true}
depends_skills     = { }
depends_interfaces = {
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

local tf_mod = require 'fawkes.tfutils'

function target_reached()
   if navigator:is_final() and navigator:error_code() ~= 0 then
      return false
   end
   return navigator:is_final()
end

function has_navigator()
   return navigator:has_writer()
end

function has_target_coordinates(self)
   return self.fsm.vars.x ~= nil and self.fsm.vars.y ~= nil and self.fsm.vars.ori ~= nil
end

function has_place(self)
   return self.fsm.vars.place ~= nil
end

function target_unreachable()
   if navigator:is_final() and navigator:error_code() ~= 0 then
      return true
   end
   return false
end

fsm:define_states{ export_to=_M,
  closure={has_navigator=has_navigator},
  {"CHECK_NAVIGATOR",   JumpState},
  {"COMPUTE_COORDINATES_BY_PLACE",       JumpState},
  {"INIT",          JumpState},
  {"MOVING",        JumpState},
  {"TIMEOUT",       JumpState},
}

fsm:add_transitions{
  {"CHECK_NAVIGATOR", "FAILED", cond="not has_navigator()", desc="Navigator not running"},
  {"CHECK_NAVIGATOR", "MOVING", cond=has_target_coordinates},
  {"CHECK_NAVIGATOR", "COMPUTE_COORDINATES_BY_PLACE", cond=has_place},
  {"CHECK_NAVIGATOR", "FAILED", cond=true, desc="missing target information"},
  {"COMPUTE_COORDINATES_BY_PLACE", "MOVING", cond=has_target_coordinates, desc="finished computing coordinates"},
  {"COMPUTE_COORDINATES_BY_PLACE", "FAILED", timeout=10, desc="computation of coordinates by place failed"},
  {"MOVING", "TIMEOUT",       timeout=2}, -- Give the interface some time to update
  {"TIMEOUT", "FINAL",         cond=target_reached, desc="Target reached"},
  {"TIMEOUT", "FAILED",        cond=target_unreachable, desc="Target unreachable"},
}

function CHECK_NAVIGATOR:init()
   if self.fsm.vars.place == nil and (self.fsm.vars.x == nil or self.fsm.vars.y == nil or self.fsm.vars.ori == nil) then
      print_error("Skill goto is missing arguments, either place or x, y, ori")
   end
end

function COMPUTE_COORDINATES_BY_PLACE:loop()
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
      if not node:is_valid() then
        print_warn("Navgraph node is invalid. Navgraph computation probably still running!")
        return
      end
      local cur_pose = tf_mod.transform({x=0, y=0, ori=0}, "base_link", "map")
      if cur_pose == nil then
        print_warn("Failed to load transform from 'base_link' to 'map'!")
        return
      end
      self.fsm.vars.x = node:x()
      self.fsm.vars.y = node:y()
      if node:has_property("orientation") then
        self.fsm.vars.ori = node:property_as_float("orientation");
      else
        self.fsm.vars.ori = cur_pose.ori
      end
    end
  else
    print_error("Missing argument 'place'!")
  end
end

function MOVING:init()
   self.fsm.vars.msgid_timeout = os.time() + 1

   local msg = navigator.CartesianGotoWithFrameMessage:new(
      self.fsm.vars.x,
      self.fsm.vars.y,
      self.fsm.vars.ori,
      "map")
   fsm.vars.goto_msgid = navigator:msgq_enqueue(msg)
end
