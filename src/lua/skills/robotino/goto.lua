
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
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = { "relgoto", "global_motor_move" }
depends_interfaces = {
   {v = "pose", type="Position3DInterface", id="Pose"},
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
local REGION_TRANS=0.2

function check_navgraph(self)
  return self.fsm.vars.place ~= nil and not navgraph
end

function reached_target_region(self)
  local region_sqr = self.fsm.vars.region_trans * self.fsm.vars.region_trans
  local rel_pos = tf_mod.transform({
                      x = self.fsm.vars.x,
                      y = self.fsm.vars.y,
                      ori = self.fsm.vars.ori or 0}, 
                      "/map", "/base_link")
  local distance_region_sqr = rel_pos.x * rel_pos.x + rel_pos.y * rel_pos.y

  if distance_region_sqr > region_sqr then
    return false
  else
    return true
  end
end

fsm:define_states{ export_to=_M,
  closure={check_navgraph=check_navgraph, reached_target_region=reached_target_region, },
  {"INIT",          JumpState},
  {"SKILL_RELGOTO", SkillJumpState, skills={{relgoto}}, final_to="FINAL", fail_to="FAILED"},
  {"REGION_REACHED_STOPPING", JumpState},
  {"FINAL_ORIENTATION",       SkillJumpState, skills={{global_motor_move}}, final_to="FINAL", fail_to="FINAL"}, -- will be ok; trust me :=)
}

fsm:add_transitions{
  {"INIT",  "FAILED",         precond=check_navgraph, desc="no navgraph"},
  {"INIT",  "FAILED",         cond="not vars.target_valid",                 desc="target invalid"},
  {"INIT",  "SKILL_RELGOTO",  cond=true},
  {"SKILL_RELGOTO", "INIT", timeout=1, desc="Recalculate target"},
  {"SKILL_RELGOTO", "REGION_REACHED_STOPPING", cond=reached_target_region, desc="Reached target, stopping local-planner"},
  {"REGION_REACHED_STOPPING", "FINAL_ORIENTATION", cond="vars.ori", desc="Stopped, do final orientation"},
  {"REGION_REACHED_STOPPING", "FINAL",             cond="not vars.ori", desc="Stopped, and don't need to orientate => FINAL"},
}

function INIT:init()
  self.fsm.vars.target_valid = true

  -- if a place is given, get the point from the navgraph and use this instead of x, y, ori
  if self.fsm.vars.place ~= nil then
    self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
    if self.fsm.vars.node:is_valid() then
      self.fsm.vars.x = self.fsm.vars.node:x()
      self.fsm.vars.y = self.fsm.vars.node:y()
      if self.fsm.vars.node:has_property("orientation") then
        self.fsm.vars.ori   = self.fsm.vars.node:property_as_float("orientation");
      else
        self.fsm.vars.ori   = nil   -- if orientation is not set, we don't care
      end
    else
      self.fsm.vars.target_valid = false
    end
  end 

  if self.fsm.vars.target_valid then
    local rel_pos = tf_mod.transform({
                      x = self.fsm.vars.x,
                      y = self.fsm.vars.y,
                      ori = self.fsm.vars.ori or 0},
                      "/map", "/base_link")

    -- sanity check *this is an error*, but where is it comming from???
    if rel_pos.x <= 20 or rel_pos.y <= 20 then
      self.fsm.vars.rel_x   = rel_pos.x
      self.fsm.vars.rel_y   = rel_pos.y
    else
      self.fsm.vars.rel_x   = 0
      self.fsm.vars.rel_y   = 0
      print_error("GOTO ERROR!!!!!!!!!! place: " .. self.fsm.vars.place ..
                                        " f_x: " .. self.fsm.vars.x ..
                                        " f_y: " .. self.fsm.vars.y ..
                                        " f_ori: " .. self.fsm.vars.ori ..
                                        " t_x: " .. rel_pos.x ..
                                        " t_y: " .. rel_pos.y ..
                                        " t_ori: " .. rel_pos.ori)
    end

    if self.fsm.vars.ori == nil then
      self.fsm.vars.rel_ori = nil
    else
      self.fsm.vars.rel_ori = rel_pos.ori
    end
  end

  self.fsm.vars.region_trans = self.fsm.vars.region_trans or REGION_TRANS
end

function SKILL_RELGOTO:init()
	 self.args["relgoto"] = { x = self.fsm.vars.rel_x, y = self.fsm.vars.rel_y, ori = self.fsm.vars.rel_ori }
end

function REGION_REACHED_STOPPING:init()
  local msg = navigator.StopMessage:new( )
  navigator:msgq_enqueue(msg)
end

function FINAL_ORIENTATION:init()
  self.args["global_motor_move"] = {ori=self.fsm.vars.ori}
end
