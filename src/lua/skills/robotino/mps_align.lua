
----------------------------------------------------------------------------
--  align_mps.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
--                   Johannes Rothe
--                   Nicolas Limpert
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
name               = "mps_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = { "align_tag", "motor_move", "check_tag" }
depends_interfaces = {
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"},
   {v = "line5", type="LaserLineInterface", id="/laser-lines/5"},
   {v = "line6", type="LaserLineInterface", id="/laser-lines/6"},
   {v = "line7", type="LaserLineInterface", id="/laser-lines/7"},
   {v = "line8", type="LaserLineInterface", id="/laser-lines/8"},
   {v = "tag_0", type = "Position3DInterface", id="/tag-vision/0"},
   {v = "tag_1", type = "Position3DInterface", id="/tag-vision/1"},
   {v = "tag_2", type = "Position3DInterface", id="/tag-vision/2"},
   {v = "tag_3", type = "Position3DInterface", id="/tag-vision/3"},
   {v = "tag_4", type = "Position3DInterface", id="/tag-vision/4"},
   {v = "tag_5", type = "Position3DInterface", id="/tag-vision/5"},
   {v = "tag_6", type = "Position3DInterface", id="/tag-vision/6"},
   {v = "tag_7", type = "Position3DInterface", id="/tag-vision/7"},
   {v = "tag_8", type = "Position3DInterface", id="/tag-vision/8"},
   {v = "tag_9", type = "Position3DInterface", id="/tag-vision/9"},
   {v = "tag_10", type = "Position3DInterface", id="/tag-vision/10"},
   {v = "tag_11", type = "Position3DInterface", id="/tag-vision/11"},
   {v = "tag_12", type = "Position3DInterface", id="/tag-vision/12"},
   {v = "tag_13", type = "Position3DInterface", id="/tag-vision/13"},
   {v = "tag_14", type = "Position3DInterface", id="/tag-vision/14"},
   {v = "tag_15", type = "Position3DInterface", id="/tag-vision/15"},
   {v = "tag_info", type = "TagVisionInterface", id="/tag-vision/info"},
}

documentation      = [==[ align_mps

                          This skill does:
                          - aligns to the machine via sensor information AND a optional offsets which are given as parameters 

                          @param tag_id     int     optional the tag_id for align_tag
                          @param place      string  
                          @param x          float   the x offset after the alignmend
                          @param y          float   optional the y offset after the alignmend; TODO this just works within the AREA_LINE_ERR_Y
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("tf_module")
local llutils = require("fawkes.laser-lines_utils")
local tag_utils = require("tag_utils")

-- Tunables
local MIN_VIS_HIST=10
local MIN_VIS_HIST_TAG=10

local LINE_LENGTH_MIN=0.64
local LINE_LENGTH_MAX=0.71
local LINE_XDIST_MAX=0.6
local LINE_BEARING_MIN=0.1
local SEARCH_MOVES={ {ori=0.5}, {ori=-1} }

local MAX_TRANS_ERR=0.05
local MAX_ORI_ERR=0.1


function get_lines()
   for i,line in ipairs(fsm.vars.lines) do
      if line:visibility_history() >= MIN_VIS_HIST and
         line:length() >= LINE_LENGTH_MIN and
         line:length() <= LINE_LENGTH_MAX and
         line:bearing() > LINE_BEARING_MIN and
         math.min(line:end_point_1(0), line:end_point_2(0)) <= LINE_XDIST_MAX
      then
         table.insert(fsm.vars.interesting_lines, {iface=line, idx=i})
      end
   end
end


function tag_visible(vis_hist)
   local tag_iface = tag_utils.iface_for_id(fsm.vars.tags, tag_info, fsm.vars.tag_id)
   print(tag_iface:visibility_history())
   return tag_iface and tag_iface:visibility_history() > vis_hist
end


function pose_error()
   local node = navgraph:node(fsm.vars.place)
   local node_bl = tfm.transform(
      { x=node:x(), y=node:y(), ori=node:get_property_as_float("orientation") },
      "/map", "/base_link"
   )
   return node_bl.x > MAX_TRANS_ERR
      or node_bl.y > MAX_TRANS_ERR
      or node_bl.ori > MAX_ORI_ERR
end


function place_valid()
   if fsm.vars.place then
      if (string.sub(fsm.vars.place, -1) == "I"
         or string.sub(fsm.vars.place, -1) == "O") then
         return navgraph:node(fsm.vars.place):is_valid()
      else
         print("ERROR: place arg must be a machine INPUT or OUTPUT node!")
         return false
      end
   end
   return true
end


function want_search()
   if fsm.vars.place then
      return fsm.vars.globalsearch_done and fsm.vars.search_idx <= #SEARCH_MOVES
   else
      return fsm.vars.search_idx <= #SEARCH_MOVES
   end
end


function best_line()
   local min_bearing = 1000
   local best_line = nil
   for i,line in ipairs(fsm.vars.lines) do
      if line:bearing() < min_bearing then
         best_line = line
      end
   end
   fsm.vars.best_line = best_line
   return best_line
end


fsm:define_states{ export_to=_M, closure={SEARCH_MOVES=SEARCH_MOVES, place_valid=place_valid,
      pose_error=pose_error, tag_visible=tag_visible, MIN_VIS_HIST_TAG=MIN_VIS_HIST_TAG,
      want_search=want_search, line_visible=line_visible },
   {"INIT",                   JumpState},
   {"CHECK_TAG",              JumpState},
   {"SEARCH",                 JumpState},
   {"SEARCH_LINES",           SkillJumpState, skills={{"motor_move"}}, final_to="CHECK_TAG", fail_to="FAILED"},
   {"SEARCH_GLOBAL",          SkillJumpState, skills={{"global_motor_move"}}, final_to="CHECK_TAG", fail_to="FAILED"},
   {"FIND_TAG",               SkillJumpState, skills={{"motor_move"}}, final_to="CHECK_TAG", fail_to="FAILED"},
   {"SETTLE_LINE",            JumpState},
   {"ALIGN",                  SkillJumpState, skills={{"motor_move"}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT",          "FAILED",          precond="not place_valid()", desc="place arg invalid"},
   {"INIT",          "FAILED",          precond="not vars.x", desc="x argument missing"},
   {"INIT",          "CHECK_TAG",       cond=true},

   {"CHECK_TAG",     "FIND_TAG",        timeout=1, desc="no tag"},
   {"CHECK_TAG",     "SETTLE_LINE",     cond="tag_visible(MIN_VIS_HIST_TAG)"},

   {"FIND_TAG",      "SEARCH_GLOBAL",   cond="want_search() and vars.place and pose_error"},
   {"FIND_TAG",      "SEARCH_LINES",    cond="want_search() and #vars.interesting_lines > 0"},
   {"FIND_TAG",      "FAILED",          cond="not want_search()", timeout=1},

   {"SEARCH_GLOBAL", "CHECK_TAG",       cond="tag_visible(MIN_VIS_HIST_TAG)"},
   {"SEARCH_LINES",  "CHECK_TAG",       cond="tag_visible(MIN_VIS_HIST_TAG)"},

   {"SETTLE_LINE",   "ALIGN",           cond=best_line},
}

function INIT:init()
   self.fsm.vars.y   = self.fsm.vars.y   or 0
   self.fsm.vars.ori = self.fsm.vars.ori or 0

   self.fsm.vars.lines = {}
   self.fsm.vars.lines[1]  = line1
   self.fsm.vars.lines[2]  = line2
   self.fsm.vars.lines[3]  = line3
   self.fsm.vars.lines[4]  = line4
   self.fsm.vars.lines[5]  = line5
   self.fsm.vars.lines[6]  = line6
   self.fsm.vars.lines[7]  = line7
   self.fsm.vars.lines[8]  = line8

   self.fsm.vars.interesting_lines = {}

   self.fsm.vars.tags = {}
   self.fsm.vars.tags[1] = tag_0
   self.fsm.vars.tags[2] = tag_1
   self.fsm.vars.tags[3] = tag_2
   self.fsm.vars.tags[4] = tag_3
   self.fsm.vars.tags[5] = tag_4
   self.fsm.vars.tags[6] = tag_5
   self.fsm.vars.tags[7] = tag_6
   self.fsm.vars.tags[8] = tag_7
   self.fsm.vars.tags[9] = tag_8
   self.fsm.vars.tags[10] = tag_9
   self.fsm.vars.tags[11] = tag_10
   self.fsm.vars.tags[12] = tag_11
   self.fsm.vars.tags[13] = tag_12
   self.fsm.vars.tags[14] = tag_13
   self.fsm.vars.tags[15] = tag_14
   self.fsm.vars.tags[16] = tag_15

   self.fsm.vars.search_idx = 0
   self.fsm.vars.globalsearch_done = false
end

function FIND_TAG:loop()
   get_lines()
end

function FIND_TAG:exit()
   self.fsm.vars.search_idx = self.fsm.vars.search_idx + 1
end

function SEARCH_GLOBAL:init()
   self.args["global_motor_move"] = self.fsm.vars.place
end

function SEARCH_GLOBAL:exit()
   self.fsm.vars.globalsearch_done = true
end

function SEARCH_LINES:init()
   local min_dist = 1000
   local target = nil
   for i,line in ipairs(self.fsm.vars.interesting_lines) do
      local center = llutils.center(line)
      local dist = math.vec_length(center.x, center.y)
      if dist * line:bearing() < min_dist then
         target = llutils.point_in_front(llutis.center(line), self.fsm.vars.x)
         min_dist = dist * line:bearing()
      end
   end
   
   if target then
      self.args["motor_move"] = { x=target.x, y=target.y, ori=target.ori }
   else
      self.args["motor_move"] = SEARCH_MOVES[self.fsm.vars.search_idx]
   end
end

function ALIGN:init()
   self.args["motor_move"] = llutils.point_in_front(llutils.center(
      self.fsm.vars.best_line), self.fsm.vars.x)
end

