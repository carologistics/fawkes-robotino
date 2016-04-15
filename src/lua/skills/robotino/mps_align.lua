
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

local tfm = require("fawkes.tfutils")
local llutils = require("fawkes.laser-lines_utils")
local tag_utils = require("tag_utils")

-- Tunables
local MIN_VIS_HIST_LINE=10
local MIN_VIS_HIST_TAG=10

local LINE_LENGTH_MIN=0.64
local LINE_LENGTH_MAX=0.71
local LINE_XDIST_MAX=0.6

local MAX_TRANS_ERR=0.05
local MAX_ORI_ERR=0.1
local LINE_MATCH_TOLERANCE=0.3

local TURN_MOVES={
   { ori = math.pi/2},
   { ori = -math.pi},
   { ori = -math.pi/2}
}
local MAX_TRIES = 6


function tag_visible(vis_hist)
   local tag_iface = tag_utils.iface_for_id(fsm.vars.tags, tag_info, fsm.vars.tag_id)
   if tag_iface and tag_iface:visibility_history() > vis_hist then
      print("Tag visible: " .. tag_iface:id())
      return true
   end
   return false
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
   return not fsm.vars.place
      or (navgraph:node(fsm.vars.place):is_valid()
      and (string.sub(fsm.vars.place, -1) == "I"
         or string.sub(fsm.vars.place, -1) == "O"))
end


function want_search()
   if fsm.vars.place then
      return fsm.vars.globalsearch_done and fsm.vars.search_idx <= MAX_TRIES
   else
      return fsm.vars.search_idx <= MAX_TRIES
   end
end


fsm:define_states{ export_to=_M, closure={place_valid=place_valid,
      pose_error=pose_error, tag_visible=tag_visible, MIN_VIS_HIST_TAG=MIN_VIS_HIST_TAG,
      want_search=want_search, line_visible=line_visible },
   {"INIT",                   JumpState},
   {"CHECK_TAG",              JumpState},
   {"FIND_TAG",               JumpState},
   {"SEARCH_LINES",           SkillJumpState, skills={{"motor_move"}}, final_to="CHECK_TAG", fail_to="FAILED"},
   {"SEARCH_GLOBAL",          SkillJumpState, skills={{"global_motor_move"}}, final_to="CHECK_TAG", fail_to="FAILED"},
   {"TURN_AROUND",            SkillJumpState, skills={{"motor_move"}}, final_to="CHECK_TAG", fail_to="FAILED"},
   {"SETTLE_LINE",            JumpState},
   {"ALIGN",                  SkillJumpState, skills={{"motor_move"}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT",          "FAILED",          precond="not place_valid()", desc="place arg invalid"},
   {"INIT",          "FAILED",          precond="not vars.x", desc="x argument missing"},
   {"INIT",          "CHECK_TAG",       cond=true},

   {"CHECK_TAG",     "FIND_TAG",        timeout=1, desc="no tag"},
   {"CHECK_TAG",     "SETTLE_LINE",     cond="tag_visible(MIN_VIS_HIST_TAG)", desc="found tag"},

   {"FIND_TAG",      "SEARCH_GLOBAL",   cond="want_search() and vars.place and pose_error()", desc="correct pose"},
   {"FIND_TAG",      "SEARCH_LINES",    cond="want_search() and #vars.interesting_lines > 0", desc="search 4 tag"},
   {"FIND_TAG",      "SETTLE_LINE",     cond="tag_visible(MIN_VIS_HIST_TAG)", desc="found tag"},
   {"FIND_TAG",      "TURN_AROUND",     timeout=1, desc="no interesting lines"},
   {"FIND_TAG",      "FAILED",          cond="not want_search()"},

   {"SEARCH_GLOBAL", "SETTLE_LINE",     cond="tag_visible(MIN_VIS_HIST_TAG)", desc="found tag"},
   {"SEARCH_LINES",  "SETTLE_LINE",     cond="tag_visible(MIN_VIS_HIST_TAG)", desc="found tag"},

   {"SETTLE_LINE",   "ALIGN",           cond="vars.matched_line"},
   {"SETTLE_LINE",   "CHECK_TAG",       timeout=2, desc="lost line"}
}

function INIT:init()
   self.fsm.vars.y   = self.fsm.vars.y   or 0
   self.fsm.vars.ori = self.fsm.vars.ori or 0

   self.fsm.vars.lines = {}
   self.fsm.vars.lines[line1:id()]  = line1
   self.fsm.vars.lines[line2:id()]  = line2
   self.fsm.vars.lines[line3:id()]  = line3
   self.fsm.vars.lines[line4:id()]  = line4
   self.fsm.vars.lines[line5:id()]  = line5
   self.fsm.vars.lines[line6:id()]  = line6
   self.fsm.vars.lines[line7:id()]  = line7
   self.fsm.vars.lines[line8:id()]  = line8

   -- Tracker to remember how often we looked for a tag at each line
   -- Required so we don't alternate between two lines that both don't have the desired tag.
   self.fsm.vars.lines_visited = {}
   for k,v in pairs(self.fsm.vars.lines) do
      self.fsm.vars.lines_visited[k] = 0
   end

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

-- Match tag to laser line
function match_line(tag, lines)
   local matched_line = nil
   if tag and tag:visibility_history() >= MIN_VIS_HIST_TAG then
      local tag_laser = tfm.transform6D(
         { x=tag:translation(0), y=tag:translation(1), z=tag:translation(2),
            ori = { x=tag:rotation(0), y=tag:rotation(1), z=tag:rotation(2), w=tag:rotation(3)  }
         }, tag:frame(), "/base_laser"
      )
      local min_dist = 1000
      for k,line in pairs(lines) do
         local line_center = llutils.center(line)
         local dist = math.vec_length(tag_laser.x - line_center.x, tag_laser.y - line_center.y)
         if line:visibility_history() >= MIN_VIS_HIST_LINE
            and dist < LINE_MATCH_TOLERANCE
            and dist < min_dist
         then
            min_dist = dist
            matched_line = line
            printf("tag: %f,%f; center: %f,%f", tag_laser.x, tag_laser.y, line_center.x, line_center.y)
            printf("matched %s tag dist: %f", line:id(), dist)
         end
      end
   end

   return matched_line
end

-- Return all lines which may have the tag we're looking for
function get_interesting_lines(lines)
   local rv = {}
   
   -- Shallow-copy input table so we don't delete values from it
   local good_lines = {}
   for k,v in pairs(lines) do good_lines[k] = v end

   -- Match unwanted tags to lines and remove them
   local bad_tags = tag_utils.bad_tags(fsm.vars.tags, tag_info, fsm.vars.tag_id)
   for j,tag in ipairs(bad_tags) do
      if tag:visibility_history() > 0 then
         local matched = match_line(tag, good_lines)
         if matched then good_lines[matched:id()] = nil end
      end
   end
   
   -- Use only lines that have been inspected 0 times or less often than the others
   local max_num_visited = 1
   for k,v in pairs(fsm.vars.lines_visited) do
      if v > max_num_visited then
         max_num_visited = v
      end
   end

   for k,line in pairs(good_lines) do
      if line then
         local center = llutils.center(line)
         local ori = math.atan2(center.y, center.x)
         if line:visibility_history() >= MIN_VIS_HIST_LINE and
            line:length() >= LINE_LENGTH_MIN and
            line:length() <= LINE_LENGTH_MAX and
            fsm.vars.lines_visited[line:id()] < max_num_visited and
            math.min(line:end_point_1(0), line:end_point_2(0)) <= LINE_XDIST_MAX
         then
            table.insert(rv, line)
            printf("interesting %s ori: %f", line:id(), ori)
         end
      end
   end
   return rv
end


function CHECK_TAG:loop()
   local tag = tag_utils.iface_for_id(fsm.vars.tags, tag_info, fsm.vars.tag_id)
   if tag and tag:visibility_history() > MIN_VIS_HIST_TAG then
      self.fsm.vars.matched_line = match_line(tag, self.fsm.vars.lines)
   end
end
function CHECK_TAG:init()
end

function FIND_TAG:init()
   self.fsm.vars.interesting_lines = get_interesting_lines(self.fsm.vars.lines)
end

function FIND_TAG:loop()
   self.fsm.vars.interesting_lines = get_interesting_lines(self.fsm.vars.lines)
end


function SEARCH_GLOBAL:init()
   self.args["global_motor_move"] = self.fsm.vars.place
end
function SEARCH_GLOBAL:exit()
   self.fsm.vars.globalsearch_done = true
end


function SEARCH_LINES:init()
   local min_dist = 1000
   local ori = nil
   local chosen_line = nil

   for i,l in ipairs(self.fsm.vars.interesting_lines) do
      local center = tfm.transform(llutils.center(l), "/base_laser", "/base_link")
      ori = math.atan2(center.y, center.x)

      if math.abs(ori) < min_dist then
         min_dist = math.abs(ori)
         chosen_line = l
      end
   end

   self.fsm.vars.lines_visited[chosen_line:id()] = self.fsm.vars.lines_visited[chosen_line:id()] + 1
   
   print("SEARCH LINES turn ori: " .. ori)
   self.args["motor_move"].ori = ori
   self.fsm.vars.search_idx = self.fsm.vars.search_idx + 1
end


function TURN_AROUND:init()
   print("TURN_AROUND search_idx: " .. self.fsm.vars.search_idx)
   self.args["motor_move"] = TURN_MOVES[math.mod(self.fsm.vars.search_idx, #TURN_MOVES)+1]
   self.fsm.vars.search_idx = self.fsm.vars.search_idx + 1

   for k,v in pairs(self.fsm.vars.lines_visited) do
      self.fsm.vars.lines_visited[k] = 0
   end
end


function SETTLE_LINE:loop()
   local tag = tag_utils.iface_for_id(fsm.vars.tags, tag_info, fsm.vars.tag_id)
   if tag then
      print("Matched tag: " .. tag:id())
   end

   self.fsm.vars.matched_line = match_line(tag, self.fsm.vars.lines)
   if self.fsm.vars.matched_line then
      print("Matched line: " .. self.fsm.vars.matched_line:id())
   end
end


function ALIGN:init()
   local center = llutils.center(self.fsm.vars.matched_line)
   local center_bl = tfm.transform(center, "/base_laser", "/base_link")
   local p = llutils.point_in_front(center_bl, self.fsm.vars.x)
   self.args["motor_move"] = p
end

