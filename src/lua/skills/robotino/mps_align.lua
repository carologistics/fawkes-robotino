
----------------------------------------------------------------------------
--  align_mps.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
--                   Johannes Rothe
--                   Nicolas Limpert
--             2016  Victor Mataré
--             2019  Sebastian Eltester
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
depends_skills     = { "motor_move" ,"gripper_commands"}
depends_interfaces = {
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"},
   {v = "line5", type="LaserLineInterface", id="/laser-lines/5"},
   {v = "line6", type="LaserLineInterface", id="/laser-lines/6"},
   {v = "line7", type="LaserLineInterface", id="/laser-lines/7"},
   {v = "line8", type="LaserLineInterface", id="/laser-lines/8"},
   {v = "line1_avg", type="LaserLineInterface", id="/laser-lines/1/moving_avg"},
   {v = "line2_avg", type="LaserLineInterface", id="/laser-lines/2/moving_avg"},
   {v = "line3_avg", type="LaserLineInterface", id="/laser-lines/3/moving_avg"},
   {v = "line4_avg", type="LaserLineInterface", id="/laser-lines/4/moving_avg"},
   {v = "line5_avg", type="LaserLineInterface", id="/laser-lines/5/moving_avg"},
   {v = "line6_avg", type="LaserLineInterface", id="/laser-lines/6/moving_avg"},
   {v = "line7_avg", type="LaserLineInterface", id="/laser-lines/7/moving_avg"},
   {v = "line8_avg", type="LaserLineInterface", id="/laser-lines/8/moving_avg"},
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
   {v = "laserline_switch", type = "SwitchInterface", id="laser-lines"},
}

documentation      = [==[Align precisely at the given coordinates, relative to the center of an MPS
@param tag_id (optional) Make sure to align only to the MPS with the given tag ID. Turn around and look for
                         it if it's not currently visible. Fail if it cannot be found. If unspecified, use
                         the first tag that's visible and aligned with some laser line.
@param x X offset, i.e. distance from the MPS in meters.
@param y (optional) Y offset from the center of the MPS. Defaults to 0.
]==]

local pre_conveyor_pose = { x = 0.05, y = 0.0, z = 0.03 }

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("fawkes.tfutils")
local llutils = require("fawkes.laser-lines_utils")
local tag_utils = require("tag_utils")

-- Tunables
local MIN_VIS_HIST_LINE=5 --15
local MIN_VIS_HIST_LINE_SEARCH=4
local MIN_VIS_HIST_TAG=5

local LINE_LENGTH_MIN=0.64
local LINE_LENGTH_MAX=0.71
local LINE_XDIST_MAX=0.6

local LINE_MATCH_TOLERANCE=0.3

local TURN_MOVES={
   { ori = math.pi/8},
   { ori = -math.pi/8},
}
local MAX_TRIES = 8


function tag_visible(vis_hist)
   local tag_iface = nil
   if fsm.vars.tag_id then
      tag_iface = tag_utils.iface_for_id(fsm.vars.tags, tag_info, fsm.vars.tag_id)
   else
      local min_dist = 1000
      for i = 0,15 do
         if tag_info:tag_id(i) ~= 0 and fsm.vars.tags[i+1]:visibility_history() > vis_hist
            and fsm.vars.tags[i+1]:translation(2) < min_dist
         then
            tag_iface = fsm.vars.tags[i+1]
            min_dist = tag_iface:translation(2)
            fsm.vars.tag_id = tag_info:tag_id(i)
         end
      end
   end

   if tag_iface and tag_iface:visibility_history() > vis_hist then
      print("Tag visible: " .. tag_iface:id())
      return true
   end
   return false
end


function want_search()
   return fsm.vars.search_idx <= MAX_TRIES or fsm.vars.turn_around_idx <= MAX_TRIES
end


fsm:define_states{ export_to=_M, closure={
      pose_error=pose_error, tag_visible=tag_visible, MIN_VIS_HIST_TAG=MIN_VIS_HIST_TAG,
      want_search=want_search, line_visible=line_visible },
   {"INIT",                   JumpState},
   {"GRIPPER_PRE_CONVEYOR",   SkillJumpState, skills={{"gripper_commands"}}, final_to="CHECK_TAG", fail_to="CHECK_TAG"},
   {"CHECK_TAG",              JumpState},
   {"FIND_TAG",               JumpState},
   {"SEARCH_LINES",           SkillJumpState, skills={{"motor_move"}}, final_to="CHECK_TAG", fail_to="FAILED"},
   {"TURN_AROUND",            SkillJumpState, skills={{"motor_move"}}, final_to="CHECK_TAG", fail_to="FAILED"},
   {"MATCH_LINE",             JumpState},
   {"NO_LINE",                JumpState},
   {"SEARCH_TAG_LINE",        SkillJumpState, skills={{"motor_move"}}, final_to="MATCH_LINE", fail_to="CHECK_TAG"},
   {"ALIGN_FAST",             SkillJumpState, skills={{"motor_move"}}, final_to="MATCH_AVG_LINE", fail_to="FAILED"},
   {"MATCH_AVG_LINE",         JumpState},
   {"ALIGN_PRECISE",          SkillJumpState, skills={{"motor_move"}}, final_to="ALIGN_TURN", fail_to="ALIGN_FAST"},
   {"ALIGN_TURN",             SkillJumpState, skills={{"motor_move"}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT",          "FAILED",          precond="not vars.x", desc="x argument missing"},
   {"INIT",          "GRIPPER_PRE_CONVEYOR",       cond=true},

   {"CHECK_TAG",     "MATCH_LINE",      cond="tag_visible(MIN_VIS_HIST_TAG)", desc="found tag"},
   {"CHECK_TAG",     "FIND_TAG",        timeout=2, desc="no tag"},

   {"FIND_TAG",      "SEARCH_LINES",    cond="want_search() and #vars.interesting_lines > 0", desc="search 4 tag"},
   {"FIND_TAG",      "MATCH_LINE",      cond="tag_visible(MIN_VIS_HIST_TAG)", desc="found tag"},
   {"FIND_TAG",      "TURN_AROUND",     timeout=1, desc="no interesting lines"},
   {"FIND_TAG",      "FAILED",          cond="not want_search()"},

   {"SEARCH_LINES",  "MATCH_LINE",      cond="tag_visible(MIN_VIS_HIST_TAG)", desc="found tag"},

   {"MATCH_LINE",   "ALIGN_FAST",       cond="vars.matched_line and tag_visible(MIN_VIS_HIST_TAG)"},
   {"MATCH_LINE",   "NO_LINE",          timeout=2, desc="lost line"},

   {"MATCH_AVG_LINE", "ALIGN_PRECISE",  timeout=1},

   {"NO_LINE",       "FAILED",          cond="vars.approached_tag"},
   {"NO_LINE",       "SEARCH_TAG_LINE", cond=true},

   {"ALIGN_FAST",    "FAILED",          cond="vars.align_attempts >= 3"}
}

function INIT:init()
   laserline_switch:msgq_enqueue(laserline_switch.EnableSwitchMessage:new())

   self.fsm.vars.y   = self.fsm.vars.y   or 0
   self.fsm.vars.ori = self.fsm.vars.ori or 0

   self.fsm.vars.lines = {}
   self.fsm.vars.lines[line1:id()] = line1
   self.fsm.vars.lines[line2:id()] = line2
   self.fsm.vars.lines[line3:id()] = line3
   self.fsm.vars.lines[line4:id()] = line4
   self.fsm.vars.lines[line5:id()] = line5
   self.fsm.vars.lines[line6:id()] = line6
   self.fsm.vars.lines[line7:id()] = line7
   self.fsm.vars.lines[line8:id()] = line8

   self.fsm.vars.lines_avg = {}
   self.fsm.vars.lines_avg[line1_avg:id()] = line1_avg
   self.fsm.vars.lines_avg[line2_avg:id()] = line2_avg
   self.fsm.vars.lines_avg[line3_avg:id()] = line3_avg
   self.fsm.vars.lines_avg[line4_avg:id()] = line4_avg
   self.fsm.vars.lines_avg[line5_avg:id()] = line5_avg
   self.fsm.vars.lines_avg[line6_avg:id()] = line6_avg
   self.fsm.vars.lines_avg[line7_avg:id()] = line7_avg
   self.fsm.vars.lines_avg[line8_avg:id()] = line8_avg

   -- Tracker to remember how often we looked for a tag at each line
   -- Required so we don't alternate between two lines that both don't have the desired tag.
   self.fsm.vars.lines_visited = {}
   for k,v in pairs(self.fsm.vars.lines) do
      self.fsm.vars.lines_visited[k] = 0
   end

   self.fsm.vars.interesting_lines = {}

   self.fsm.vars.tags = { tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7,
      tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15 }

   self.fsm.vars.search_idx = 0
   self.fsm.vars.turn_around_idx = 0
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
         local line_center = llutils.center(line, 0)
         local dist = math.vec_length(tag_laser.x - line_center.x, tag_laser.y - line_center.y)
         if line:visibility_history() >= MIN_VIS_HIST_LINE
            and dist < LINE_MATCH_TOLERANCE
            and dist < min_dist
         then
            min_dist = dist
            matched_line = line
            printf("Line dist: %f", dist)
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
         local center = llutils.center(line, 0)
         local ori = math.atan2(center.y, center.x)
         if line:visibility_history() >= MIN_VIS_HIST_LINE_SEARCH and
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


function FIND_TAG:init()
   self.fsm.vars.interesting_lines = get_interesting_lines(self.fsm.vars.lines)
end

function FIND_TAG:loop()
   self.fsm.vars.interesting_lines = get_interesting_lines(self.fsm.vars.lines)
end


function SEARCH_LINES:init()
   local min_dist = 1000
   local ori = nil
   local chosen_line = nil

   for i,l in ipairs(self.fsm.vars.interesting_lines) do
      local center = tfm.transform(llutils.center(l, 0), "/base_laser", "/base_link")
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
   print("TURN_AROUND turn_around_idx: " .. self.fsm.vars.turn_around_idx)
   print(TURN_MOVES[math.mod(self.fsm.vars.turn_around_idx, #TURN_MOVES) + 1]["ori"] * self.fsm.vars.turn_around_idx)
   self.args["motor_move"] = {ori = TURN_MOVES[math.mod(self.fsm.vars.turn_around_idx, #TURN_MOVES) + 1]["ori"] * self.fsm.vars.turn_around_idx}
   self.fsm.vars.turn_around_idx = self.fsm.vars.turn_around_idx + 1

   for k,v in pairs(self.fsm.vars.lines_visited) do
      self.fsm.vars.lines_visited[k] = 0
   end
end


function MATCH_LINE:init()
   self.fsm.vars.align_attempts = 0
end


function MATCH_LINE:loop()
   local tag = tag_utils.iface_for_id(self.fsm.vars.tags, tag_info, self.fsm.vars.tag_id)
   self.fsm.vars.matched_line = match_line(tag, self.fsm.vars.lines)
end


function SEARCH_TAG_LINE:init()
   self.fsm.vars.approached_tag = true
   local frame = tag_utils.frame_for_id(self.fsm.vars.tags, tag_info, self.fsm.vars.tag_id)

   if frame then
      self.args["motor_move"] = {
         x = 0.4,
         ori = math.pi,
         frame = frame
      }
   else
      -- This should never happen since we had the frame in the previous state
      -- and we didn't move. So let's fail miserably if it does happen.
      --
      self.args["motor_move"] = {
         frame = "LOST_TAG_FRAME"
      }
   end
end

function GRIPPER_PRE_CONVEYOR:init()
  self.args["gripper_commands"] = pre_conveyor_pose
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].wait = false
end

function ALIGN_FAST:init()
   local center = llutils.center(self.fsm.vars.matched_line)
   printf("center l: %f, %f, %f", center.x, center.y, center.ori)
   local center_bl = tfm.transform(center, "/base_laser", "/base_link")
   local p = llutils.point_in_front(center_bl, self.fsm.vars.x)
   
   printf("p    : %f %f %f", p.x, p.y, p.ori)

   self.args["motor_move"] = {
      x = p.x,
      y = p.y,
      ori = p.ori,
      tolerance = { x=0.05, y=0.03, ori=0.03 }
   }
end


function MATCH_AVG_LINE:loop()
   local tag = tag_utils.iface_for_id(fsm.vars.tags, tag_info, self.fsm.vars.tag_id)
   if tag then
      local tag_idx = string.sub(tag:id(), 13)
      self.fsm.vars.tag_frame_id = "/tag_" .. tag_idx

      local matched_line = match_line(tag, self.fsm.vars.lines_avg)

      if matched_line then
         -- Input coordinates are supposed to be relative to laser line, but there is no
         -- transform for laser lines, so we do this weird an stupid "manual transformation"
         local center = llutils.center(matched_line)
         --local center_bl = tfm.transform(center, "/base_laser", "/base_link")
         local p = llutils.point_in_front(center, self.fsm.vars.x)
         local p_tag = tfm.transform6D(
            {  x = p.x,
               y = p.y + (self.fsm.vars.y or 0),
               z = 0,
               ori = fawkes.tf.create_quaternion_from_yaw(matched_line:bearing()) },
            matched_line:frame_id(), "/odom"
         )
         if p_tag then
            self.fsm.vars.p_tag = p_tag
         end
      end
   end
end


function ALIGN_PRECISE:init()
   self.fsm.vars.align_attempts = self.fsm.vars.align_attempts + 1
  
   if self.fsm.vars.p_tag then
      printf("p_tag: %f %f %f",
         self.fsm.vars.p_tag.x,
         self.fsm.vars.p_tag.y,
         fawkes.tf.get_yaw(self.fsm.vars.p_tag.ori))

      self.args["motor_move"] = { 
         x = self.fsm.vars.p_tag.x,
         y = self.fsm.vars.p_tag.y,
         ori = fawkes.tf.get_yaw(self.fsm.vars.p_tag.ori),
         frame = "/odom"
      }
   else
      print("WARNING: lost laser line before precise alignment. Continuing with odometry only.")
      self.args["motor_move"].y = self.fsm.vars.y
   end
end


function ALIGN_TURN:init()
   self.args["motor_move"] = {
      ori = self.fsm.vars.ori
   }
end


