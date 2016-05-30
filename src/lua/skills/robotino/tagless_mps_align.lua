
----------------------------------------------------------------------------
--  align_laserlines.lua - Align to a line detected by laserlines plugin
--
--  Created: Thu Jul 3
--  Copyright  2014  Bahram Maleki-Fard
--                   Tim Niemueller
--                   Johannes Rothe
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
name               = "tagless_mps_align"
fsm                = SkillHSM:new{name=name, start="CHECK_INTERFACE", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = {
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"},
   {v = "line5", type="LaserLineInterface", id="/laser-lines/5"},
   {v = "line6", type="LaserLineInterface", id="/laser-lines/6"},
   {v = "line7", type="LaserLineInterface", id="/laser-lines/7"},
   {v = "line8", type="LaserLineInterface", id="/laser-lines/8"}
}

documentation      = [==[Align to straight line in laser data.
                     takes a navgraph place as input
                        example: align_laserlines{place="M1"}
                     or
                     just an orientation
                        example: align_laserines{ori=math.pi/2}
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
MIN_VIS_HIST = -50	-- old value 10
DIST_TO_MPS = 0.15+0.46/2

-- Variables
local tfm = require("tf_module")

local map_frame = "/map"

local lines = {
   line1,
   line2,
   line3,
   line4,
   line5,
   line6,
   line7,
   line8
}

local candidates = {}

-- Functions
function visible_and_writer()
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST and o:has_writer() then
         return true
      end
   end
   return false
end

--[[
function get_best_line(sector_main_ori, lines_in_sector)
   local best_line
   local ori_diff
   for _,o in ipairs(lines_in_sector) do
      local min_ori_diff = 10
      -- transform the line ori to map frame
      local global_line_pos = {}
      global_line_pos = tfm.transform({x=0, y=0, ori=o:bearing()}, o:frame_id(), map_frame)
      -- get the ori difference of the line and the sector main ori to get the line with the best tolerance
      ori_diff = math.abs(get_ori_diff(sector_main_ori,global_line_pos.ori))
      printf("sector %f, line_ori in /map: %f (Interface: %s), ori_diff: %f", sector_main_ori,  global_line_pos.ori, o:id(), ori_diff)
      -- get the closest line to the given angle
      if ori_diff < min_ori_diff then
         min_ori_diff = ori_diff
         best_line = o
      end
   end
   if best_line ~= nil then
      local global_0 = tfm.transform({x=0, y=0, ori=best_line:bearing()}, best_line:frame_id(), map_frame)
      printf("Sector %f Winner: %s, it has the global angle: %f", sector_main_ori, best_line:id(), global_0.ori)
      return best_line, ori_diff
   else
      printf("No Line in the Sector %f", sector_main_ori)
      return nil
   end
end
--]]

function get_best_line(line_list)
   local best_line
   local min_dist=100
   for _,o in ipairs(line_list) do
      x,y,ori=get_position_infront_line(o)
      if math.sqrt(x*x+y*y)<min_dist then
         best_line=o
         min_dist=math.sqrt(x*x+y*y)
      end
   end
   printf("%f",min_dist)
   return best_line
end

function get_position_infront_line(line)
   local a={line:end_point_1(0),line:end_point_1(1)}
   b={line:end_point_2(0),line:end_point_2(1)}
   m={(a[1]+b[1])/2,(a[2]+b[2])/2}
   normal_ori=math.pi+line:bearing()
   target={m[1]+math.cos(normal_ori)*DIST_TO_MPS,m[2]+math.sin(normal_ori)*DIST_TO_MPS}
   target_in_base_link = tfm.transform({x=target[1], y=target[2], ori=normal_ori-math.pi}, line:frame_id(), "/base_link")

   return target_in_base_link.x, target_in_base_link.y, target_in_base_link.ori
--[[
   if normal_ori<0 then
      return target[1], target[2], normal_ori+math.pi
   else
      return target[1], target[2], normal_ori-math.pi
   end
--]]
end

-- States
fsm:define_states{
   export_to=_M,
   closure={visible_and_writer=visible_and_writer},
   {"CHECK_INTERFACE", JumpState},
   {"INIT", JumpState},
   {"DETECT_LINE", JumpState},
   {"ALIGN", SkillJumpState, skills={{"motor_move"}}, final_to="DETECT_LINE", fail_to="FAILED"},
}

-- Transitions
fsm:add_transitions {
   {"CHECK_INTERFACE", "FAILED", timeout = 1, desc="no writer or vis_hist too low"},
   {"CHECK_INTERFACE", "INIT", cond=visible_and_writer},
   {"INIT", "DETECT_LINE", cond=visible_and_writer, desc="initialized"},
   {"DETECT_LINE", "ALIGN", cond="vars.line_detected"},
   {"DETECT_LINE", "DETECT_LINE", cond=visible_and_writer, desc="there still might be a laserline with low visibility"},
   {"DETECT_LINE", "FAILED", cond="true"},
   {"ALIGN", "FINAL", cond="vars.tries>=3"},
}

function INIT:init()
   self.fsm.vars.tries = 0
   self.fsm.vars.detect_tries = 0
end

function DETECT_LINE:init()
   local global_line_pos = {}
   local candidates = {}
   self.fsm.vars.line_detected=false

   -- Gather all lines with high visiblity history
   for _,o in ipairs(lines) do
      if o:visibility_history() >= math.max(10-2*self.fsm.vars.detect_tries,1) then
         table.insert(candidates, o)
         self.fsm.vars.line_detected=true
      end
   end

   if self.fsm.vars.line_detected then
      local best_line = get_best_line(candidates)
      self.fsm.vars.x_to_drive, self.fsm.vars.y_to_drive, self.fsm.vars.ori_to_drive = get_position_infront_line(best_line)
   end
   if not self.fsm.vars.line_detected then
      printf("no line detected:%i",math.max(10-self.fsm.vars.detect_tries,1))
   end
   self.fsm.vars.detect_tries=self.fsm.vars.detect_tries+1
end

function ALIGN:init()
   self.args["motor_move"] = {x = self.fsm.vars.x_to_drive, y = self.fsm.vars.y_to_drive, ori = self.fsm.vars.ori_to_drive,
				vel_trans = 0.2,
				tolerance = { x=0.002, y=0.002, ori=0.01 }}
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   self.fsm.vars.detect_tries = 0
end
