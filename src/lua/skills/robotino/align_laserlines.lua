
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
name               = "align_laserlines"
fsm                = SkillHSM:new{name=name, start="CHECK_INTERFACE", debug=true}
depends_skills     = {"motor_move"}
depends_interfaces = {
   {v = "line1", type="LaserLineInterface", id="/laser-lines/1"},
   {v = "line2", type="LaserLineInterface", id="/laser-lines/2"},
   {v = "line3", type="LaserLineInterface", id="/laser-lines/3"},
   {v = "line4", type="LaserLineInterface", id="/laser-lines/4"}
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
MIN_VIS_HIST = 10

-- Variables
local navgraph = fawkes.load_yaml_navgraph("navgraph-llsf.yaml")
local tfm = require("tf_module")

local map_frame = "/map"

local lines = {
   line1,
   line2,
   line3,
   line4
}

local lines_sector_0 = {}
local lines_sector_p90 = {}
local lines_sector_n90 = {}
local lines_sector_180 = {}

local best_line_sector_0
local best_line_sector_p90
local best_line_sector_n90
local best_line_sector_180

local candidates = {}
local closest_line

-- Functions
function visible_and_writer()
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST and o:has_writer() then
         return true
      end
   end
   return false
end

function get_ori_diff(ori, is_ori)
   local diff = 0
   if ori > is_ori then
      if ori - is_ori < math.pi then
         diff =  ori - is_ori
      else
         diff =  - 2.0 * math.pi + ori - is_ori
      end
   else
      if is_ori - ori < math.pi then
         diff = ori - is_ori
      else
         diff = 2.0 * math.pi - is_ori + ori;
      end
   end
   return diff
end

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

-- States
fsm:define_states{
   export_to=_M,
   closure={visible_and_writer=visible_and_writer},
   {"CHECK_INTERFACE", JumpState},
   {"INIT", JumpState},
   {"ALIGN", SkillJumpState, skills={{"motor_move"}}, final_to="FINAL", fail_to="FAILED"}
}

-- Transitions
fsm:add_transitions {
   {"CHECK_INTERFACE", "FAILED", timeout = 1, desc="no writer or vis_hist too low"},
   {"CHECK_INTERFACE", "FAILED", cond ="not visible_and_writer()", desc="no writer or vis_hist too low"},
   {"CHECK_INTERFACE", "INIT", cond=visible_and_writer},
   {"INIT", "ALIGN", cond=visible_and_writer, desc="initialized"}
}

function INIT:init()
   local global_line_pos = {}
   self.fsm.vars.tries = 0
   candidates = {}
   lines_sector_0 = {}
   lines_sector_p90 = {}
   lines_sector_n90 = {}
   lines_sector_180 = {}

   best_line_sector_0 = nil
   best_line_sector_p90 = nil
   best_line_sector_n90 = nil
   best_line_sector_180 = nil

   -- if there's just an ori given
   self.fsm.vars.target_ori = self.fsm.vars.ori
   -- if there is a navgraph place given
   if type(self.fsm.vars.place) == "string" then
      local node = navgraph:node(self.fsm.vars.place)
      if node:has_property("orientation") then
         self.fsm.vars.target_ori = node:property_as_float("orientation")
      end
   end

   -- Gather all lines with high visiblity history
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST then
         table.insert(candidates, o)
      end
   end

   -- sort the lines into the four different sectors
   for _,o in ipairs(candidates) do
      -- transform the line ori to map frame
      global_line_pos = tfm.transform({x=0, y=0, ori=o:bearing()}, o:frame_id(), map_frame)
      -- sector 0° (between -45° and +45°)
      if global_line_pos.ori < math.pi/4 and global_line_pos.ori > -math.pi/4 then
         table.insert(lines_sector_0, o)
      -- sector positive 90° (between +45° and +135°)
      elseif global_line_pos.ori >= math.pi/4 and global_line_pos.ori < (3*math.pi)/4 then
         table.insert(lines_sector_p90, o)
      -- sector negative 90° (between -45° and -135°)
      elseif global_line_pos.ori <= -math.pi/4 and global_line_pos.ori > -(3*math.pi)/4 then
         table.insert(lines_sector_n90, o)
      -- sector 180° (between +135° and -135°)
      elseif global_line_pos.ori >= (3*math.pi)/4 or global_line_pos.ori <= -(3*math.pi)/4 then
         table.insert(lines_sector_180, o)
      end
   end

   local ori_diff_0 = 0
   local ori_diff_p90 = 0
   local ori_diff_n90 = 0
   local ori_diff_180 = 0
   local min_ori_diff = 10
   local wanted_base_link_pos = tfm.transform({x=0, y=0, ori=self.fsm.vars.target_ori}, map_frame, "/base_link")
-- get the best lines from all sectors and while doing this get the best global line
-- get the best line from sector 0°
   if get_best_line(0, lines_sector_0) ~= nil then
      best_line_sector_0, ori_diff_0 = get_best_line(0, lines_sector_0)
      if ori_diff_0 < min_ori_diff then
         min_ori_diff = ori_diff_0
         closest_line = best_line_sector_0
         local base_link_line_pos = {}
         base_link_line_pos = tfm.transform({x=0, y=0, ori=closest_line:bearing()}, closest_line:frame_id(), "/base_link")
         self.fsm.vars.ori_to_drive = math.normalize_mirror_rad(base_link_line_pos.ori + get_ori_diff(wanted_base_link_pos.ori, base_link_line_pos.ori))
      end
   end
-- get the best line from sector +90°
   if get_best_line(math.pi/2, lines_sector_p90) ~= nil then
      best_line_sector_p90, ori_diff_p90 = get_best_line(math.pi/2, lines_sector_p90)
      if ori_diff_p90 < min_ori_diff then
         min_ori_diff = ori_diff_p90
         closest_line = best_line_sector_p90
         local base_link_line_pos = {}
         base_link_line_pos = tfm.transform({x=0, y=0, ori=closest_line:bearing()}, closest_line:frame_id(), "/base_link")
         self.fsm.vars.ori_to_drive = math.normalize_mirror_rad(base_link_line_pos.ori + get_ori_diff(wanted_base_link_pos.ori, base_link_line_pos.ori))
      end
   end
-- get the best line from sector -90°
   if get_best_line(-math.pi/2, lines_sector_n90) ~= nil then
      best_line_sector_n90, ori_diff_n90 = get_best_line(-math.pi/2, lines_sector_n90)
      if ori_diff_n90 < min_ori_diff then
         min_ori_diff = ori_diff_n90
         closest_line = best_line_sector_n90
         local base_link_line_pos = {}
         base_link_line_pos = tfm.transform({x=0, y=0, ori=closest_line:bearing()}, closest_line:frame_id(), "/base_link")
         self.fsm.vars.ori_to_drive = math.normalize_mirror_rad(base_link_line_pos.ori + get_ori_diff(wanted_base_link_pos.ori, base_link_line_pos.ori))
      end
   end
-- get the best line from sector 180°
   if get_best_line(math.pi, lines_sector_180) ~= nil then
      best_line_sector_180, ori_diff_180 = get_best_line(math.pi, lines_sector_180)
      if ori_diff_180 < min_ori_diff then
         min_ori_diff = ori_diff_180
         closest_line = best_line_sector_180
         local base_link_line_pos = {}
         base_link_line_pos = tfm.transform({x=0, y=0, ori=closest_line:bearing()}, closest_line:frame_id(), "/base_link")
         self.fsm.vars.ori_to_drive = math.normalize_mirror_rad(base_link_line_pos.ori + get_ori_diff(wanted_base_link_pos.ori, base_link_line_pos.ori))
      end
   end

   --self.fsm.vars.ori_to_drive = closest_line:bearing()
   printf("-----> And the WINNER is %s", closest_line:id())
   printf("ori_to_drive: %f", self.fsm.vars.ori_to_drive)
end

function ALIGN:init()
   self.skills[1].ori = self.fsm.vars.ori_to_drive
   self.skills[1].tolerance = {x=0.05, y=0.05, ori=0.02}
   self.fsm.vars.tries = self.fsm.vars.tries + 1
end
