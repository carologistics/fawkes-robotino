
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
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
MAX_TRIES = 3
ORI_TOLERANCE = 0.05

-- Variables
local navgraph = fawkes.load_yaml_navgraph("navgraph-llsf.yaml")
local tfm = require("tf_module")

local map_frame = "/map"
local line_frame = line1:frame_id()

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
function visible()
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST then
         return true
      end
   end
   return false
end

function pose_ok(self)
   printf("Richtiger Winkel: %f, Winkel aus dem Interface: %f",self.fsm.vars.ori_to_Drive, closest_line:bearing())
   return math.abs(self.fsm.vars.ori_to_drive) <= ORI_TOLERANCE
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


-- States
fsm:define_states{
   export_to=_M,
   closure={visible=visible, pose_ok=pose_ok, MAX_TRIES=MAX_TRIES},
   {"INIT", JumpState},
   {"ALIGN", SkillJumpState, skills={{"motor_move"}}, final_to="SETTLE_TIME", fail_to="FAILED"},
   {"SETTLE_TIME", JumpState},
   {"CHECK_POSE", JumpState}
}

-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", precond="not visible()", desc="no writer or vis_hist too low"},
   {"INIT", "FINAL", cond=true, desc="initialized"},
   {"SETTLE_TIME", "CHECK_POSE", timeout=1, desc="Let the visibility history increase"},
   {"CHECK_POSE", "ALIGN", cond="not pose_ok(self) and vars.tries < MAX_TRIES", desc="Align again"},
   {"CHECK_POSE", "FAILED", cond="vars.tries >= MAX_TRIES", desc="MAX_TRIES reached!"},
   {"CHECK_POSE", "FINAL", cond="pose_ok(self)", desc="We are now aligned to the walls"}
}

function INIT:init()
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
      local global_line_pos = tfm.transform({x=0, y=0, ori=o:bearing()}, line_frame, map_frame)
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
      -- TODO or richtig?
      elseif global_line_pos.ori >= (3*math.pi)/4 or global_line_pos.ori <= -(3*math.pi)/4 then
         table.insert(lines_sector_180, o)
      end
   end

   -- get the best line from sector 0°
   for _,o in ipairs(lines_sector_0) do
      local min_ori_diff = 10
      local sector_main_ori = 0
      -- transform the line ori to map frame
      local global_line_pos = tfm.transform({x=0, y=0, ori=o:bearing()}, line_frame, map_frame)
      -- get the ori difference of the line and the sector main ori to get the line with the best tolerance
      local ori_diff = math.abs(get_ori_diff(sector_main_ori,global_line_pos.ori))
      printf("sector 0, line_ori in /map: %f (Interface: %s), ori_diff: %f", global_line_pos.ori, o:id(), ori_diff)
      -- get the closest line to the given angle
      if ori_diff < min_ori_diff then
         min_ori_diff = ori_diff
         best_line_sector_0 = o
      end
   end
   if best_line_sector_0 ~= nil then
      local global_0 = tfm.transform({x=0, y=0, ori=best_line_sector_0:bearing()}, line_frame, map_frame)
      printf("Sector 0 Winner: %s, it has the global angle: %f", best_line_sector_0:id(), global_0.ori)
   else
      printf("No Line in the Sector 0")
   end

   -- get the best line from sector positive 90°
   for _,o in ipairs(lines_sector_p90) do
      local min_ori_diff = 10
      local sector_main_ori = math.pi/2
      -- transform the line ori to map frame
      local global_line_pos = tfm.transform({x=0, y=0, ori=o:bearing()}, line_frame, map_frame)
      -- get the ori difference of the line and the sector main ori to get the line with the best tolerance
      local ori_diff = math.abs(get_ori_diff(sector_main_ori,global_line_pos.ori))
      printf("sector +90, line_ori in /map: %f (Interface: %s), ori_diff: %f", global_line_pos.ori, o:id(), ori_diff)
      -- get the closest line to the given angle
      if ori_diff < min_ori_diff then
         min_ori_diff = ori_diff
         best_line_sector_p90 = o
      end
   end
   if best_line_sector_p90 ~= nil then
      local global_p90 = tfm.transform({x=0, y=0, ori=best_line_sector_p90:bearing()}, line_frame, map_frame)
      printf("Sector +90 Winner: %s, it has the global angle: %f", best_line_sector_p90:id(),global_p90.ori)
   else
      printf("No Line in the Sector +90")
   end

   -- get the best line from sector negative 90°
   for _,o in ipairs(lines_sector_n90) do
      local min_ori_diff = 10
      local sector_main_ori = -math.pi/2
      -- transform the line ori to map frame
      local global_line_pos = tfm.transform({x=0, y=0, ori=o:bearing()}, line_frame, map_frame)
      -- get the ori difference of the line and the sector main ori to get the line with the best tolerance
      local ori_diff = math.abs(get_ori_diff(sector_main_ori,global_line_pos.ori))
      printf("sector -90, line_ori in /map: %f (Interface: %s), ori_diff: %f", global_line_pos.ori, o:id(), ori_diff)
      -- get the closest line to the given angle
      if ori_diff < min_ori_diff then
         min_ori_diff = ori_diff
         best_line_sector_n90 = o
      end
   end
   if best_line_sector_n90 ~= nil then
      local global_n90 = tfm.transform({x=0, y=0, ori=best_line_sector_n90:bearing()}, line_frame, map_frame)
      printf("Sector -90 Winner: %s, it has the global angle: %f", best_line_sector_n90:id(), global_n90.ori)
   else
      printf("No Line in the Sector -90")
   end

   -- get the best line from sector 180°
   for _,o in ipairs(lines_sector_180) do
      local min_ori_diff = 10
      local sector_main_ori = math.pi
      -- transform the line ori to map frame
      local global_line_pos = tfm.transform({x=0, y=0, ori=o:bearing()}, line_frame, map_frame)
      -- get the ori difference of the line and the sector main ori to get the line with the best tolerance
      local ori_diff = math.abs(get_ori_diff(sector_main_ori,global_line_pos.ori))
      printf("sector 180, line_ori in /map: %f (Interface: %s), ori_diff: %f", global_line_pos.ori, o:id(), ori_diff)
      -- get the closest line to the given angle
      if ori_diff < min_ori_diff then
         min_ori_diff = ori_diff
         best_line_sector_180 = o
      end
   end

   if best_line_sector_180 ~= nil then
      local global_180 = tfm.transform({x=0, y=0, ori=best_line_sector_180:bearing()}, line_frame, map_frame)
      printf("Sector 180 Winner: %s, it has the global angle: %f", best_line_sector_180:id(), global_180.ori)
   else
      printf("No Line in the Sector 180")
   end

   --self.fsm.vars.ori_to_drive = closest_line:bearing()
   --printf("-----> turning to the line %s, ori_to_drive: %f", closest_line:id(), self.fsm.vars.ori_to_drive)
end

function ALIGN:init()
   self.skills[1].ori = self.fsm.vars.ori_to_drive
   self.skills[1].tolerance = {x=0.05, y=0.05, ori=0.02}
   self.fsm.vars.tries = self.fsm.vars.tries + 1
end

function CHECK_POSE:init()
   -- nochmal alle minimieren da es sein kann, dass in das interface inzwischen eine andere linie geschrieben wurde
   candidates = {}
   for _,o in ipairs(lines) do
      if o:visibility_history() >= MIN_VIS_HIST then
         table.insert(candidates, o)
      end
   end

   local min_ori = 10
   for _,o in ipairs(candidates) do
      local ori = math.abs(o:bearing())
      if ori < min_ori then
         min_ori = ori
         closest_line = o
      end
   end
   self.fsm.vars.ori_to_drive = closest_line:bearing()
end
