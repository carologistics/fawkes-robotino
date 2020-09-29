
----------------------------------------------------------------------------
--  drive_to_machine_point.lua
--
--  Created: Sat Jul 12 13:25:47 2014
--  Copyright  2008       Tim Niemueller [www.niemueller.de]
--             2014-2015  Tobias Neumann
--             2018       Nicolas Limpert
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
name               = "drive_to_machine_point"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "goto","mps_align","motor_move" }
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
   {v = "navi", type = "NavigatorInterface", id="Navigator"},
   {v = "laserline_switch", type = "SwitchInterface", id="laser-lines"},
}

documentation      = [==[Drive to point with colisoin avoidance and approach a point relative to the laser line.
                         The taken laser line is the one close to the navgraph point.
                         If it fails to find the laserline in time it will try to perform mps_align with the given offsets.

Parameters:
      place:        Where to drive to
      x_at_mps:     x coordinate of point relative to the laserline
      option:       The point of interest in front of the MPS. This is used to determine the final y position.
                    It can be one of the following:
                     - CONVEYOR
                     - SHELF_RIGHT
                     - SHELF_MIDDLE
                     - SHELF_LEFT
                     - SLIDE
      y_at_mps:     (optional) y coordinate of point relative to the laserline (used for mps_align, if it's needed)
      ori_atmps:    (optional) orientation relative to the laserline
      tag_id:       (optional) The tag_id for mps_align in case it needs to be called.
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("fawkes.tfutils")
local llutils = require("fawkes.laser-lines_utils")

-- Tunables
-- meter threshold of laserline to navgraph point
local LINE_MATCH_TOLERANCE = config_get_float_or_default("/skills/drive_to_machine_point/LINE_MATCH_TOLERANCE", 1.2)
-- rad threshold of laserline to navgraph point
local LINE_MATCH_ANG_TOLERANCE = config_get_float_or_default("/skills/drive_to_machine_point/LINE_MATCH_ANG_TOLERANCE", 0.05)
-- meter threshold of laser to navgraph point
local NAVGRAPH_LIN_TOLERANCE = config_get_float_or_default("/skills/drive_to_machine_point/NAVGRAPH_LIN_TOLERANCE", 0.5)
-- rad threshold of laser to navgraph point
local NAVGRAPH_ANG_TOLERANCE = config_get_float_or_default("/skills/drive_to_machine_point/NAVGRAPH_ANG_TOLERANCE", 0.5)
-- minimum laser line length
local LINE_LENGTH_MIN = config_get_float_or_default("/skills/drive_to_machine_point/LINE_LENGTH_MIN", 0.64)
-- maximum laser line length
local LINE_LENGTH_MAX = config_get_float_or_default("/skills/drive_to_machine_point/LINE_MATCH_TOLERANCE", 0.72)
-- maximum velocity for motor_move
local MAX_VEL_MOTOR_MOVE = config_get_float_or_default("/skills/drive_to_machine_point/MAX_VEL_MOTOR_MOVE", 0.1)
-- number of tries to correct the pose in front of the laser line
local NUM_DIRECT_MPS_ALIGN_TRIES = config_get_int_or_default("/skills/drive_to_machine_point/NUM_DIRECT_MPS_ALIGN_TRIES", 4)
-- number of tries to correct the pose in front of the laser line
local CONVEYOR_IN_OUT_OFFSET = config_get_float_or_default("/skills/drive_to_machine_point/CONVEYOR_IN_OUT_OFFSET", 0.03)

-- Offsets for points of interest at the MPS.
-- Considered to be from the right of the MPS.
local OFFSET_CONVEYOR = config_get_float_or_default("/skills/drive_to_machine_point/OFFSET_CONVEYRO", 0)
local OFFSET_SHELF_RIGHT = config_get_float_or_default("/skills/drive_to_machine_point/OFFSET_SHELF_RIGHT", -0.3)
local OFFSET_SHELF_MIDDLE = config_get_float_or_default("/skills/drive_to_machine_point/OFFSET_SHELF_MIDDLE", -0.2)
local OFFSET_SHELF_LEFT = config_get_float_or_default("/skills/drive_to_machine_point/OFFSET_SHELF_LEFT", -0.1)
local OFFSET_SLIDE = config_get_float_or_default("/skills/drive_to_machine_point/OFFSET_SLIDE", -0.27)

local MIN_VIS_HIST_LINE = config_get_int_or_default("/skills/drive_to_machine_point/MIN_VIS_HIST_LINE", 5) --15
local MIN_VIS_HIST_LINE_SEARCH = config_get_float_or_default("/skills/drive_to_machine_point/MIN_VIS_HIST_LINE_SEARCH", 6) --15

function node_is_valid(self)
  if self.fsm.vars.point_set then
    return self.fsm.vars.point_valid
  end
  return true
end

function parameters_valid(self)
  if self.fsm.vars.x_at_mps then
    return true
    else
    return false
  end
end

-- Match tag to navgraph point
function match_line(self,lines)
   local matched_line = nil

   local navgraph_point_laser = tfm.transform6D(
         { x=self.fsm.vars.x, y=self.fsm.vars.y, z=0,
           ori=fawkes.tf.create_quaternion_from_yaw(self.fsm.vars.ori) },
           "/map", "/base_laser")


   for k,line in pairs(self.fsm.vars.lines) do
      local line_center = llutils.center(line, 0)
      local d_navgraph_to_line = math.vec_length(navgraph_point_laser.x - line_center.x, navgraph_point_laser.y - line_center.y)

      -- this is difference from the laser to navgraph point
      local d_laser_to_navgraph = math.vec_length(navgraph_point_laser.x, navgraph_point_laser.y)

      -- angular distance between the navgraph point (pointing towards the machine) and the line bearing
      -- (also pointing towards the machine when the robot is standing in front of it).
      -- This value should be very low when we are standing in front of the correct machine.
      local yaw = fawkes.tf.get_yaw(navgraph_point_laser.ori)
      local ang_dist = math.angle_distance(yaw, line:bearing())

      if line:visibility_history() >= MIN_VIS_HIST_LINE
         and d_navgraph_to_line < LINE_MATCH_TOLERANCE
         and d_laser_to_navgraph < NAVGRAPH_LIN_TOLERANCE
         and math.abs(yaw) < NAVGRAPH_ANG_TOLERANCE
         and ang_dist < LINE_MATCH_ANG_TOLERANCE
      then
         matched_line = line
      end
   end

   return matched_line
end

-- Return all lines which may be close to the navgraph point we're looking for
function get_interesting_lines(self)
   local rv = {}

   -- Shallow-copy input table so we don't delete values from it
   local good_lines = {}
   for k,v in pairs(self.fsm.vars.lines) do good_lines[k] = v end

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
--         printf("line " .. line:id() .. " " .. line:visibility_history() .. " " .. line:length())
         if line:visibility_history() >= MIN_VIS_HIST_LINE_SEARCH and
            line:length() >= LINE_LENGTH_MIN and
            line:length() <= LINE_LENGTH_MAX and
            self.fsm.vars.lines_visited[line:id()] < max_num_visited --and
         then
            table.insert(rv, line)
         end
      end
   end
   return rv
end

function laser_line_found(self)

  self.fsm.vars.interesting_lines = get_interesting_lines(self)

  self.fsm.vars.matched_line = match_line(self, self.fsm.vars.lines)

  if self.fsm.vars.matched_line ~= nil then
    printf ("found line: " .. self.fsm.vars.matched_line:id())
    self.fsm.vars.line_point = llutils.point_in_front(llutils.center(self.fsm.vars.matched_line), self.fsm.vars.x_at_mps)
  end

  return self.fsm.vars.line_point ~= nil
end

function laser_line_still_visible(self)
  if self.fsm.vars.matched_line ~= nil then
    return self.fsm.vars.lines[self.fsm.vars.matched_line:id()]:visibility_history() >= MIN_VIS_HIST_LINE_SEARCH
  else
    return false
  end
end

fsm:define_states{ export_to=_M,
  closure={navgraph=navgraph, node_is_valid=node_is_valid, parameters_valid=parameters_valid,
           laser_line_still_visible=laser_line_still_visible, laser_line_found=laser_line_found, NUM_DIRECT_MPS_ALIGN_TRIES=NUM_DIRECT_MPS_ALIGN_TRIES},

  {"INIT",                     JumpState},

  -- DRIVE_TO fails when the goal cannot be reached - should we still try to perform an MPS_ALIGN?
  {"SKILL_DRIVE_TO",           SkillJumpState, skills={{goto}},          final_to="SKILL_MPS_ALIGN", fail_to="FAILED"},
  {"SKILL_MPS_ALIGN",          SkillJumpState, skills={{mps_align}},         final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
  { "INIT",    "FAILED",                   cond="not navgraph", desc="navgraph not available" },
  { "INIT",    "FAILED",                   cond="not node_is_valid(self)",  desc="point invalid" },
  { "INIT",    "FAILED",                   cond="not parameters_valid(self)",  desc="parameters invalid" },
  { "INIT",    "FAILED",                   timeout=5,  desc="timeout" }, -- ONLY FOR TESTING!
  { "INIT",    "SKILL_DRIVE_TO",           cond=true },
  { "SKILL_DRIVE_TO", "SKILL_MPS_ALIGN",    cond="laser_line_found(self)" },
}

function INIT:init()
  laserline_switch:msgq_enqueue(laserline_switch.EnableSwitchMessage:new())

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

  self.fsm.vars.mps_align_tries = 0

   self.fsm.vars.ori_at_mps = self.fsm.vars.ori_at_mps or 0
   self.fsm.vars.option = self.fsm.vars.option or "CONVEYOR"
   self.fsm.vars.move_y = 0
   if self.fsm.vars.option == "CONVEYOR" then
     self.fsm.vars.move_y = OFFSET_CONVEYOR
     if string.match(self.fsm.vars.place, 'I$') then
        self.fsm.vars.move_y = self.fsm.vars.move_y + CONVEYOR_IN_OUT_OFFSET
     elseif string.match(self.fsm.vars.place, 'O$') then
        self.fsm.vars.move_y = self.fsm.vars.move_y - CONVEYOR_IN_OUT_OFFSET
     end
   elseif self.fsm.vars.option == "SHELF_RIGHT" then
     self.fsm.vars.move_y = OFFSET_SHELF_RIGHT
   elseif self.fsm.vars.option == "SHELF_MIDDLE" then
     self.fsm.vars.move_y = OFFSET_SHELF_MIDDLE
   elseif self.fsm.vars.option == "SHELF_LEFT" then
     self.fsm.vars.move_y = OFFSET_SHELF_LEFT
   elseif self.fsm.vars.option == "SLIDE" then
     self.fsm.vars.move_y = OFFSET_SLIDE
   end

  self.fsm.vars.lines_visited = {}
  for k,v in pairs(self.fsm.vars.lines) do
     self.fsm.vars.lines_visited[k] = 0
  end

  self.fsm.vars.interesting_lines = {}

  self.fsm.vars.point_set   = false
  self.fsm.vars.point_valid = false
  if self.fsm.vars.place ~= nil then
    self.fsm.vars.point_set = true
    self.fsm.vars.node = navgraph:node(self.fsm.vars.place)

    if self.fsm.vars.node:is_valid() then
      self.fsm.vars.point_valid = true
      self.fsm.vars.x = self.fsm.vars.node:x()
      self.fsm.vars.y = self.fsm.vars.node:y()
      if self.fsm.vars.ori == nil and self.fsm.vars.node:has_property("orientation") then
        self.fsm.vars.ori = self.fsm.vars.node:property_as_float("orientation");
      end
    end
  end
end

function SKILL_DRIVE_TO:init()
	self.args["goto"] = {x = self.fsm.vars.x, y = self.fsm.vars.y, ori = self.fsm.vars.ori}
end

function SKILL_MPS_ALIGN:init()
  self.args["mps_align"] = {tag_id =self.fsm.vars.tag_id, x = self.fsm.vars.x_at_mps, y = self.fsm.vars.move_y}
end
