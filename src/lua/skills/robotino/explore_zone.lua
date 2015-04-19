
----------------------------------------------------------------------------
--  explore_zone.lua
--
--  Created: Thu Aug 14 14:32:47 2008
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
name               = "explore_zone"
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = { "drive_to_local" }
depends_interfaces = {
  {v = "pose",      type="Position3DInterface", id="Pose"},
  {v = "tag_0",     type = "Position3DInterface"},
  {v = "tag_1",     type = "Position3DInterface"},
  {v = "tag_2",     type = "Position3DInterface"},
  {v = "tag_3",     type = "Position3DInterface"},
  {v = "tag_4",     type = "Position3DInterface"},
  {v = "tag_5",     type = "Position3DInterface"},
  {v = "tag_6",     type = "Position3DInterface"},
  {v = "tag_7",     type = "Position3DInterface"},
  {v = "tag_8",     type = "Position3DInterface"},
  {v = "tag_9",     type = "Position3DInterface"},
  {v = "tag_10",    type = "Position3DInterface"},
  {v = "tag_11",    type = "Position3DInterface"},
  {v = "tag_12",    type = "Position3DInterface"},
  {v = "tag_13",    type = "Position3DInterface"},
  {v = "tag_14",    type = "Position3DInterface"},
  {v = "tag_15",    type = "Position3DInterface"},
  {v = "tag_info",  type = "TagVisionInterface"},
  {v = "line1",     type = "LaserLineInterface", id="/laser-lines/1"},
  {v = "line2",     type = "LaserLineInterface", id="/laser-lines/2"},
  {v = "line3",     type = "LaserLineInterface", id="/laser-lines/3"},
  {v = "line4",     type = "LaserLineInterface", id="/laser-lines/4"},
  {v = "line5",     type = "LaserLineInterface", id="/laser-lines/5"},
  {v = "line6",     type = "LaserLineInterface", id="/laser-lines/6"},
  {v = "line7",     type = "LaserLineInterface", id="/laser-lines/7"},
  {v = "line8",     type = "LaserLineInterface", id="/laser-lines/8"},
}

local TIMEOUT = 1
local HIST_MIN_LINE = 5
local HIST_MIN_TAG  = 5
local ROBOT_WALL_DIST = 0.3
local TAG_DIST = 0.5
local MPS_OFFSET_TO_ZONE = 0.2

documentation      = [==[
Explores a zone defined by (x_min, y_min) and (x_max, y_max)
Searches for mps with the laser and tag vision

@param min_x  minimal x coordinate to explore
@param min_y  minimal y coordinate to explore
@param max_x  maximal x coordinate to explore
@param max_y  maximal y coordinate to explore

@param tags_searched   list of tags to search fore (all others will be ignored), if nil all will be used

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require 'tf_module'
local tfutils = require("fawkes.tfutils")

function mps_in_zone(self, x, y)
  if x > self.fsm.vars.max_x - MPS_OFFSET_TO_ZONE or
     x < self.fsm.vars.min_x + MPS_OFFSET_TO_ZONE or
     y > self.fsm.vars.max_y - MPS_OFFSET_TO_ZONE or
     y < self.fsm.vars.min_y + MPS_OFFSET_TO_ZONE then
    return false
  end
  return true
end

function tag_searched(self, if_ID)
  if self.fsm.vars.tags_searched == nil then
    return true
  end

  if_ID = string.sub(if_ID, 13)
  printf("TAG ID: " .. tag_info:tag_id(if_ID))

  for k,v in pairs(self.fsm.vars.tags_searched) do
    if v == tag_info:tag_id(if_ID) then
      return true
    end
  end

  return false
end

function mps_visible_tag(self)
  self.fsm.vars.tag_chosen = nil
  local tags_vis = {}
  for k,v in pairs( self.fsm.vars.tags ) do

    if v:visibility_history() >= HIST_MIN_LINE then
      if tag_searched(self, v:id()) then
        local point_on_obj = {}

        local obj_map_6D = tfutils.transform6D({ x=v:translation(0), y=v:translation(1), z=v:translation(2),
                                                 ori={x=v:rotation(0), y=v:rotation(1), z=v:rotation(2), w=v:rotation(3)}
                                               }, v:frame(), "/base_link")
      
        point_on_obj["frame_id"] = "/base_link"
        point_on_obj["x"]   = obj_map_6D.x
        point_on_obj["y"]   = obj_map_6D.y
        local q = fawkes.tf.Quaternion:new(obj_map_6D.ori.x, obj_map_6D.ori.y, obj_map_6D.ori.z, obj_map_6D.ori.w)
        point_on_obj["ori"] = math.normalize_mirror_rad( fawkes.tf.get_yaw(q) )

        local obj_map = tfm.transform({x=point_on_obj["x"], y=point_on_obj["y"], ori=point_on_obj["ori"]}, point_on_obj["frame_id"], "/map")
        point_on_obj["x_map"]   = obj_map.x
        point_on_obj["y_map"]   = obj_map.y
        point_on_obj["ori_map"] = obj_map.ori
      
        table.insert( tags_vis, point_on_obj )
      end
    end
  end

  if table.getn( tags_vis ) <= 0 then
    return false
  end

  -- get closest
  self.fsm.vars.tag_chosen = tags_vis[1]
  local tag_chosen_dist = math.sqrt( self.fsm.vars.tag_chosen["x"]*self.fsm.vars.tag_chosen["x"] +
                                     self.fsm.vars.tag_chosen["y"]*self.fsm.vars.tag_chosen["y"])
  for k,v in pairs( tags_vis ) do
    local v_dist = math.sqrt( v["x"]*v["x"] + v["y"]*v["y"] )
    if tag_chosen_dist > v_dist then
      self.fsm.vars.tag_chosen = v
      tag_chosen_dist = v_dist
    end
  end
  return true
end

function mps_visible_laser(self)
  self.fsm.vars.line_chosen = nil
  -- visible
  local lines_vis = {}
  for k,v in pairs( self.fsm.vars.lines ) do

    if v:visibility_history() >= HIST_MIN_LINE then
      local point_on_obj = {}
      point_on_obj["frame_id"] = v:frame_id()
      point_on_obj["x"]   = v:end_point_1(0) + ( v:end_point_2(0) - v:end_point_1(0) ) / 2
      point_on_obj["y"]   = v:end_point_1(1) + ( v:end_point_2(1) - v:end_point_1(1) ) / 2
      point_on_obj["ori"] = math.normalize_mirror_rad( math.pi + v:bearing() )
      local obj_map = tfm.transform({x=point_on_obj["x"], y=point_on_obj["y"], ori=point_on_obj["ori"]}, point_on_obj["frame_id"], "/map")
      point_on_obj["x_map"]   = obj_map.x
      point_on_obj["y_map"]   = obj_map.y
      point_on_obj["ori_map"] = obj_map.ori
      
      if mps_in_zone( self, obj_map.x, obj_map.y ) then
        table.insert( lines_vis, point_on_obj )
      end
    end
  end
  if table.getn( lines_vis ) <= 0 then
    return false
  end

  -- get closest
  self.fsm.vars.line_chosen = lines_vis[1]
  local line_chosen_dist = math.sqrt( self.fsm.vars.line_chosen["x"]*self.fsm.vars.line_chosen["x"] +
                                      self.fsm.vars.line_chosen["y"]*self.fsm.vars.line_chosen["y"])
  for k,v in pairs( lines_vis ) do
    local v_dist = math.sqrt( v["x"]*v["x"] + v["y"]*v["y"] )
    if line_chosen_dist > v_dist then
      self.fsm.vars.line_chosen = v
      line_chosen_dist = v_dist
    end
  end
  return true
end

fsm:define_states{ export_to=_M,
  closure={mps_visible_tag=mps_visible_tag, mps_visible_laser=mps_visible_laser,},
  {"INIT",                        JumpState},
  {"DECIDE_NEXT_POINT",           JumpState},
  {"DRIVE_TO_NEXT_EXPLORE_POINT", SkillJumpState, skills={{drive_to_local}}, final_to="WAIT_FOR_SENSORS", fail_to="FAILED"},
  {"WAIT_FOR_SENSORS",            JumpState},
  {"DRIVE_TO_POSSIBLE_MPS",       SkillJumpState, skills={{drive_to_local}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
  {"INIT",                "FAILED",                       cond="vars.parameter_nil",                            desc="one ore more parameter are nil"},
  {"INIT",                "DECIDE_NEXT_POINT",            cond=true},
  {"DECIDE_NEXT_POINT",   "DRIVE_TO_NEXT_EXPLORE_POINT",  cond="table.getn(self.fsm.vars.poses_to_check) ~= 0"},
  {"DECIDE_NEXT_POINT",   "FINAL",                        cond=true},
  {"WAIT_FOR_SENSORS",    "DRIVE_TO_POSSIBLE_MPS",        cond=mps_visible_tag,                                 desc="saw tag, drive to"},
  {"WAIT_FOR_SENSORS",    "DRIVE_TO_POSSIBLE_MPS",        cond=mps_visible_laser,                               desc="saw laser, drive to"},
  {"WAIT_FOR_SENSORS",    "DECIDE_NEXT_POINT",            timeout=TIMEOUT,                                      desc="saw nothing, trying next point"},
}

function INIT:init()
  self.fsm.vars.parameter_nil = false

  if self.fsm.vars.min_x == nil or
     self.fsm.vars.min_y == nil or
     self.fsm.vars.max_x == nil or
     self.fsm.vars.max_y == nil then
     self.fsm.vars.parameter_nil = true
  else

    -- swap min/max if nessesary
    if self.fsm.vars.min_x > self.fsm.vars.max_x then
      self.fsm.vars.min_x, self.fsm.vars.max_x = self.fsm.vars.max_x, self.fsm.vars.min_x
    end
    if self.fsm.vars.min_y > self.fsm.vars.max_y then
      self.fsm.vars.min_y, self.fsm.vars.max_y = self.fsm.vars.max_y, self.fsm.vars.min_y
    end

    self.fsm.vars.mid_x = self.fsm.vars.min_x + ((self.fsm.vars.max_x - self.fsm.vars.min_x) / 2)
    self.fsm.vars.mid_y = self.fsm.vars.min_y + ((self.fsm.vars.max_y - self.fsm.vars.min_y) / 2)
    printf("Explore zone (" .. self.fsm.vars.min_x .. ", " .. self.fsm.vars.min_y .. ") (" .. self.fsm.vars.max_x .. ", " .. self.fsm.vars.max_y .. ")")

    local wall_min_x = false
    local wall_min_y = false
    local wall_max_x = false
    local wall_max_y = false
    -- check for zone at walls
    if self.fsm.vars.min_x <= -5.9 then
      wall_min_x = true
    end
    if self.fsm.vars.min_y <= 0.1 then
      wall_min_y = true
    end
    if self.fsm.vars.max_x >= 5.9 then
      wall_max_x = true
    end
    if self.fsm.vars.max_y >= 5.9 then
      wall_max_y = true
    end

    self.fsm.vars.poses_to_check = {}

    local y   = self.fsm.vars.max_y
    local ori = -1.57
    if wall_max_y then
      y   = self.fsm.vars.min_y
      ori = 1.57
    end

    -- top/bottom view form line
    table.insert(self.fsm.vars.poses_to_check, {name  = "Lr",
                                                x     = self.fsm.vars.max_x - ROBOT_WALL_DIST,
                                                y     = y,
                                                ori   = ori})
    table.insert(self.fsm.vars.poses_to_check, {name  = "Ll",
                                                x     = self.fsm.vars.min_x + ROBOT_WALL_DIST,
                                                y     = y,
                                                ori   = ori})

    -- middle view
    table.insert(self.fsm.vars.poses_to_check, {name  = "Ml",
                                                x     = self.fsm.vars.min_x + ROBOT_WALL_DIST,
                                                y     = self.fsm.vars.mid_y,
                                                ori   = 0})
    table.insert(self.fsm.vars.poses_to_check, {name  = "Mr",
                                                x     = self.fsm.vars.max_x - ROBOT_WALL_DIST,
                                                y     = self.fsm.vars.mid_y,
                                                ori   = 3.14})
    table.insert(self.fsm.vars.poses_to_check, {name  = "Mc",
                                                x     = self.fsm.vars.mid_x,
                                                y     = self.fsm.vars.mid_y,
                                                ori   = 3.14})
 
    local output = "Explore: \n"
    for key,value in pairs(self.fsm.vars.poses_to_check) do
      output = output .. value.name .. ": (" .. value.x .. ", " ..value.y .. ", " .. value.ori .. ")\n"
    end
    printf(output)
  end

  self.fsm.vars.tags = {}
  self.fsm.vars.tags[1]   = tag_0
  self.fsm.vars.tags[2]   = tag_1
  self.fsm.vars.tags[3]   = tag_2
  self.fsm.vars.tags[4]   = tag_3
  self.fsm.vars.tags[5]   = tag_4
  self.fsm.vars.tags[6]   = tag_5
  self.fsm.vars.tags[7]   = tag_6
  self.fsm.vars.tags[8]   = tag_7
  self.fsm.vars.tags[9]   = tag_8
  self.fsm.vars.tags[10]  = tag_9
  self.fsm.vars.tags[11]  = tag_10
  self.fsm.vars.tags[12]  = tag_11
  self.fsm.vars.tags[13]  = tag_12
  self.fsm.vars.tags[14]  = tag_13
  self.fsm.vars.tags[15]  = tag_14
  self.fsm.vars.tags[16]  = tag_15
  self.fsm.vars.lines = {}
  self.fsm.vars.lines[1]  = line1
  self.fsm.vars.lines[2]  = line2
  self.fsm.vars.lines[3]  = line3
  self.fsm.vars.lines[4]  = line4
  self.fsm.vars.lines[5]  = line5
  self.fsm.vars.lines[6]  = line6
  self.fsm.vars.lines[7]  = line7
  self.fsm.vars.lines[8]  = line8
end

function DRIVE_TO_NEXT_EXPLORE_POINT:init()
  local point = table.remove(self.fsm.vars.poses_to_check, 1)
  printf("Explore from " .. point.name .. " (" .. point.x .. ", " .. point.y .. ", " .. point.ori .. ")")
  self.skills[1].x        = point.x
  self.skills[1].y        = point.y
  self.skills[1].ori      = point.ori
  self.skills[1].just_ori = true
end

function DRIVE_TO_POSSIBLE_MPS:init()
  local x   = nil
  local y   = nil
  local ori = nil
  local point_on_obj = {}
 
  local chosen = nil
  --[[
  if self.fsm.vars.tag_chosen ~=nil then
    chosen = self.fsm.vars.tag_chosen
  elseif self.fsm.vars.line_chosen ~= nil then
    chosen = self.fsm.vars.line_chosen
  --]]
  if self.fsm.vars.line_chosen ~=nil then
    printf("Drive to line")
    chosen = self.fsm.vars.line_chosen
  elseif self.fsm.vars.tag_chosen ~= nil then
    printf("Drive to tag")
    chosen = self.fsm.vars.tag_chosen
  else
    printf("Error, I have no tag and no line to drive to")
  end
 
  x   = chosen["x_map"] + math.cos( chosen["ori_map"] ) * ( TAG_DIST + ROBOT_WALL_DIST )
  y   = chosen["y_map"] + math.sin( chosen["ori_map"] ) * ( TAG_DIST + ROBOT_WALL_DIST )
  ori = math.normalize_mirror_rad( chosen["ori_map"] + math.pi )

  printf("Point drive:  ( " .. x .. " , " .. y .. " , " .. ori .. " )")
  
  self.skills[1].x        = x
  self.skills[1].y        = y
  self.skills[1].ori      = ori
  self.skills[1].just_ori = true
end
