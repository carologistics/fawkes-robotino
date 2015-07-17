
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
depends_skills     = { "drive_to_global", "drive_to_local", "motor_move" }
depends_interfaces = {
  {v = "pose",      type="Position3DInterface", id="Pose"},
  {v = "zone_info", type="ZoneInterface",       id="/explore-zone/info"},
  {v = "zone_pose", type="Position3DInterface", id="/explore-zone/pose"},
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
  {v = "cluster_mps_1",   type = "Position3DInterface"},
  {v = "cluster_mps_2",   type = "Position3DInterface"},
  {v = "cluster_mps_3",   type = "Position3DInterface"},
  {v = "cluster_mps_4",   type = "Position3DInterface"},
  {v = "cluster_mps_5",   type = "Position3DInterface"},
  {v = "cluster_mps_6",   type = "Position3DInterface"},
  {v = "cluster_mps_7",   type = "Position3DInterface"},
  {v = "cluster_mps_8",   type = "Position3DInterface"},
  {v = "cluster_mps_9",   type = "Position3DInterface"},
  {v = "cluster_mps_10",  type = "Position3DInterface"},
}

local TIMEOUT = 1
local HIST_MIN_LINE = 5
local HIST_MIN_TAG  = 5
local ROBOT_WALL_DIST = 0.35
local TAG_DIST = 0.5
local MPS_OFFSET_TO_ZONE = 0.2
local WALL_MIN_X = -5.9
local WALL_MAX_X =  5.9
local WALL_MIN_Y =  0.1
local WALL_MAX_Y =  5.9

documentation      = [==[
Explores a zone defined by (x_min, y_min) and (x_max, y_max)
Searches for mps with the laser and tag vision

@param min_x  minimal x coordinate to explore
@param min_y  minimal y coordinate to explore
@param max_x  maximal x coordinate to explore
@param max_y  maximal y coordinate to explore

@param change_order if set to true, the points of a zone will be in different order (in case we are unable to find machines)
@param disable_cluster if set to true, the laser-cluster for fast decision will be disabled (in case we are unable to find machines

@param search_tags   list of tags to search fore (all others will be ignored), if nil all will be used

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require 'tf_module'
local tfutils = require("fawkes.tfutils")

function mps_in_zone(self, x, y, use_offset)
--  printf("Check MPS (" .. x .. ", " .. y .. ")")
  use_offset = use_offset or true
  mps_offset = 0
  if use_offset then
    mps_offset = MPS_OFFSET_TO_ZONE
  end
  -- x compared to the maximum - offset and minimum + offset ( depending on positive and negative numbers )
  if x > self.fsm.vars.max_x / math.abs(self.fsm.vars.max_x) * ( math.abs(self.fsm.vars.max_x) - mps_offset ) or
     x < self.fsm.vars.min_x / math.abs(self.fsm.vars.min_x) * ( math.abs(self.fsm.vars.min_x) + mps_offset ) or
     y > self.fsm.vars.max_y / math.abs(self.fsm.vars.max_y) * ( math.abs(self.fsm.vars.max_y) - mps_offset ) or
     y < self.fsm.vars.min_y / math.abs(self.fsm.vars.min_y) * ( math.abs(self.fsm.vars.min_y) + mps_offset ) then
--[[     printf("MPS in zone x (" .. 
                              self.fsm.vars.min_x / math.abs(self.fsm.vars.min_x) * ( math.abs(self.fsm.vars.min_x) + mps_offset ) ..
                              ", " ..
                              self.fsm.vars.max_x / math.abs(self.fsm.vars.max_x) * ( math.abs(self.fsm.vars.max_x) - mps_offset ) ..
                              ") y ( " ..
                              self.fsm.vars.min_y / math.abs(self.fsm.vars.min_y) * ( math.abs(self.fsm.vars.min_y) + mps_offset ) ..
                              ", " ..
                              self.fsm.vars.max_y / math.abs(self.fsm.vars.max_y) * ( math.abs(self.fsm.vars.max_y) - mps_offset ) ..
                              ")")
--]]
    return false
  end
  return true
end

function get_tag_id(self, if_ID)
  if_ID = string.sub(if_ID, 13)
  
  return tag_info:tag_id(if_ID)
end

function tag_searched(self, if_ID)
  if self.fsm.vars.search_tags == nil then
    return true
  end

  printf("TAG ID: " .. get_tag_id(self, if_ID))

  for k,v in pairs(self.fsm.vars.search_tags) do
    if v == get_tag_id(self, if_ID) then
      return true
    end
  end

  return false
end

function mps_visible_tag(self, hist_min, use_offset_to_boarder)
  hist_min = hist_min or HIST_MIN_TAG
  self.fsm.vars.tag_chosen = nil
  local tags_vis = {}
  for k,v in pairs( self.fsm.vars.tags ) do

    if v:visibility_history() >= hist_min then
      if tag_searched(self, v:id()) then
        local point_on_obj = {}

        local obj_map_6D = tfutils.transform6D({ x=v:translation(0), y=v:translation(1), z=v:translation(2),
                                                 ori={x=v:rotation(0), y=v:rotation(1), z=v:rotation(2), w=v:rotation(3)}
                                               }, v:frame(), "/base_link", v:timestamp())
      
        point_on_obj["frame_id"] = "/base_link"
        point_on_obj["x"]   = obj_map_6D.x
        point_on_obj["y"]   = obj_map_6D.y
        local q = fawkes.tf.Quaternion:new(obj_map_6D.ori.x, obj_map_6D.ori.y, obj_map_6D.ori.z, obj_map_6D.ori.w)
        point_on_obj["ori"] = math.normalize_mirror_rad( fawkes.tf.get_yaw(q) )

        local obj_map = tfm.transform({x=point_on_obj["x"], y=point_on_obj["y"], ori=point_on_obj["ori"]}, point_on_obj["frame_id"], "/map", v:timestamp())
        point_on_obj["x_map"]   = obj_map.x
        point_on_obj["y_map"]   = obj_map.y
        point_on_obj["ori_map"] = obj_map.ori
        point_on_obj["tag_id"]  = get_tag_id(self, v:id())

        if mps_in_zone(self, obj_map.x, obj_map.y, use_offset_to_boarder) then
          table.insert( tags_vis, point_on_obj )
          printf("Found MPS by tag at (%f, %f, %f)", point_on_obj["x_map"], point_on_obj["y_map"], point_on_obj["ori_map"])
        end
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

function mps_visible_laser(self, hist_min)
  hist_min = hist_min or HIST_MIN_LINE
  self.fsm.vars.line_chosen = nil
  -- visible
  local lines_vis = {}
  for k,v in pairs( self.fsm.vars.lines ) do

    if v:visibility_history() >= hist_min then
      local point_on_obj = {}
      point_on_obj["frame_id"] = v:frame_id()
      point_on_obj["x"]   = v:end_point_1(0) + ( v:end_point_2(0) - v:end_point_1(0) ) / 2
      point_on_obj["y"]   = v:end_point_1(1) + ( v:end_point_2(1) - v:end_point_1(1) ) / 2
      point_on_obj["ori"] = math.normalize_mirror_rad( math.pi + v:bearing() )
      local obj_map = tfm.transform({x=point_on_obj["x"], y=point_on_obj["y"], ori=point_on_obj["ori"]}, point_on_obj["frame_id"], "/map")
      point_on_obj["x_map"]   = obj_map.x
      point_on_obj["y_map"]   = obj_map.y
      point_on_obj["ori_map"] = obj_map.ori
      point_on_obj["tag_id"]  = -1
      
      if mps_in_zone( self, obj_map.x, obj_map.y ) then
        table.insert( lines_vis, point_on_obj )
          printf("Found MPS by laser-lines at (%f, %f, %f)", point_on_obj["x_map"], point_on_obj["y_map"], point_on_obj["ori_map"])
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

function cluster_visible(self)
  for k,v in pairs(self.fsm.vars.cluster) do
    local obj_map = tfm.transform({x=v:translation(0), y=v:translation(1), ori=0}, v:frame(), "/map")
    if v:visibility_history() > 0 and
       mps_in_zone(self, obj_map.x, obj_map.y) then
      return true
    end
  end
  return false
end

function poses_to_check(self)
  if table.getn(self.fsm.vars.poses_to_check) > self.fsm.vars.poses_to_check_iterator then
    return true
  else
    return false
  end
end

fsm:define_states{ export_to=_M,
  closure={mps_visible_tag=mps_visible_tag, mps_visible_laser=mps_visible_laser,cluster_visible=cluster_visible,poses_to_check=poses_to_check, HIST_MIN_TAG=HIST_MIN_TAG},
  {"INIT",                        JumpState},
  {"DRIVE_TO_ZONE",               SkillJumpState, skills={{drive_to_global}}, final_to="DECIDE_CLUSTER", fail_to="FAILED"},
  {"DECIDE_CLUSTER",              JumpState},
  {"TIMEOUT_CLUSTER",             JumpState},
  {"DECIDE_NEXT_POINT",           JumpState},
  {"DRIVE_TO_NEXT_EXPLORE_POINT", SkillJumpState, skills={{drive_to_local}}, final_to="WAIT_FOR_SENSORS", fail_to="FAILED"},
  {"FIX_INTERNAL_VARS",        SkillJumpState, skills={{motor_move}}, final_to="WAIT_FOR_SENSORS", fail_to="FAILED"},
  {"WAIT_FOR_SENSORS",            JumpState},
--  {"DRIVE_TO_POSSIBLE_MPS_PRE",   SkillJumpState, skills={{drive_to_local}}, final_to="WAIT_FOR_SENSORS", fail_to="FAILED"},
  {"DRIVE_TO_POSSIBLE_MPS",       SkillJumpState, skills={{drive_to_local}}, final_to="CHECK_SEE_TAG", fail_to="FAILED"},
  {"CHECK_SEE_TAG",               JumpState},
}

fsm:add_transitions{
  {"INIT",                "FAILED",                       cond="vars.parameter_nil",                            desc="one ore more parameter are nil"},
  {"INIT",                "DRIVE_TO_ZONE",                cond=true},
  --{"DRIVE_TO_ZONE",       "DRIVE_TO_POSSIBLE_MPS_PRE",    cond="mps_visible_tag(self, 1)",                      desc="saw tag on route, drive to there"},
  {"DECIDE_CLUSTER",      "DECIDE_NEXT_POINT",            cond="vars.disable_cluster ~= nil"},
  {"DECIDE_CLUSTER",      "TIMEOUT_CLUSTER",              cond=true},
  {"TIMEOUT_CLUSTER",     "WAIT_FOR_SENSORS",             cond=cluster_visible,                                 desc="cluster in zone, start checking"},
  --{"TIMEOUT_CLUSTER",     "FAILED",                       cond=cluster_visible,                                 desc="TODO this is for a test, remove me and add line above"},
  {"TIMEOUT_CLUSTER",     "FINAL",                        timeout=TIMEOUT,                                      desc="saw no cluster so stop checking zone"},
  --{"DECIDE_NEXT_POINT",   "DRIVE_TO_NEXT_EXPLORE_POINT",  cond="table.getn(self.fsm.vars.poses_to_check) ~= 0"},
  {"DECIDE_NEXT_POINT",   "DRIVE_TO_NEXT_EXPLORE_POINT",  cond=poses_to_check},
  {"DECIDE_NEXT_POINT",   "FINAL",                        cond=true},
  --{"DRIVE_TO_NEXT_EXPLORE_POINT", "FIX_INTERNAL_VARS", cond="mps_visible_tag(self, 1)",                      desc="saw tag on route, drive to there"},
  {"DRIVE_TO_NEXT_EXPLORE_POINT", "FIX_INTERNAL_VARS", cond="mps_visible_laser(self, 1)",                    desc="saw line on route, drive to there"},
  {"WAIT_FOR_SENSORS",    "DRIVE_TO_POSSIBLE_MPS",        cond=mps_visible_tag,                                 desc="saw tag, drive to"},
  {"WAIT_FOR_SENSORS",    "DRIVE_TO_POSSIBLE_MPS",        cond=mps_visible_laser,                               desc="saw laser, drive to"},
  {"WAIT_FOR_SENSORS",    "DECIDE_NEXT_POINT",            timeout=TIMEOUT,                                      desc="saw nothing, trying next point"},
  {"CHECK_SEE_TAG",       "FINAL",                        cond="mps_visible_tag(self, HIST_MIN_TAG, false)",    desc="see tag after alignmend"},
  {"CHECK_SEE_TAG",       "FAILED",                       timeout=TIMEOUT,                                      desc="can't see tag after position in front, wtf happend?"},
}

function INIT:init()
  self.fsm.vars.parameter_nil = false
  local wall_min_x = false
  local wall_min_y = false
  local wall_max_x = false
  local wall_max_y = false

  -- clear interfaces
  zone_info:set_search_state(zone_info.UNKNOWN)
  zone_info:set_tag_id(-1)

  zone_pose:set_visibility_history(-1)

  -- check params
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

    -- check for zone at walls
    if self.fsm.vars.min_x <= WALL_MIN_X then
      wall_min_x = true
    end
    if self.fsm.vars.min_y <= WALL_MIN_Y then
      wall_min_y = true
    end
    if self.fsm.vars.max_x >= WALL_MAX_X then
      wall_max_x = true
    end
    if self.fsm.vars.max_y >= WALL_MAX_Y then
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

    if self.fsm.vars.change_order then
      local poses_to_check_dir = {}
      local size = table.getn(self.fsm.vars.poses_to_check)
      for i = 1,size do
        table.insert(poses_to_check_dir, self.fsm.vars.poses_to_check[size+1-i])
      end
      self.fsm.vars.poses_to_check = poses_to_check_dir
    end
    
    local output = "Explore: \n"
    for i,value in ipairs(self.fsm.vars.poses_to_check) do
      output = output .. value.name .. ": (" .. value.x .. ", " ..value.y .. ", " .. value.ori .. ")\n"
    end
    printf(output)
    self.fsm.vars.poses_to_check_iterator = 1
  end

  -- point to check for clusters and to drive to with drive_to_global
  printf("Walls: x (" .. tostring(wall_min_x) .. tostring(wall_max_x) .. ") y (" .. tostring(wall_min_y) .. tostring(wall_max_y) .. ")")

  self.fsm.vars.point_cluster = {}
  if wall_max_x then
    if wall_max_y then
      self.fsm.vars.point_cluster.x = self.fsm.vars.min_x
      self.fsm.vars.point_cluster.y = self.fsm.vars.min_y
    else
      self.fsm.vars.point_cluster.x = self.fsm.vars.min_x
      self.fsm.vars.point_cluster.y = self.fsm.vars.max_y
    end
  else
    if wall_max_y then
      self.fsm.vars.point_cluster.x = self.fsm.vars.max_x
      self.fsm.vars.point_cluster.y = self.fsm.vars.min_y
    else
      self.fsm.vars.point_cluster.x = self.fsm.vars.max_x
      self.fsm.vars.point_cluster.y = self.fsm.vars.max_y
    end
  end
  -- ori will be from my point towards the center
  local dir_x = self.fsm.vars.mid_x - self.fsm.vars.point_cluster.x
  local dir_y = self.fsm.vars.mid_y - self.fsm.vars.point_cluster.y
  local ori = 0
  if dir_x >= 0 then
    ori = 0.785
  else
    ori = 2.355
  end
  if dir_y <= 0 then
    ori = ori * (-1)
  end
  self.fsm.vars.point_cluster.ori = ori

  -- interfaces in lists
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
  self.fsm.vars.cluster = {}
  self.fsm.vars.cluster[1]  = cluster_mps_1
  self.fsm.vars.cluster[2]  = cluster_mps_2
  self.fsm.vars.cluster[3]  = cluster_mps_3
  self.fsm.vars.cluster[4]  = cluster_mps_4
  self.fsm.vars.cluster[5]  = cluster_mps_5
  self.fsm.vars.cluster[6]  = cluster_mps_6
  self.fsm.vars.cluster[7]  = cluster_mps_7
  self.fsm.vars.cluster[8]  = cluster_mps_8
  self.fsm.vars.cluster[9]  = cluster_mps_9
  self.fsm.vars.cluster[10] = cluster_mps_10
end

function DRIVE_TO_ZONE:init()
  self.skills[1].x        = self.fsm.vars.point_cluster.x
  self.skills[1].y        = self.fsm.vars.point_cluster.y
  self.skills[1].ori      = self.fsm.vars.point_cluster.ori
  self.skills[1].just_ori = true
  printf("Pre Point Zone: (" .. self.skills[1].x .. ", " .. self.skills[1].y .. ", " .. self.skills[1].ori .. ")")
end

function DRIVE_TO_NEXT_EXPLORE_POINT:init()
  --local point = table.remove(self.fsm.vars.poses_to_check, 1)
  local point = self.fsm.vars.poses_to_check[self.fsm.vars.poses_to_check_iterator]
  self.fsm.vars.poses_to_check_iterator = self.fsm.vars.poses_to_check_iterator + 1
  printf("Explore from " .. point.name .. " (" .. point.x .. ", " .. point.y .. ", " .. point.ori .. ")")
  self.skills[1].x        = point.x
  self.skills[1].y        = point.y
  self.skills[1].ori      = point.ori
  self.skills[1].just_ori = true
end

function pose_in_front_of_mps_calculator(self, chosen, factor)
  factor = factor or 1

  x   = chosen["x_map"] + math.cos( chosen["ori_map"] ) * ( factor * TAG_DIST + ROBOT_WALL_DIST )
  y   = chosen["y_map"] + math.sin( chosen["ori_map"] ) * ( factor * TAG_DIST + ROBOT_WALL_DIST )
  ori = math.normalize_mirror_rad( chosen["ori_map"] + math.pi )
  return x, y, ori
end

function FIX_INTERNAL_VARS:init()
  self.fsm.vars.poses_to_check_iterator = self.fsm.vars.poses_to_check_iterator - 1
--[[ this is code to turn to the possivle MPS
  local ori = nil
  if self.fsm.vars.line_chosen ~=nil then
    printf("Turn to possible line")
    ori = math.normalize_mirror_rad( math.pi + self.fsm.vars.line_chosen["ori"] )
  elseif self.fsm.vars.tag_chosen ~= nil then
    printf("Turn to possible to tag")
    ori = self.fsm.vars.tag_chosen["ori"]
  else
    printf("Error, I have no tag and no line to turn to, continue as normal")
    ori = 0
  end
  self.skills[1].ori      = ori
  self.skills[1].ori      = nil
  self.skills[1].just_ori = true
--]]
end

--[[
function DRIVE_TO_POSSIBLE_MPS_PRE:init()
  local x   = nil
  local y   = nil
  local ori = nil
  local point_on_obj = {}
 
  local chosen = self.fsm.vars.tag_chosen

  x, y, ori = pose_in_front_of_mps_calculator(self, chosen, 2)
  printf("PrePoint drive: ( " .. x .. " , " .. y .. " , " .. ori .. " )")
  
  self.skills[1].x        = x
  self.skills[1].y        = y
  self.skills[1].ori      = ori
  self.skills[1].just_ori = true
end
--]]

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

  -- update interfaces
  zone_info:set_search_state(zone_info.MAYBE)
  zone_info:set_tag_id(chosen["tag_id"])
  zone_pose:set_visibility_history(1)
  zone_pose:set_translation(0, chosen["x_map"])
  zone_pose:set_translation(1, chosen["y_map"])
  printf("Found maybe ( " .. chosen["tag_id"] .. " ) (at far pose) at: ( " .. chosen["x_map"] .. ", " .. chosen["y_map"] .. "; " .. chosen["ori_map"] .. " )")
  local q = fawkes.tf.create_quaternion_from_yaw( chosen["ori_map"] )
  zone_pose:set_rotation( 0, q:x() )
  zone_pose:set_rotation( 1, q:y() )
  zone_pose:set_rotation( 2, q:z() )
  zone_pose:set_rotation( 3, q:w() )
  printf("Found maybe (at far pose) yaw: " .. chosen["ori_map"] .. "q = ( " .. q:x() .. ", " .. q:y() .. ", " .. q:z() .. ", " .. q:w() .. ")")

  -- get point to drive to (in front of MPS)
  x, y, ori = pose_in_front_of_mps_calculator(self, chosen)
  printf("Point drive:  ( " .. x .. " , " .. y .. " , " .. ori .. " )")
  
  self.skills[1].x        = x
  self.skills[1].y        = y
  self.skills[1].ori      = ori
  self.skills[1].just_ori = true
end

function FINAL:init()
  if self.fsm.vars.tag_chosen == nil then
    -- update interfaces
    zone_info:set_search_state(zone_info.NO)
  else
    local chosen = self.fsm.vars.tag_chosen
  
    -- update interfaces
    zone_info:set_search_state(zone_info.YES)
    zone_info:set_tag_id(chosen["tag_id"])
    zone_pose:set_visibility_history(1)
    zone_pose:set_translation(0, chosen["x_map"])
    zone_pose:set_translation(1, chosen["y_map"])
    printf("Found tag ( " .. chosen["tag_id"] .. " ) at: ( " .. chosen["x_map"] .. ", " .. chosen["y_map"] .. " )")
    local q = fawkes.tf.create_quaternion_from_yaw( chosen["ori_map"] )
    zone_pose:set_rotation( 0, q:x() )
    zone_pose:set_rotation( 1, q:y() )
    zone_pose:set_rotation( 2, q:z() )
    zone_pose:set_rotation( 3, q:w() )
    printf("Found tag yaw: " .. chosen["ori_map"] .. "q = ( " .. q:x() .. ", " .. q:y() .. ", " .. q:z() .. ", " .. q:w() .. ")")
  end
end
