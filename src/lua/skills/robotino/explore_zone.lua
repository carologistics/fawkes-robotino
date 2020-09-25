
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "goto", "motor_move" }
depends_interfaces = {
   {v = "zone_info", type="ZoneInterface",       id="/explore-zone/info", writing=true},
   {v = "bb_found_tag", type="Position3DInterface", id="/explore-zone/found-tag", writing=true},
	{v = "tag_0",    type = "Position3DInterface", id="/tag-vision/0"},
	{v = "tag_1",    type = "Position3DInterface", id="/tag-vision/1"},
	{v = "tag_2",    type = "Position3DInterface", id="/tag-vision/2"},
	{v = "tag_3",    type = "Position3DInterface", id="/tag-vision/3"},
	{v = "tag_4",    type = "Position3DInterface", id="/tag-vision/4"},
	{v = "tag_5",    type = "Position3DInterface", id="/tag-vision/5"},
	{v = "tag_6",    type = "Position3DInterface", id="/tag-vision/6"},
	{v = "tag_7",    type = "Position3DInterface", id="/tag-vision/7"},
	{v = "tag_8",    type = "Position3DInterface", id="/tag-vision/8"},
	{v = "tag_9",    type = "Position3DInterface", id="/tag-vision/9"},
	{v = "tag_10",   type = "Position3DInterface", id="/tag-vision/10"},
	{v = "tag_11",   type = "Position3DInterface", id="/tag-vision/11"},
	{v = "tag_12",   type = "Position3DInterface", id="/tag-vision/12"},
	{v = "tag_13",   type = "Position3DInterface", id="/tag-vision/13"},
	{v = "tag_14",   type = "Position3DInterface", id="/tag-vision/14"},
	{v = "tag_15",   type = "Position3DInterface", id="/tag-vision/15"},
	{v = "tag_info", type = "TagVisionInterface", id="/tag-vision/info"},
   {v = "line1",    type = "LaserLineInterface", id="/laser-lines/1"},
   {v = "line2",    type = "LaserLineInterface", id="/laser-lines/2"},
   {v = "line3",    type = "LaserLineInterface", id="/laser-lines/3"},
   {v = "line4",    type = "LaserLineInterface", id="/laser-lines/4"},
   {v = "line5",    type = "LaserLineInterface", id="/laser-lines/5"},
   {v = "line6",    type = "LaserLineInterface", id="/laser-lines/6"},
   {v = "line7",    type = "LaserLineInterface", id="/laser-lines/7"},
   {v = "line8",    type = "LaserLineInterface", id="/laser-lines/8"},
   {v = "cluster1", type = "Position3DInterface", id="/laser-cluster/mps/1"},
   {v = "cluster2", type = "Position3DInterface", id="/laser-cluster/mps/2"},
   {v = "cluster3", type = "Position3DInterface", id="/laser-cluster/mps/3"},
   {v = "cluster4", type = "Position3DInterface", id="/laser-cluster/mps/4"},
   {v = "cluster5", type = "Position3DInterface", id="/laser-cluster/mps/5"},
   {v = "cluster6", type = "Position3DInterface", id="/laser-cluster/mps/6"},
   {v = "cluster7", type = "Position3DInterface", id="/laser-cluster/mps/7"},
   {v = "cluster8", type = "Position3DInterface", id="/laser-cluster/mps/8"},
   {v = "cluster9", type = "Position3DInterface", id="/laser-cluster/mps/9"},
   {v = "cluster10", type = "Position3DInterface", id="/laser-cluster/mps/10"},
}

documentation      = [==[
Searches for mps with the laser and tag vision

@param zone          The zone to search

@param search_tags   list of tags to search for (all others will be ignored), if nil all will be used

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("fawkes.tfutils")
local llutils = require("fawkes.laser-lines_utils")
local tag_utils = require("tag_utils")

-- Tunables
local ZONE_MARGIN = config:get_float_or_default("/skills/explore_zone/ZONE_MARGIN", 0.05)
local CAM_ANGLE = config:get_float_or_default("/skills/explore_zone/CAM_ANGLE", 0.4)
local MIN_VIS_HIST =config:get_int_or_default("/skills/explore_zone/MIN_VIS_HIST", 10)
local MAX_ATTEMPTS = config:get_int_or_default("/skills/explore_zone/MAX_ATTEMPTS", 4)

-- Maximum reachable coordinates (i.e. usable playing field dimensions)
-- X direction is mirrored, so no minimum here
local X_MAX = config:get_float_or_default("/skills/explore_zone/X_MAX", 6.6) 
local Y_MIN = config:get_float_or_default("/skills/explore_zone/Y_MIN", 0.3)
local Y_MAX = config:get_float_or_default("/skills/explore_zone/Y_MAX", 7.6)

-- When we can't find a laser-line in the zone, we try looking at the zone
-- from these coordinates, relative to the center.
local ZONE_CORNERS = {
   { x = config:get_float_or_default("/skills/explore_zone/ZONE_CORNERS/x_1", 0.7), 
     y = config:get_float_or_default("/skills/explore_zone/ZONE_CORNERS/y_1", 0.07)},
   { x = config:get_float_or_default("/skills/explore_zone/ZONE_CORNERS/x_2", -0.07), 
     y = config:get_float_or_default("/skills/explore_zone/ZONE_CORNERS/y_2", 0.7)},
   { x = config:get_float_or_default("/skills/explore_zone/ZONE_CORNERS/x_3", -0.76), 
     y = config:get_float_or_default("/skills/explore_zone/ZONE_CORNERS/y_3", -0.81)},
}

-- Maximum difference between tag and line trans/rot
local TAG_LINE_TOLERANCE = {
   trans = config:get_float_or_default("/skills/explore_zone/TAG_LINE_TOLERANCE_trans", 0.2),
   rot = config:get_float_or_default("/skills/explore_zone/TAG_LINE_TOLERANCE_rot", 0.05)
}


function args_ok()
   return fsm.vars.zone and string.match(fsm.vars.zone, "[MC][-]Z[1-7][1-8]")
end


function in_zone(x, y)
   return math.abs(x - fsm.vars.x) < 0.5 - ZONE_MARGIN
      and math.abs(y - fsm.vars.y) < 0.5 - ZONE_MARGIN
end


function within_map(p)
   if math.abs(p.x) >= 5 then -- Take care of the insertion zone (|x| >= 5)!
      return p.y >= Y_MAX + 1
   else
      return math.abs(p.x) <= X_MAX and p.y >= Y_MIN and p.y <= Y_MAX
   end
end


function line_in_zone(lines)
   for k,line in pairs(lines) do
      if line:visibility_history() >= MIN_VIS_HIST then
         local center = llutils.center(line)
         local center_map = tfm.transform(center, line:frame_id(), "map")
         if in_zone(center_map.x, center_map.y) then
            return line
         end
      end
   end
   return nil
end


function cluster_in_zone(clusters)
   for i,cluster in pairs(clusters) do
      if cluster:visibility_history() >= MIN_VIS_HIST then
         local cluster_map = tfm.transform(
            { x = cluster:translation(0), y = cluster:translation(1), ori = 0 },
            cluster:frame(), "map"
         )
         if in_zone(cluster_map.x, cluster_map.y) then
            return cluster_map
         end
      end
   end
   return nil
end


function local_bearing(x, y)
   local p_bl = tfm.transform(
      {x = x, y = y, ori = 0},
      "map", "base_link")
   return math.atan2(p_bl.y, p_bl.x)
end


function found_tag()
   local tag_ifs = tag_utils.matching_tags(
      fsm.vars.tags, tag_info, fsm.vars.tag_id_set)
   for id,tag in pairs(tag_ifs) do
      if tag:visibility_history() >= MIN_VIS_HIST then
         local line = line_in_zone(fsm.vars.lines)
         if line and line:visibility_history() >= MIN_VIS_HIST then
            -- IF we see a line in this zone, we make sure the tag is roughly aligned with it,
            -- otherwise we assume the tag pose is somehow borked, e.g. flipped orientation as
            -- it happens with the current ALVAR lib.
            local line_c = llutils.center(line)

            local tag_laser = tfm.transform6D(
               { x = 0, y = 0, z = 0, ori = fawkes.tf.create_quaternion_from_yaw(math.pi) },
               tag_utils.frame_for_id(fsm.vars.tags, tag_info, id), line:frame_id()
            )
            local line_c_tag = tfm.transform6D(
               { x = line_c.x, y = line_c.y, z = 0,
                 ori = fawkes.tf.create_quaternion_from_rpy(0, 0, line:bearing() - math.pi)
               }, line:frame_id(), tag_utils.frame_for_id(fsm.vars.tags, tag_info, id)
            )
            local d_trans = math.vec_length(line_c_tag.x, line_c_tag.y)
            -- WORKAROUND: ignore ori diff, use line ori instead
            -- local d_rot = math.angle_distance(line:bearing(), fawkes.tf.get_yaw(tag_laser.ori))

            if d_trans <= TAG_LINE_TOLERANCE.trans then
               local line_bearing_map = tfm.transform(
                  { x = 0, y = 0, ori = math.normalize_mirror_rad(line:bearing() + math.pi) }, line:frame_id(), "map"
               )
               local tag_map = tfm.transform6D(
                  { x = 0, y = 0, z = 0,
                     ori = fawkes.tf.Quaternion:getIdentity()
                  },
                  tag_utils.frame_for_id(fsm.vars.tags, tag_info, id),
                  "map"
               )
               if not tag_map then
                 printf("Discarding tag #%d, cannot transform", id)
               else
                 if in_zone(tag_map.x, tag_map.y) then
                   local yaw = fawkes.tf.get_yaw(tag_map.ori)
                   if yaw == yaw then -- false for NAN
                      -- Rescale & Discretize angle from 0..315°
                      print_debug("Yaw 1: " .. yaw)
                      if id % 2 == 0 then
                         yaw = yaw + math.pi
                         print_debug("Yaw 2: " .. yaw)
                      end
                      if yaw < 0 then
                         yaw = 2 * math.pi + yaw
                         print_debug("Yaw 3: ".. yaw)
                      end
                      local yaw_discrete = math.round(yaw / math.pi * 4) * 45
                      if yaw_discrete == 360 then yaw_discrete = 0 end
                      zone_info:set_zone(fsm.vars.zone)
                      zone_info:set_orientation(yaw_discrete)
                      zone_info:set_tag_id(id)
                      zone_info:set_search_state(zone_info.YES)
                      bb_found_tag:set_translation(0, tag_map.x)
                      bb_found_tag:set_translation(1, tag_map.y)
                      bb_found_tag:set_translation(2, tag_map.z)
                      local line_bearing_map_q = fawkes.tf.create_quaternion_from_yaw(line_bearing_map.ori)
                      bb_found_tag:set_rotation(0, line_bearing_map_q:x())
                      bb_found_tag:set_rotation(1, line_bearing_map_q:y())
                      bb_found_tag:set_rotation(2, line_bearing_map_q:z())
                      bb_found_tag:set_rotation(3, line_bearing_map_q:w())
                      bb_found_tag:set_frame("map")
                      bb_found_tag:set_visibility_history(tag:visibility_history())
                      fsm.vars.found_something = true
                      return true
                   end
                 else
                   printf("Discarding tag #%d, misaligned. Is at %f\t%f", id, tag_map.x, tag_map.y)
                 end
               end
            end

            --[[ if d_trans > TAG_LINE_TOLERANCE.trans or d_rot > TAG_LINE_TOLERANCE.rot then
               printf("Discarding tag #%d, misaligned by %f, %f.", id, d_trans, d_rot)
               return false
            end ]]--
         end
      end
   end
   return false
end

function lost_tag()
   local tag_ifs = tag_utils.matching_tags(
      fsm.vars.tags, tag_info, fsm.vars.tag_id_set)
   for id,tag in pairs(tag_ifs) do
      if tag:visibility_history() > -3 then
         return false
      end
   end
   return true
end

fsm:define_states{ export_to=_M,
   closure={args_ok=args_ok, local_bearing=local_bearing, CAM_ANGLE=CAM_ANGLE, ZONE_CORNERS=ZONE_CORNERS,
      MAX_ATTEMPTS=MAX_ATTEMPTS, found_tag=found_tag},
   {"INIT", JumpState},
   {"WAIT_FOR_TAG", JumpState},
   {"TURN", SkillJumpState, skills={{motor_move}}, final_to="WAIT_FOR_TAG", fail_to="FAILED"},
   {"WAIT_FOR_SENORS", JumpState},
   {"FIND_LINE", SkillJumpState, skills={{goto}}, final_to="WAIT_FOR_TAG", fail_to="WAIT_FOR_TAG"},
   {"PICK_VISTA_POINT", JumpState},
   {"GOTO_LINE", SkillJumpState, skills={{goto}}, final_to="WAIT_FOR_TAG", fail_to="WAIT_FOR_TAG"},
   {"GOTO_VISTA_POINT", SkillJumpState, skills={{goto}}, final_to="WAIT_FOR_TAG", fail_to="WAIT_FOR_TAG"},
   {"WAIT_AMCL", JumpState}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not args_ok()", desc="invalid arguments"},
   {"INIT", "TURN", cond="vars.line_center"},
   {"INIT", "TURN", cond="local_bearing(vars.x, vars.y) > CAM_ANGLE"},
   {"INIT", "WAIT_FOR_TAG", cond=true},
   {"WAIT_FOR_TAG", "WAIT_AMCL", cond=found_tag, desc="found tag"},
   {"WAIT_FOR_TAG", "WAIT_FOR_SENORS", timeout=1},
   {"WAIT_FOR_SENORS", "WAIT_AMCL", cond=found_tag, desc="found tag"},
   {"WAIT_FOR_SENORS", "FAILED", cond="vars.attempts >= MAX_ATTEMPTS", desc="give up"},
   {"WAIT_FOR_SENORS", "GOTO_LINE", cond="vars.line_vista"},
   {"WAIT_FOR_SENORS", "FIND_LINE", cond="vars.cluster_vista"},
   {"WAIT_FOR_SENORS", "PICK_VISTA_POINT", timeout=2},
   {"PICK_VISTA_POINT", "GOTO_VISTA_POINT", cond="vars.zone_corner"},
   {"PICK_VISTA_POINT", "FAILED", cond="not vars.zone_corner"},
   {"GOTO_VISTA_POINT", "WAIT_AMCL", cond=found_tag, desc="found tag"},
   {"GOTO_LINE", "WAIT_AMCL", cond=found_tag, desc="found tag"},
   {"WAIT_AMCL", "WAIT_FOR_SENORS", cond=lost_tag, desc="lost tag"},
   {"WAIT_AMCL", "FINAL", timeout=1},
}


function INIT:init()
   zone_info:set_zone(self.fsm.vars.zone)
   zone_info:set_search_state(zone_info.UNKNOWN)
   zone_info:set_tag_id(-1)
   zone_info:set_orientation(-1)

   bb_found_tag:set_visibility_history(-1)

   self.fsm.vars.x = tonumber(string.sub(self.fsm.vars.zone, 4, 4)) - 0.5
   self.fsm.vars.y = tonumber(string.sub(self.fsm.vars.zone, 5, 5)) - 0.5
   if string.sub(self.fsm.vars.zone, 1, 1) == "M" then
      self.fsm.vars.x = -self.fsm.vars.x
   end

   if self.fsm.vars.search_tag then
      self.fsm.vars.tag_id_set = {}
      for k,v in self.fsm.vars.search_tag do
         self.fsm.vars.tag_id_set[k] = true
      end
   end

   self.fsm.vars.lines = {}
   self.fsm.vars.lines[line1:id()] = line1
   self.fsm.vars.lines[line2:id()] = line2
   self.fsm.vars.lines[line3:id()] = line3
   self.fsm.vars.lines[line4:id()] = line4
   self.fsm.vars.lines[line5:id()] = line5
   self.fsm.vars.lines[line6:id()] = line6
   self.fsm.vars.lines[line7:id()] = line7
   self.fsm.vars.lines[line8:id()] = line8

   self.fsm.vars.tags = { tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7,
      tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15 }
   self.fsm.vars.clusters = { cluster1, cluster2, cluster3, cluster4, cluster5,
      cluster6, cluster7, cluster8, cluster9, cluster10 } 

   self.fsm.vars.zone_corner_idx = 1
   self.fsm.vars.attempts = 0

   local line = line_in_zone(self.fsm.vars.lines)
   if line then
      self.fsm.vars.line_center = llutils.center(line)
   end
end


function TURN:init()
   if self.fsm.vars.line_center then
      self.args["motor_move"].ori = math.atan2(self.fsm.vars.line_center.y, self.fsm.vars.line_center.x)
   else
      self.args["motor_move"].ori = local_bearing(self.fsm.vars.x, self.fsm.vars.y)
   end
end


function WAIT_FOR_SENORS:init()
   self.fsm.vars.line_vista = nil
   self.fsm.vars.cluster_vista = nil
end

function WAIT_FOR_SENORS:loop()
   local line = line_in_zone(self.fsm.vars.lines)
   if line then
      local p = llutils.point_in_front(llutils.center(line), 0.8)
      local p_map = tfm.transform6D(
         { x = p.x, y = p.y, z = 0,
           ori = fawkes.tf.create_quaternion_from_yaw(line:bearing()) },
         line:frame_id(), "map")
      printf("p_map: %f, %f; ori = %f", p_map.x, p_map.y, fawkes.tf.get_yaw(p_map.ori))
      if within_map(p_map) then
         self.fsm.vars.line_vista = p_map
      end
   end
   
   local cluster_map = cluster_in_zone(self.fsm.vars.clusters)
   if not self.fsm.vars.line_vista and cluster_map then
      -- No line found, but maybe we can guess something from a cluster...
      -- Compute a point left or right of the cluster from where we're standing
      local phi = local_bearing(self.fsm.vars.x, self.fsm.vars.y)
      local zone_bl = tfm.transform(
         { x = self.fsm.vars.x, y = self.fsm.vars.y, ori = 0 },
         "map", "base_link"
      )
      local x = 0.9 * math.sin(phi)
      local y = 0.9 * math.cos(phi)
      local left_bl = {
         x = zone_bl.x - x,
         y = zone_bl.y + y,
         ori = 0
      }
      left_map = tfm.transform(left_bl, "base_link", "map")
      left_map.ori = math.atan2(
         self.fsm.vars.y - left_map.y,
         self.fsm.vars.x - left_map.x
      )
      if within_map(left_map) then
         self.fsm.vars.cluster_vista = left_map
         print_debug("left_map: " .. left_map.x .. ", " .. left_map.y .. "; ori = " .. left_map.ori)
      else
         local right_bl = {
            x = zone_bl.x - x,
            y = zone_bl.y - y,
            ori = 0
         }
         right_map = tfm.transform(right_bl, "base_link", "map")
         right_map.ori = math.atan2(
            self.fsm.vars.y - right_map.y,
            self.fsm.vars.x - right_map.x
         )
         if within_map(right_map) then
            self.fsm.vars.cluster_vista = right_map
            print_debug("right_map: " .. right_map.x .. ", " .. right_map.y .. "; ori = " .. right_map.ori)
         end
      end
   end
end


function WAIT_FOR_SENORS:exit()
   self.fsm.vars.attempts = self.fsm.vars.attempts + 1
end


function FIND_LINE:init()
   self.fsm.vars.found_something = true
   self.args["goto"] = self.fsm.vars.cluster_vista
end


function GOTO_LINE:init()
   self.fsm.vars.found_something = true
   self.args["goto"] = {
      x = self.fsm.vars.line_vista.x,
      y = self.fsm.vars.line_vista.y,
      ori = fawkes.tf.get_yaw(self.fsm.vars.line_vista.ori)
   }
end


function PICK_VISTA_POINT:init()
   -- Make sure we don't use a zone corner coordinate that's inside a wall or
   -- entirely outside the playing field...
   
   self.fsm.vars.zone_corner = nil

   while not self.fsm.vars.zone_corner and self.fsm.vars.zone_corner_idx <= #ZONE_CORNERS do
      local dx = ZONE_CORNERS[self.fsm.vars.zone_corner_idx].x
      local dy = ZONE_CORNERS[self.fsm.vars.zone_corner_idx].y

      if math.abs(self.fsm.vars.x + dx) > X_MAX then
         dx = 0
      end
      if self.fsm.vars.y + dy > Y_MAX or self.fsm.vars.y + dy < Y_MIN then
         dy = 0
      end
      if math.abs(self.fsm.vars.x + dx) > 4 and self.fsm.vars.y + dy < Y_MIN + 1 then
         dx = 0
         dy = 0
      end

      if dx ~= 0 or dy ~= 0 then
         self.fsm.vars.zone_corner = {
            x = self.fsm.vars.x + dx,
            y = self.fsm.vars.y + dy,
            ori = math.atan2(-dy, -dx)
         }
      end
      
      self.fsm.vars.zone_corner_idx = self.fsm.vars.zone_corner_idx + 1
   end
end


function GOTO_VISTA_POINT:init()
   self.args["goto"] = self.fsm.vars.zone_corner
end

function FAILED:init()
   if not self.fsm.vars.found_something then
      zone_info:set_search_state(zone_info.NO)
   else
      zone_info:set_search_state(zone_info.UNKNOWN)
   end
end

function WAIT_AMCL:loop()
   found_tag()
end
