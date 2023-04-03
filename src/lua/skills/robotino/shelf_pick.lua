
----------------------------------------------------------------------------
--  shelf_pick.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2015  Tobias Neumann
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
name               = "shelf_pick"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move", "gripper_commands", "reset_gripper"}
depends_interfaces = {
   {v = "line1_avg", type="LaserLineInterface", id="/laser-lines/1/moving_avg"},
   {v = "line2_avg", type="LaserLineInterface", id="/laser-lines/2/moving_avg"},
   {v = "line3_avg", type="LaserLineInterface", id="/laser-lines/3/moving_avg"},
   {v = "line4_avg", type="LaserLineInterface", id="/laser-lines/4/moving_avg"},
   {v = "line5_avg", type="LaserLineInterface", id="/laser-lines/5/moving_avg"},
   {v = "line6_avg", type="LaserLineInterface", id="/laser-lines/6/moving_avg"},
   {v = "line7_avg", type="LaserLineInterface", id="/laser-lines/7/moving_avg"},
   {v = "line8_avg", type="LaserLineInterface", id="/laser-lines/8/moving_avg"},
   {v = "robotino_sensor", type="RobotinoSensorInterface", id="Robotino"},
}

documentation      = [==[ shelf_pick
                          This skill does:
                          - Picks of Shelf
                          @param slot       string  the slot to pick the puck of; options ( LEFT | MIDDLE | RIGHT )
]==]

-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")
local llutils = require("fawkes.laser-lines_utils")

local x_distance = 0.28
local gripper_adjust_z_distance = 0.0
local gripper_adjust_x_distance = 0.015
local adjust_target_frame = "gripper_home"

local shelf_to_conveyor = 0.1
local shelf_distance = 0.1

local MIN_VIS_HIST_LINE=5
local LINE_LENGTH_MIN=0.64
local LINE_LENGTH_MAX=0.71
local MAX_ORI=30 -- in degrees

-- function to find the closest LL's direction 
-- which fulfills the following criteria:
-- visibility history over MIN_VIS_HIST_LINE
-- minimum distance to bot
-- angle from bot's forward direction to laserline's center smaller than MAX_ORI
--
-- the function returns the direction of this LL
-- if no LL is found which fulfills all the above criterias, the ideal direction is returned.
function find_ll_direction(lines)
  local closest_ll = -1
  local min_distance = math.huge
  for line_id,line in pairs(lines) do
    local center = llutils.center(line)
    local distance = math.sqrt(math.pow(center.x,2),math.pow(center.y,2))
    local ori = math.deg(math.atan2(center.y, center.x))
    if math.abs(ori) < MAX_ORI then -- approximately in front of us
      if line:visibility_history() >= MIN_VIS_HIST_LINE then -- this laser line is not stale
        if distance < min_distance then
          closest_ll = line:id()
          min_distance = distance
        end
      end
    end
  end

  -- determination of laser line is done

  if closest_ll == -1 then
    return {x=0.0,y=-1.0}
  else
    direction = {x=lines[closest_ll]:line_direction(0),y=lines[closest_ll]:line_direction(1)}
    length_direction = math.sqrt(math.pow(direction.x,2)+math.pow(direction.y,2))
    return {x=direction.x/length_direction,y=direction.y/length_direction}
  end
end

-- function to evaluate sensor data
function is_grabbed()
   return true
-- if not robotino_sensor:has_writer() then
--   print_warn("No robotino sensor writer to check sensor")
--   return true
-- end
-- if robotino_sensor:is_digital_in(0) == false and robotino_sensor:is_digital_in(1) == true then -- white cable on DI1 and black on DI2
--    return true
-- else
--   return false
-- end
end


function pose_gripper_offset(x,y,z)
  local target_pos = { x = x,
                        y = y,
                        z = z,
                        ori = { x = 0, y = 0, z = 0, w = 0}

   }
   local tmp = { x = 0,
                 y = 0,
                 z = 0,
                 ori = { x = 0, y = 0, z = 0, w = 0}
   }

   -- Get offset from gripper axis (middle of z sledge) to gripper finger
   local gripper_rel = tfm.transform6D(tmp,"gripper","gripper_z_dyn")

   -- Shift target point to gripper axis frame
   gripper_rel.x = target_pos.x - gripper_rel.x
   gripper_rel.y = target_pos.y - gripper_rel.y
   gripper_rel.z = target_pos.z - gripper_rel.z

   -- Transform target to gripper home frame = absolut coordinates of the axis
   local gripper_home_rel = tfm.transform6D(gripper_rel,"gripper","gripper_home")

   -- Clip to axis limits
   return { x = math.max(0,math.min(gripper_home_rel.x,fsm.vars.x_max)),
            y = math.max(-fsm.vars.y_max/2,math.min(gripper_home_rel.y,fsm.vars.y_max/2)),
            z = math.max(0,math.min(gripper_home_rel.z,fsm.vars.z_max))}
end


fsm:define_states{ export_to=_M, closure={is_grabbed=is_grabbed},
   {"INIT", SkillJumpState, skills={{gripper_commands}}, final_to="GOTO_SHELF", fail_to="FAILED" },
   {"GOTO_SHELF", SkillJumpState, skills={{motor_move}}, final_to="MOVE_ABOVE_PUCK", fail_to="FAILED"},
   {"MOVE_ABOVE_PUCK", SkillJumpState, skills={{gripper_commands}}, final_to="GRAB_PRODUCT", fail_to="FAILED" },
   {"GRAB_PRODUCT", SkillJumpState, skills={{gripper_commands}}, final_to="LEAVE_SHELF", fail_to="FAILED"},
   {"LEAVE_SHELF", SkillJumpState, skills={{motor_move}}, final_to="RESET_GRIPPER", fail_to="FAILED"},
   {"RESET_GRIPPER", SkillJumpState, skills={{reset_gripper}}, final_to="CHECK_PUCK", fail_to="FAILED"},
   {"CHECK_PUCK", JumpState},
   {"WAIT_AFTER_GRAB", JumpState},
}

fsm:add_transitions{
   {"GOTO_SHELF", "FAILED", cond="vars.error"},
   {"CHECK_PUCK", "FAILED", cond="not is_grabbed()", desc="Not holding puck"},
   {"CHECK_PUCK", "FINAL", cond=true},
}

function INIT:init()
   self.args["gripper_commands"].command = "OPEN"

   -- Override values if host specific config value is set
   if config:exists("/skills/shelf_pick/gripper_adjust_z_distance") then
       gripper_adjust_z_distance = config:get_float("/skills/shelf_pick/gripper_adjust_z_distance")
   end

   if config:exists("/arduino/x_max") then
       self.fsm.vars.x_max = config:get_float("/arduino/x_max")
   else
       self.fsm.vars.x_max = 0.114
   end
   if config:exists("/arduino/y_max") then
       self.fsm.vars.y_max = config:get_float("/arduino/y_max")
   else
       self.fsm.vars.y_max = 0.037
   end
   if config:exists("/arduino/max_z") then
       self.fsm.vars.z_max = config:get_float("/arduino/z_max")
   else
       self.fsm.vars.z_max = 0.057
   end


   self.fsm.vars.left_slot_y_offset = config:get_float("/skills/shelf_pick/left_slot_y_offset")
   self.fsm.vars.middle_slot_y_offset = config:get_float("/skills/shelf_pick/middle_slot_y_offset")
   self.fsm.vars.right_slot_y_offset = config:get_float("/skills/shelf_pick/right_slot_y_offset")

   self.fsm.vars.lines_avg = {}
   self.fsm.vars.lines_avg[line1_avg:id()] = line1_avg
   self.fsm.vars.lines_avg[line2_avg:id()] = line2_avg
   self.fsm.vars.lines_avg[line3_avg:id()] = line3_avg
   self.fsm.vars.lines_avg[line4_avg:id()] = line4_avg
   self.fsm.vars.lines_avg[line5_avg:id()] = line5_avg
   self.fsm.vars.lines_avg[line6_avg:id()] = line6_avg
   self.fsm.vars.lines_avg[line7_avg:id()] = line7_avg
   self.fsm.vars.lines_avg[line8_avg:id()] = line8_avg

end

function GOTO_SHELF:init()
   if self.fsm.vars.slot == "LEFT" then
      dist_y = shelf_to_conveyor
      dist_y = dist_y + self.fsm.vars.left_slot_y_offset
   elseif self.fsm.vars.slot == "MIDDLE" then
      dist_y = shelf_to_conveyor + shelf_distance
      dist_y = dist_y + self.fsm.vars.middle_slot_y_offset
   elseif self.fsm.vars.slot == "RIGHT" then
      dist_y = shelf_to_conveyor + 2*shelf_distance
      dist_y = dist_y + self.fsm.vars.right_slot_y_offset
   else
      dist_y = 0
      self.fsm:set_error("no shelf side set")
      self.fsm.vars.error = true
   end
   

   local target_pos_cp = { x = gripper_adjust_x_distance,
                           y = 0.0,
                           z = 0.0,
                           ori = { x=0,y=0,z=0,w=0}
                         }

   local target_pos_odom = tfm.transform6D(target_pos_cp, "conveyor_pose", "odom")
   local target_pos_bl = tfm.transform6D(target_pos_odom, "odom", "base_link")

   laserline_direction = find_ll_direction(self.fsm.vars.lines_avg)
   target_pos_bl.x = target_pos_bl.x + dist_y * laserline_direction.x
   target_pos_bl.y = target_pos_bl.y + dist_y * laserline_direction.y

   self.fsm.vars.target_pos_odom = tfm.transform6D(target_pos_bl, "base_link", "odom")
   
  self.args["motor_move"] =
	{ y = dist_y * laserline_direction.y, 
    x = dist_y * laserline_direction.x,
		tolerance = { x=0.002, y=0.002, ori=0.01 }
	}
end

function MOVE_ABOVE_PUCK:init()
  local grip_pos = tfm.transform6D(self.fsm.vars.target_pos_odom, "odom", "gripper")

  local pose = pose_gripper_offset(grip_pos.x,grip_pos.y,grip_pos.z)
  self.args["gripper_commands"].x = pose.x
  self.args["gripper_commands"].y = pose.y
  self.args["gripper_commands"].z = gripper_adjust_z_distance
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].target_frame = "gripper_home"

end

function GRAB_PRODUCT:init()
   self.args["gripper_commands"].command = "CLOSE"
end

function LEAVE_SHELF:init()
   self.args["motor_move"].x = -0.2
end

