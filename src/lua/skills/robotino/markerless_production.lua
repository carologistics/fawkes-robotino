----------------------------------------------------------------------------
--  markerless_production.lua
--
--  Created Wed Apr 15
--  Copyright  2019  Morian Sonnet
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
name               = "markerless_production"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"gripper_commands", "motor_move", "reset_gripper"}
depends_interfaces = {
   {v = "robotino_sensor", type = "RobotinoSensorInterface", id="Robotino"} -- Interface to read I/O ports
   {v = "realsense_switch", type = "SwitchInterface", id="realsense2"},
   {v = "laserline_switch", type = "SwitchInterface", id="laser-lines"},
}

documentation      = [==[
Skill to do markerless production

Parameters:
]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")
local llutils = require("fawkes.laser-lines_utils")

local MAX_ORI = 80


-- function to find ll in front
function find_ll(lines, prealigned)
  if prealigned == nil then
    prealigned = false
  end
  local closest_ll = nil
  local min_distance = math.huge
  for line_id,line in pairs(lines) do
    local center = llutils.center(line)
    local distance = math.sqrt(math.pow(center.x,2),math.pow(center.y,2))
    local ori = math.deg(math.atan2(center.y, center.x))
    if prealigned then
       ori = ori + math.deg(math.atan2(fsm.vars.y, fsm.vars.x))
    elseif fsm.vars.preori ~= nil then
       ori = ori - fsm.vars.preori
    end
    
    if math.abs(ori) < MAX_ORI then -- approximately in front of us
      if line:visibility_history() >= MIN_VIS_HIST_LINE then -- this laser line is not stale
        if distance < min_distance then
          if line:length() > LINE_LENGTH_MIN and line:length() < LINE_LENGTH_MAX then
            closest_ll = line
            min_distance = distance
          end
        end
      end
    end
  end
  return closest_ll
end

-- function to evaluate sensor data
function is_grabbed()
 if not robotino_sensor:has_writer() then
   print_warn("No robotino sensor writer to check sensor")
   return true
 end
 if robotino_sensor:is_digital_in(0) == false and robotino_sensor:is_digital_in(1) == true then -- white cable on DI1 and black on DI2
    return true
 else
   -- Ignore puck laser, until realsense is disabled by default 
   return true
 end
end


function pose_not_exist()
  local target_pos = { x = gripper_pose_offset_x,
                       y = gripper_pose_offset_y,
                       z = gripper_pose_offset_z,
                       ori = { x=0, y = 0, z= 0, w= 0}

   }

   local transformed_pos = tfm.transform6D(target_pos, "conveyor_pose", "gripper")
   if transformed_pos == nil then
     return true
   end
   return false
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


fsm:define_states{ export_to=_M,
   closure={pose_not_exist=pose_not_exist, is_grabbed=is_grabbed},
   {"INIT", JumpState},
   {"MPS_ALIGN", SkillJumpState, skills={{markerless_mps_align}}, final_to="CHECK_SIDE", fail_to="FAILED"},
   {"CHECK_SIDE", JumpState},
   {"TEST_INPUT", JumpState},
   {"TEST_OUTPUT", JumpState},
   {"DECIDE_SIDE", JumpState},
   {"SWITCH_SIDE_BEFORE", SkillJumpState, skills={{goto}}, final_to="MPS_ALIGN_OTHER_SIDE", fail_to="FAILED"},
   {"MPS_ALIGN_OTHER_SIDE", SkillJumpState, skills={{markerless_mps_align}}, final_to="CONVEYOR_ALIGN", fail_to="FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="SHELF_PICK", fail_to="FAILED"},
   {"SHELF_PICK", SkillJumpState, skills={{markerless_shelf_pick}}, final_to="MPS_ALIGN_BEFORE_PICK", fail_to="FAILED"},
   {"MPS_ALIGN_BEFORE_PICK", SkillJumpState, skills={{markerless_mps_align}}, final_to="PUT_PRODUCT", fail_to="FAILED"},
   {"CONVEYOR_ALIGN_PUT", SkillJumpState, skills={{conveyor_align}}, final_to="SHELF_PICK", fail_to="FAILED"},
   {"PUT_PRODUCT", SkillJumpState, skills={{product_put}}, final_to="SWITCH_SIDE", fail_to="FAILED"},
   {"SWITCH_SIDE", SkillJumpState, skills={{goto}}, final_to="PICK_PRODUCT", fail_to="FAILED"},
   {"CONVEYOR_ALIGN_PICK", SkillJumpState, skills={{conveyor_align}}, final_to="SHELF_PICK", fail_to="FAILED"},
   {"PICK_PRODUCT", SkillJumpState, skills={{product_pick}}, final_to="MOVE_BACK", fail_to="FAILED"},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="MOVE_BACK"},
}

fsm:add_transitions{
  {"INIT", "MPS_ALIGN", cond=true},
  {"CHECK_SIDE", "SWITCH_SIDE_BEFORE", cond="vars.input ~= nil and vars.input==true"},
  {"CHECK_SIDE", "SHELF_PICK", cond="vars.input ~= nil and vars.input==false"},
  {"CHECK_SIDE", "FAILED", cond="vars.error"},
  {"DECIDE_SIDE", "SWITCH_SIDE_BEFORE", cond="vars.fitness_input < vars.fitness_output"},
  {"DECIDE_SIDE", "CONVEYOR_ALIGN", cond=true}
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
end

function MPS_ALIGN:init()
  self.args["markerless_mps_align"].x=0.3
end


function MPS_ALIGN_OTHER_SIDE:init()
  self.args["markerless_mps_align"].x=0.3
  self.args["markerless_mps_align"].y= + 0.03
end

function MPS_ALIGN_BEFORE_PICK:init()
  self.args["markerless_mps_align"].x=0.3
  local laserline = find_ll(self.fsm.vars.lines)
  local center_ll = llutils.center(laserline)
  self.args["markerless_mps_align"].preori = math.atan2(center.y, center.x)
  self.args["markerless_mps_align"].y= - 0.03
end

function enable_conveyor_plane(enable)
   if if_plane_switch:has_writer() then
      local msg
      if enable then
         msg = if_plane_switch.EnableSwitchMessage:new()
      else
         msg = if_plane_switch.DisableSwitchMessage:new()
      end
      if_plane_switch:msgq_enqueue_copy(msg)
   end
end

function CHECK_SIDE:init()
   laserline_switch:msgq_enqueue(laserline_switch.EnableSwitchMessage:new())
   realsense_switch:msgq_enqueue(realsense_switch.EnableSwitchMessage:new())
   enable_conveyor_plane(true)

end

function TEST_INPUT:init()
  local mps_type = if_conveyor_pose.CAP_STATION
  local target = if_conveyor_pose.INPUT_CONVEYOR
  local msg = if_conveyor_pose.RunICPMessage:new(self.fsm.vars.mps_type, self.fsm.vars.mps_target)
  if_conveyor_pose:msgq_enqueue_copy(msg)
  self.fsm.vars.msgid = msg:id()
  self.fsm.vars.vision_retries = self.fsm.vars.vision_retries + 1
end

function TEST_INPUT:exit()
   local msg = if_conveyor_pose.StopICPMessage:new()
   if_conveyor_pose:msgq_enqueue_copy(msg)
end

function TEST_OUTPUT:init()
  local mps_type = if_conveyor_pose.CAP_STATION
  local target = if_conveyor_pose.OUTPUT_CONVEYOR
  local msg = if_conveyor_pose.RunICPMessage:new(self.fsm.vars.mps_type, self.fsm.vars.mps_target)
  if_conveyor_pose:msgq_enqueue_copy(msg)
  self.fsm.vars.msgid = msg:id()
  self.fsm.vars.vision_retries = self.fsm.vars.vision_retries + 1
end

function TEST_OUTPUT:exit()
   local msg = if_conveyor_pose.StopICPMessage:new()
   if_conveyor_pose:msgq_enqueue_copy(msg)
   enable_conveyor_plane(false)
end


function SWITCH_SIDE_BEFORE:init()
  local laserline = find_ll(self.fsm.vars.lines)
  local center_ll = llutils.center(laserline)
  local pos_to_go_ll = llutils.point_in_front(center_ll,-0.9)
  local target_pos = tfutils.transform(pos_to_go_ll, "base_laser", "base_link")
  self.args["relgoto"].rel_x = target_pos.x
  self.args["relgoto"].rel_y = target_pos.y
  self.args["relgoto"].ori=math.normalize_mirror_rad(target_pos.ori)
end

function SWITCH_SIDE:init()
  local laserline = find_ll(self.fsm.vars.lines)
  local center_ll = llutils.center(laserline)
  local pos_to_go_ll = llutils.point_in_front(center_ll,-0.9)
  local target_pos = tfutils.transform(pos_to_go_ll, "base_laser", "base_link")
  self.args["relgoto"].rel_x = target_pos.x
  self.args["relgoto"].rel_y = target_pos.y
  self.args["relgoto"].ori=math.normalize_mirror_rad(target_pos.ori)
end

function SHELF_PICK:init()
end


function CONVEYOR_ALIGN:init()
  self.args["conveyor_align"].disable_realsense_afterwards=true
  self.args["conveyor_align"].place="M-CS2"
  self.args["conveyor_align"].side="input"
end

function CONVEYOR_ALIGN_PUT:init()
  self.args["conveyor_align"].disable_realsense_afterwards=true
  self.args["conveyor_align"].place="M-CS2"
  self.args["conveyor_align"].side="input"
end

function CONVEYOR_ALIGN_PICK:init()
  self.args["conveyor_align"].disable_realsense_afterwards=true
  self.args["conveyor_align"].place="M-CS2"
  self.args["conveyor_align"].side="output"
end



