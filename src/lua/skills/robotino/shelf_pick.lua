
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
depends_skills     = {"motor_move", "gripper_commands", "approach_mps"}
depends_interfaces = {
}

documentation      = [==[ shelf_pick
                          This skill does:
                          - Picks of Shelf
                          @param slot       string  the slot to pick the puck of; options ( LEFT | MIDDLE | RIGHT )
]==]

-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

local x_distance = 0.27
local gripper_adjust_z_distance = 0.025
local gripper_adjust_x_distance = 0.015
local adjust_target_frame = "gripper_home"
local gripper_down_to_puck = -0.035

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


fsm:define_states{ export_to=_M, closure={},
   {"INIT", SkillJumpState, skills={{gripper_commands}}, final_to="GOTO_SHELF", fail_to="FAILED" },
   {"GOTO_SHELF", SkillJumpState, skills={{motor_move}}, final_to="MOVE_ABOVE_PUCK", fail_to="FAILED"},
   {"MOVE_ABOVE_PUCK", SkillJumpState, skills={{gripper_commands}}, final_to="ADJUST_HEIGHT", fail_to="FAILED" },
   {"ADJUST_HEIGHT", SkillJumpState, skills={{gripper_commands}}, final_to="APPROACH_SHELF", fail_to="FAILED" },
   {"APPROACH_SHELF", SkillJumpState, skills={{approach_mps}}, final_to="GRAB_PRODUCT", fail_to="FAILED"},
   {"GRAB_PRODUCT", SkillJumpState, skills={{gripper_commands}}, final_to="LEAVE_SHELF", fail_to="FAILED"},
   {"LEAVE_SHELF", SkillJumpState, skills={{motor_move}}, final_to="HOME_GRIPPER", fail_to="FAILED"},
   {"HOME_GRIPPER", SkillJumpState, skills={{gripper_commands}}, final_to="FINAL", fail_to="FAILED"},
   {"WAIT_AFTER_GRAB", JumpState},
}

fsm:add_transitions{
   {"GOTO_SHELF", "FAILED", cond="vars.error"},
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
end

function GOTO_SHELF:init()
   local shelf_to_conveyor = 0.1 --TODO measure both values
   local shelf_distance = 0.1
   if self.fsm.vars.slot == "LEFT" then
      dest_y = shelf_to_conveyor
   elseif self.fsm.vars.slot == "MIDDLE" then
      dest_y = shelf_to_conveyor + shelf_distance
   elseif self.fsm.vars.slot == "RIGHT" then
      dest_y = shelf_to_conveyor + 2*shelf_distance
   else
      dest_y = 0
      self.fsm:set_error("no shelf side set")
      self.fsm.vars.error = true
   end
   
   self.args["motor_move"] =
			{ y = -dest_y, --shelf is on the right side of the conveyor
				tolerance = { x=0.002, y=0.002, ori=0.01 }
			}
end

function MOVE_ABOVE_PUCK:init()
   local target_pos = { x = gripper_adjust_x_distance,
                       y = 0,
                       z = gripper_adjust_z_distance,
                       ori = { x = 0, y = 0, z = 0, w = 0}
   }

  local grip_pos = tfm.transform6D(target_pos, "conveyor_pose", "gripper")

  local pose = pose_gripper_offset(grip_pos.x,grip_pos.y,grip_pos.z)
  self.args["gripper_commands"] = pose
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].target_frame = "gripper_home"

end

function ADJUST_HEIGHT:init()

  local pose = { x = 0, y = 0, z = gripper_down_to_puck }
  self.args["gripper_commands"] = pose
  self.args["gripper_commands"].command = "MOVEREL"
  self.args["gripper_commands"].target_frame = "gripper_home"

end

function APPROACH_SHELF:init()
   self.args["approach_mps"].x = x_distance
   self.args["approach_mps"].use_conveyor = false
end

function GRAB_PRODUCT:init()
   self.args["gripper_commands"].command = "CLOSE"
end

function LEAVE_SHELF:init()
   self.args["motor_move"].x = -0.2
end

function HOME_GRIPPER:init()
  self.args["gripper_commands"].x = 0
  self.args["gripper_commands"].y = 0
  self.args["gripper_commands"].z = 0
  self.args["gripper_commands"].target_frame = "gripper_home"
  self.args["gripper_commands"].command = "MOVEABS"
  self.args["gripper_commands"].wait = false
end
