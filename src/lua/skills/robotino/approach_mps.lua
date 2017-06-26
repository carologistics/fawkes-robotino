----------------------------------------------------------------------------
--  approach_mps.lua
--
--  Created Sun Jun 26 11:44:40 2016
--  Copyright  2016  Frederik Zwilling
--             2017  Tobias Neumann
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
name               = "approach_mps"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = {
  {v = "if_conveyor", type = "Position3DInterface", id="conveyor_pose/pose"},
  {v = "conveyor_switch", type = "SwitchInterface", id="conveyor_pose/switch"},
  {v = "conveyor_config", type = "ConveyorConfigInterface", id="conveyor_pose/config"},
}

documentation      = [==[
                        The robot just drives forward according to the current distance to the mps and the desired position
                        @param "x" int The x distance of the conveyor in the base_link frame when finished
                     ]==]


-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")
local cfg_frame_ = "gripper"

function transformed_pose(self)
  local from = { x = if_conveyor:translation(0),
                 y = if_conveyor:translation(1),
                 z = if_conveyor:translation(2),
                 ori = { x = if_conveyor:rotation(0),
                         y = if_conveyor:rotation(1),
                         z = if_conveyor:rotation(2),
                         w = if_conveyor:rotation(3),
                       }
               }
  local cp = tfm.transform6D(from, if_conveyor:frame(), cfg_frame_)

  -- TODO check nil

  local ori = fawkes.tf.get_yaw( fawkes.tf.Quaternion:new(cp.ori.x, cp.ori.y, cp.ori.z, cp.ori.w))
  
  return { x = cp.x,
           y = cp.y,
           z = cp.z,
           ori = ori 
         }
end

function mps_is_near(self)
  self.fsm.vars.pose = transformed_pose()
  if if_conveyor:visibility_history() > 5 and self.fsm.vars.pose.x < 0.5 then
    return true
  else
    printf("mps_approach failed: visibility history is %f, dist to object in front is %f. I don't drive with this visibility history or this far without collision avoidance", if_conveyor:visibility_history(), self.fsm.vars.pose.x)
    return false
  end
end


fsm:define_states{ export_to=_M, closure={mps_is_near=mps_is_near},
   {"INIT", JumpState},
   {"APPROACH", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "APPROACH", cond=mps_is_near},
   {"INIT", "FAILED", timeout=1.0}
}

function INIT:init()
  conveyor_switch:msgq_enqueue_copy(conveyor_switch.EnableSwitchMessage:new())
end

function APPROACH:init()
  local x_goal = self.fsm.vars.pose.x - self.fsm.vars.x
  printf("distance is: %f => drive to: %f", self.fsm.vars.pose.x, x_goal)
  self.args["motor_move"] = {x = self.fsm.vars.x, vel_trans = 0.1, frame="conveyor"}
end

function cleanup()
  conveyor_switch:msgq_enqueue_copy(conveyor_switch.DisableSwitchMessage:new())
end

function FINAL:init()
  cleanup()
end

function FAILED:init()
  cleanup()
  self.fsm.vars.pose = transformed_pose()
  printf("mps_approach failed (maybe because of subskill): visibility history is %f, dist to object in front is %f.", if_conveyor:visibility_history(), self.fsm.vars.pose.x)
end
