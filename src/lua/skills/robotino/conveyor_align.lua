----------------------------------------------------------------------------
--  conveyor_align.lua - align orthogonal to the conveyor using the conveyor vision
--
--  Copyright  2013 The Carologistics Team
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
name               = "conveyor_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"motor_move","approach_mps", "ax12gripper"}
depends_interfaces = { 
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "conveyor_0", type = "Position3DInterface"},
   {v = "gripper_if", type = "AX12GripperInterface", id="Gripper AX12"},
}

documentation      = [==[aligns the robot orthogonal to the conveyor by using the
conveyor vision
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

local TOLERANCE_Y = 0.005
local TOLERANCE_Z = 0.002
local MAX_TRIES = 4
local Z_DEST_POS_WITH_PUCK = 0.002
local Z_DEST_POS_WITHOUT_PUCK = 0.008
local Z_DEST_POS = Z_DEST_POS_WITH_PUCK
local Z_DIVISOR = 2

function no_writer()
   return not conveyor_0:has_writer()
end

function tolerance_y_not_ok()
   return not (math.abs(conveyor_0:translation(1)) <= TOLERANCE_Y)
end

function tolerance_z_not_ok()
   return not (math.abs(conveyor_0:translation(2) - Z_DEST_POS) <= TOLERANCE_Z)
end

fsm:define_states{ export_to=_M,
   closure={MAX_TRIES=MAX_TRIES, Z_DEST_POS=Z_DEST_POS},
   {"INIT", JumpState},
   {"APPROACH_MPS_FAR", SkillJumpState, skills={{approach_mps}}, final_to="ROUGH_ALIGN", fail_to="FAILED"},
   {"ROUGH_ALIGN", SkillJumpState, skills={{motor_move}, {ax12gripper}}, final_to="APPROACH_MPS_NEAR", fail_to="FAILED"},
   {"APPROACH_MPS_NEAR", SkillJumpState, skills={{approach_mps}}, final_to="DRIVE_YZ", fail_to="FAILED"},
   {"DRIVE_YZ", SkillJumpState, skills={{motor_move}, {ax12gripper}}, final_to="SETTLE", fail_to="FAILED"},
   {"DRIVE_Z", SkillJumpState, skills={{ax12gripper}}, final_to="SETTLE_Z", fail_to="FAILED"},
   {"SETTLE", JumpState},
   {"SETTLE_Z", JumpState},
   {"CHECK", JumpState},
   {"CHECK_Z", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond=no_writer, desc="no conveyor vision"},
   {"INIT", "APPROACH_MPS_FAR", cond=true},
   {"SETTLE", "CHECK", timeout=0.5},
   {"SETTLE_Z", "CHECK_Z", timeout=0.5},
   {"CHECK", "FAILED", cond="vars.counter > MAX_TRIES", desc="max tries reached"},
   {"CHECK", "DRIVE_YZ", cond=tolerance_y_not_ok, desc="tolerance not ok, align y distance"},
   {"CHECK", "SETTLE_Z", cond=true},
   {"CHECK_Z", "DRIVE_Z", cond=tolerance_z_not_ok, desc="tolerance not ok, align z distance"},
   {"CHECK_Z", "FINAL", cond=true},
}

function INIT:init()
   self.fsm.vars.counter = 1
end

function APPROACH_MPS_FAR:init()
   self.skills[1].x = 0.2
end

function ROUGH_ALIGN:init()
   if gripper_if:is_holds_puck() then
      Z_DEST_POS = Z_DEST_POS_WITH_PUCK
   else
      Z_DEST_POS = Z_DEST_POS_WITHOUT_PUCK
   end
   self.skills[1].y = conveyor_0:translation(1)
   self.skills[1].tolerance = { x=0.002, y=0.002, ori=0.01 }
   self.skills[2].command = "RELGOTOZ"
   if tolerance_z_not_ok() then
      self.skills[2].z_position = ((conveyor_0:translation(2) - Z_DEST_POS) * 1000) / Z_DIVISOR
   end
end

function APPROACH_MPS_NEAR:init()
   self.skills[1].x = 0.15
end

function DRIVE_YZ:init()
   self.skills[1].y = conveyor_0:translation(1)
   self.skills[1].tolerance = { x=0.002, y=0.002, ori=0.01 }
   self.skills[2].command = "RELGOTOZ"
   if tolerance_z_not_ok() then
      self.skills[2].z_position = ((conveyor_0:translation(2) - Z_DEST_POS) * 1000) / Z_DIVISOR
   end
end

function DRIVE_Z:init()
   self.skills[1].command = "RELGOTOZ"
   if conveyor_0:translation(2) - Z_DEST_POS < 0 then
      self.skills[1].z_position = -1
   else
      self.skills[1].z_position = 1
   end
end

function CHECK:init()
   self.fsm.vars.counter = self.fsm.vars.counter + 1
end
