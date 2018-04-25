
----------------------------------------------------------------------------
--  shelf_pick_new.lua
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
name               = "shelf_pick_new"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move","gripper_commands_new"}
depends_interfaces = {

}

documentation  = [==[The robot needs to be aligned with the conveyor and then grips product from shelf.
        @param slot       string  the slot to pick the puck of; options ( LEFT | MIDDLE | RIGHT )
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Tunables
local shelf_to_conveyor = 0.1
local shelf_distance = 0.1
local drive_back_distance = -0.2
local gripper_x = 0
local gripper_y = 0
local gripper_z = 0

fsm:define_states{ export_to=_M, closure={gripper_if=gripper_if},
   {"INIT", JumpState},
   {"GOTO_SHELF", SkillJumpState, skills={{motor_move}}, final_to="OPEN_GRIPPER", fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER"},
   {"MOVE_GRIPPER_XY", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_XY", fail_to="FAILED"},
   {"MOVE_GRIPPER_Z", SkillJumpState, skills={{gripper_commands_new}}, final_to="CLOSE_GRIPPER", fail_to="FAILED"},
   {"CLOSE_GRIPPER", SkillJumpState, skills={{gripper_commands_new}}, final_to="MOVE_GRIPPER_BACK", fail_to="FAILED"},
   {"MOVE_GRIPPER_BACK", SkillJumpState, skills={{gripper_commands_new}}, final_to="DRIVE_BACK", fail_to="FAILED"},
   {"DRIVE_BACK", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
  {"INIT", "GOTO_SHELF", cond=true , desc="Drive to shelf from conveyor"},
}

function GOTO_SHELF:init()
   shelf_to_conveyor = 0.1 --TODO measure both values
   shelf_distance = 0.1
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

function OPEN_GRIPPER:init()
  self.args["gripper_commands_new"].command = "OPEN"
end

function MOVE_GRIPPER_XY:init()
  self.args["gripper_commands_new"].x = gripper_x
  self.args["gripper_commands_new"].y = gripper_y
  self.args["gripper_commands_new"].command = "MOVEABS"
end

function MOVE_GRIPPER_Z:init()
  self.args["gripper_commands_new"].z = gripper_z
  self.args["gripper_commands_new"].command = "MOVEABS"
end



function CLOSE_GRIPPER:init()
  self.args["gripper_commands_new"].command = "CLOSE"
end

function MOVE_GRIPPER_BACK:init()
  self.args["gripper_commands_new"].z = gripper_back_z
  self.args["gripper_commands_new"].command = "MOVEABS"
end

function DRIVE_BACK:init()
   self.args["motor_move"].x = drive_back_distance
end
