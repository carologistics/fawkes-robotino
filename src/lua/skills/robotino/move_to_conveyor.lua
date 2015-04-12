
----------------------------------------------------------------------------
--  move_to_conveyor.lua
--
--  Created Sat Apr 11
--  Copyright  2015  Johannes Rothe
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
name               = "move_to_conveyor"
fsm                = SkillHSM:new{name=name, start="SKILL_ALIGN_TAG", debug=true}
depends_skills     = {"motor_move", "align_tag", "ax12gripper"}
depends_interfaces = {}

documentation      = [==[Align the robot and then moves to the conveyor
@param pick_puck True if you want to pick a puck from the conveyor
@param put_puck  True if you want to put a puck on the conveyor
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

local TAG_OFFSET_Y = 0.025 --TODO if the tags are misaligned you there should be an parametric offset or another solution
local ALIGN_DISTANCE = 0.35

fsm:define_states{ export_to=_M, closure={},
   {"SKILL_ALIGN_TAG", SkillJumpState, skills={{align_tag}},
      final_to="DECIDE_OPEN", fail_to="FAILED"},
   {"DECIDE_OPEN", JumpState},
   --TODO align by laserlines and (if implemented) the conveyor detection
   {"OPEN_GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="DRIVE_FORWARD", fail_to="FAILED"},
   {"DRIVE_FORWARD", SkillJumpState, skills={{motor_move}},
      final_to="GRIPPER", fail_to="FAILED"},
   {"GRIPPER", SkillJumpState, skills={{ax12gripper}},
      final_to="MOVE_BACK", fail_to="FAILED"},
   {"MOVE_BACK", SkillJumpState, skills={{motor_move}},
      final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"DECIDE_OPEN", "DRIVE_FORWARD", cond="fsm.vars.put_puck", desc="Has a puck in front"},
   {"DECIDE_OPEN", "OPEN_GRIPPER", cond="fsm.vars.pick_puck", desc="Open gripper before driving"},
}

function SKILL_ALIGN_TAG:init()
   self.skills[1].x = ALIGN_DISTANCE
   self.skills[1].y = TAG_OFFSET_Y
   self.skills[1].ori = 0
end

function DRIVE_FORWARD:init()
   self.skills[1].x = 0.16
end

function OPEN_GRIPPER:init()
   self.skills[1].open = true
   printf("open gripper")
end

function GRIPPER:init()
   if self.fsm.vars.pick_puck then
      self.skills[1].open = false
      self.skills[1].close = true
      printf("pick the puck")
   elseif self.fsm.vars.put_puck then
      self.skills[1].open = true
      self.skills[1].close = false
      printf("put the puck")
   end
end

function MOVE_BACK:init()
   self.skills[1].x = -0.16
end
