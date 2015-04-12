
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
depends_interfaces = {
 {v = "line1", type="LaserLineInterface", id="/laser-lines/1"}
}

documentation      = [==[Align the robot and then moves to the conveyor
@param pick_puck True if you want to pick a puck from the conveyor
@param put_puck  True if you want to put a puck on the conveyor
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("tf_module")

local TAG_OFFSET_Y = 0.01 --TODO if the tags are misaligned you there should be an parametric offset or another solution
local ALIGN_DISTANCE = 0.4

function see_line()
   printf("vis_hist: %f", line1:visibility_history())
   return line1:visibility_history() > 5
end

fsm:define_states{ export_to=_M, closure={see_line = see_line},
   {"SKILL_ALIGN_TAG", SkillJumpState, skills={{align_tag}},
      final_to="SEE_LINE", fail_to="FAILED"},
   {"SEE_LINE", JumpState},
   {"ALIGN_WITH_LASERLINES", SkillJumpState, skills={{motor_move}},
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
   {"SEE_LINE", "ALIGN_WITH_LASERLINES", cond=see_line, desc="Seeing a line"},
   {"SEE_LINE", "DECIDE_OPEN", timeout=1, desc="Not seeing a line, continue just aligned by tag"}
}

function SKILL_ALIGN_TAG:init()
   -- align by ALIGN_DISTANCE from tag to base_link with align_tag
   local tag_transformed = tfm.transform({x=ALIGN_DISTANCE, y=0, ori=0}, "/base_link", "/cam_tag")
   self.skills[1].x = tag_transformed.x
   self.skills[1].y = tag_transformed.y + TAG_OFFSET_Y
   self.skills[1].ori = 0
end

function ALIGN_WITH_LASERLINES:init()
   -- align by ALIGN_DISTANCE from tag to base_link with the lase_line
   local line_transformed = tfm.transform({x=line1:point_on_line(0), y=0, ori=line1:bearing()}, line1:frame_id(), "/base_link")
   printf("line transformed x: %f", line_transformed.x)
   printf("line transformed ori: %f", line_transformed.ori)
   self.skills[1].x = line_transformed.x - ALIGN_DISTANCE
   self.skills[1].y = 0
   self.skills[1].ori = line_transformed.ori
   self.skills[1].tolerance = {x=0.01, y=0.01, ori=0.02}
end

function DRIVE_FORWARD:init()
   local gripper_transformed = tfm.transform({x=ALIGN_DISTANCE, y=0, ori=0}, "/base_link", "/gripper")
   self.skills[1].x = gripper_transformed.x + 0.01 --TODO fix this hardcode in transforms
   self.skills[1].y = gripper_transformed.y
   self.skills[1].ori = 0
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
   self.skills[1].x = -0.2
end
