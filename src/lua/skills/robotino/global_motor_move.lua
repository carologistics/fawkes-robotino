
----------------------------------------------------------------------------
--  motor_move.lua - stupidly move to some odometry position
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
name               = "global_motor_move"
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = { "motor_move" }
depends_interfaces = {
    {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[
]==]

-- Tunables
TOLERANCE = { x=0.08, y=0.04, ori=0.02 }
MAXTRIES = 3

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("tf_module")
local navgraph = fawkes.load_yaml_navgraph("/home/robotino/fawkes-robotino/cfg/navgraph-llsf.yaml")

function invalid_input()
   if fsm.vars.ori and math.abs(fsm.vars.ori) > math.pi then
      return true
   end
   return false
end

function pose_ok()
   local dist = tfm.transform(fsm.vars.target, "/map", "/base_link")
   printf("dist: %f, %f, %f", dist.x, dist.y, dist.ori)
   return math.abs(dist.x) <= TOLERANCE.x
      and math.abs(dist.y) <= TOLERANCE.y
      and math.abs(dist.ori) <= TOLERANCE.ori
end

local mm_tolerance = {
   x = TOLERANCE.x * 0.75,
   y = TOLERANCE.y * 0.75,
   ori = TOLERANCE.ori * 0.75
}

function trans_error()
   return math.abs(fsm.vars.bl_target.x) > mm_tolerance.x
      or math.abs(fsm.vars.bl_target.y) > mm_tolerance.y
end

fsm:define_states{ export_to=_M,
   closure={pose_ok=pose_ok, MAXTRIES=MAXTRIES, mm_tolerance=mm_tolerance, TOLERANCE=TOLERANCE},
   {"INIT", JumpState},
   {"STARTPOSE", JumpState},
   {"TURN", SkillJumpState, skills={{motor_move}}, final_to="DRIVE", fail_to="FAILED"},
   {"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="TURN_BACK", fail_to="FAILED"},
   {"TURN_BACK", SkillJumpState, skills={{motor_move}}, final_to="WAIT", fail_to="FAILED"},
   {"WAIT", JumpState},
   {"CHECK_POSE", JumpState},
}

fsm:add_transitions{
   {"INIT", "STARTPOSE", cond=true},
   {"STARTPOSE", "TURN", cond="vars.puck and vars.bl_target.x < -TOLERANCE.x"},
   {"STARTPOSE", "DRIVE", cond=trans_error},
   {"STARTPOSE", "TURN_BACK", cond="math.abs(vars.bl_target.ori) > TOLERANCE.ori"},
   {"STARTPOSE", "FINAL", cond=true},
   {"WAIT", "CHECK_POSE", timeout=1.5},
   {"CHECK_POSE", "STARTPOSE", cond="not pose_ok() and vars.tries < MAXTRIES"},
   {"CHECK_POSE", "FINAL", cond=pose_ok},
   {"CHECK_POSE", "FAILED", cond="vars.tries >= MAXTRIES"}
}

function INIT:init()
   self.fsm.vars.startpos = tfm.transform({x=0, y=0, ori=0}, "/base_link", "/map")
   self.fsm.vars.tries = 0
   local x = self.fsm.vars.x or self.fsm.vars.startpos.x
   local y = self.fsm.vars.y or self.fsm.vars.startpos.y
   local ori = self.fsm.vars.ori or self.fsm.vars.startpos.ori
   if type(self.fsm.vars.place) == "string" then
      local node = navgraph:node(self.fsm.vars.place)
      if node then
         x = node:x()
         y = node:y()
         ori = node:has_property("orientation")
            and node:property_as_float("orientation")
            or self.fsm.vars.startpos.ori
      end
   end
   self.fsm.vars.target = { x=x, y=y, ori=ori }
end

function STARTPOSE:init()
   self.fsm.vars.bl_target = tfm.transform({
      x=self.fsm.vars.target.x,
      y=self.fsm.vars.target.y,
      ori=self.fsm.vars.target.ori}, "/map", "/base_link")
   printf("%f, %f, %f", self.fsm.vars.bl_target.x, self.fsm.vars.bl_target.y, self.fsm.vars.bl_target.ori)
end

function TURN:init()
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   self.fsm.vars.bl_target = tfm.transform({
       x=self.fsm.vars.target.x,
       y=self.fsm.vars.target.y,
       ori=math.atan2(self.fsm.vars.target.y, self.fsm.vars.target.x)},
      "/map", "/base_link")
   printf("tf'd ori: %f", self.fsm.vars.bl_target.ori)
   self.skills[1].ori = math.atan2(self.fsm.vars.bl_target.y, self.fsm.vars.bl_target.x)
   printf("atan ori: %f", self.skills[1].ori)
   self.skills[1].vel_rot = 1.2
   self.skills[1].puck = true
   self.skills[1].tolerance = mm_tolerance
   self.skills[1].frame = "/odom"
end

function DRIVE:init()
   self.fsm.vars.bl_target = tfm.transform({
      x=self.fsm.vars.target.x,
      y=self.fsm.vars.target.y,
      ori=self.fsm.vars.target.ori}, "/map", "/base_link")
   self.skills[1].x = self.fsm.vars.bl_target.x
   self.skills[1].y = self.fsm.vars.bl_target.y
   self.skills[1].puck = true
   self.skills[1].tolerance = mm_tolerance
   self.skills[1].frame = "/odom"
end

function TURN_BACK:init()
   self.fsm.vars.bl_target = tfm.transform({
      x=self.fsm.vars.target.x,
      y=self.fsm.vars.target.y,
      ori=self.fsm.vars.target.ori}, "/map", "/base_link")
   self.skills[1].ori = self.fsm.vars.bl_target.ori
   self.skills[1].vel_rot = 1.2
   self.skills[1].puck = true
   self.skills[1].tolerance = mm_tolerance
   self.skills[1].frame = "/odom"
end

