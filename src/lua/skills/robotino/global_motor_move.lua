
----------------------------------------------------------------------------
--  global_motor_move.lua - motor_move with global coordinates. For short
--  distances only (no collision avoidance)!
--
--  Copyright  2014 Victor Mataré
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

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("fawkes.tfutils")

-- Tunables
TOLERANCE = { x = config:get_float_or_default("/skills/global_motor_move/TOLERANCE_x", 0.08), 
              y = config:get_float_or_default("/skills/global_motor_move/TOLERANCE_y", 0.06),
              ori = config:get_float_or_default("/skills/global_motor_move/TOLERANCE_ori", 0.02) }
MAXTRIES = config:get_int_or_default("/skills/global_motor_move/MAXTRIES", 1)
MAX_DIST = config:get_float_or_default("/skills/global_motor_move/MAX_DIST", 1.6)
MAX_POSE_TRIES = config:get_int_or_default("/skills/global_motor_move/MAX_POSE_TRIES", 60)

function invalid_input()
   if fsm.vars.ori and math.abs(fsm.vars.ori) > math.pi then
      return true
   end
   return false
end

function pose_ok()
   local dist = tfm.transform(fsm.vars.target, "/map", "/base_link")
   printf("dist: %f, %f, %f", dist.x, dist.y, dist.ori)
   local pose_ok = math.abs(dist.x) <= TOLERANCE.x
               and math.abs(dist.y) <= TOLERANCE.y
   if fsm.vars.turn == true then
      return pose_ok and math.abs(dist.ori) <= TOLERANCE.ori
   else
      return pose_ok
   end
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

function target_dist_ok()
   local target = tfm.transform(
      {  x = fsm.vars.target.x,
         y = fsm.vars.target.y,
         ori = fsm.vars.target.ori },
      "/map", "/base_link")
   local dist = math.sqrt(target.x * target.x + target.y * target.y)
   return dist < MAX_DIST
end

fsm:define_states{ export_to=_M,
   closure={pose_ok=pose_ok, MAXTRIES=MAXTRIES, mm_tolerance=mm_tolerance, TOLERANCE=TOLERANCE, MAX_POSE_TRIES=MAX_POSE_TRIES},
   {"INIT", JumpState},
   {"WAIT_PLAUSIBLE_POSE", JumpState},
   {"STARTPOSE", JumpState},
   {"TURN", SkillJumpState, skills={{motor_move}}, final_to="WAIT_PLAUSIBLE_TARGET", fail_to="FAILED"},
   {"WAIT_PLAUSIBLE_TARGET", JumpState},
   {"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="DECIDE_TURN", fail_to="FAILED"},
   {"DECIDE_TURN", JumpState},
   {"TURN_BACK", SkillJumpState, skills={{motor_move}}, final_to="WAIT", fail_to="FAILED"},
   {"WAIT", JumpState},
   {"CHECK_POSE", JumpState},
}

fsm:add_transitions{
   {"INIT", "WAIT_PLAUSIBLE_POSE", cond=true},
   {"WAIT_PLAUSIBLE_POSE", "STARTPOSE", cond=target_dist_ok, desc="target dist OK"},
   {"WAIT_PLAUSIBLE_POSE", "FAILED", cond="vars.pose_tries >= MAX_POSE_TRIES", desc="no plausible pose!"},
   {"STARTPOSE", "TURN", cond="vars.puck and vars.bl_target.x < -TOLERANCE.x"},
   {"STARTPOSE", "DRIVE", cond=trans_error},
   {"STARTPOSE", "TURN_BACK", cond="vars.turn == true and math.abs(vars.bl_target.ori) > TOLERANCE.ori"},
   {"STARTPOSE", "FINAL", cond=true},
   {"WAIT_PLAUSIBLE_TARGET", "DRIVE", cond=target_dist_ok, desc="target dist OK"},
   {"WAIT_PLAUSIBLE_TARGET", "FAILED", cond="vars.target_tries >= MAX_POSE_TRIES", desc="no plausible target!"},
   {"DECIDE_TURN", "TURN_BACK", cond="vars.turn == true"},
   {"DECIDE_TURN", "CHECK_POSE", cond="vars.turn == false"},
   {"WAIT", "CHECK_POSE", timeout=1.5},
   {"CHECK_POSE", "WAIT_PLAUSIBLE_POSE", cond="not pose_ok() and vars.tries < MAXTRIES"},
   {"CHECK_POSE", "FINAL", cond=pose_ok},
   {"CHECK_POSE", "FAILED", cond="vars.tries >= MAXTRIES"}
}

function INIT:init()
   self.fsm.vars.startpos = tfm.transform({x=0, y=0, ori=0}, "/base_link", "/map")
   self.fsm.vars.tries = 0
   self.fsm.vars.pose_tries = 0
   if self.fsm.vars.turn == nil then
      self.fsm.vars.turn = true
   end
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

function WAIT_PLAUSIBLE_POSE:loop()
   self.fsm.vars.pose_tries = self.fsm.vars.pose_tries + 1
end

function STARTPOSE:init()
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   self.fsm.vars.bl_target = tfm.transform({
      x=self.fsm.vars.target.x,
      y=self.fsm.vars.target.y,
      ori=self.fsm.vars.target.ori}, "/map", "/base_link")
   printf("%f, %f, %f", self.fsm.vars.bl_target.x, self.fsm.vars.bl_target.y, self.fsm.vars.bl_target.ori)
end

function TURN:init()
   self.fsm.vars.bl_target = tfm.transform({
       x=self.fsm.vars.target.x,
       y=self.fsm.vars.target.y,
       ori=math.atan2(self.fsm.vars.target.y, self.fsm.vars.target.x)},
      "/map", "/base_link")
   printf("tf'd ori: %f", self.fsm.vars.bl_target.ori)
   self.args["motor_move"].ori = math.atan2(self.fsm.vars.bl_target.y, self.fsm.vars.bl_target.x)
   printf("atan ori: %f", self.args["motor_move"].ori)
   self.args["motor_move"].vel_rot = 1.2
   self.args["motor_move"].tolerance = mm_tolerance
end

function WAIT_PLAUSIBLE_TARGET:init()
   self.fsm.vars.target_tries = 0
end

function WAIT_PLAUSIBLE_TARGET:loop()
   self.fsm.vars.target_tries = self.fsm.vars.target_tries + 1
end

function DRIVE:init()
   self.fsm.vars.bl_target = tfm.transform({
      x=self.fsm.vars.target.x,
      y=self.fsm.vars.target.y,
      ori=self.fsm.vars.target.ori}, "/map", "/base_link")
   self.args["motor_move"].x = self.fsm.vars.bl_target.x
   self.args["motor_move"].y = self.fsm.vars.bl_target.y
   self.args["motor_move"].tolerance = mm_tolerance
end

function TURN_BACK:init()
   self.fsm.vars.bl_target = tfm.transform({
      x=self.fsm.vars.target.x,
      y=self.fsm.vars.target.y,
      ori=self.fsm.vars.target.ori}, "/map", "/base_link")
   self.args["motor_move"] =
			{ ori = self.fsm.vars.bl_target.ori,
				vel_rot = 1.2,
				tolerance = mm_tolerance,
			}
end

