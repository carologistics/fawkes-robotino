
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

documentation      = [==[Move on a (kind of) straight line relative to /base_link.
@param x The target X coordinate
@param y Dito
@param ori Relative rotation. -pi <= ori <= pi.
@param frame Measure distances relative to this frame. Can be "/map" or "/odom" (default).
@param vel_trans Translational top-speed. Upper limit: hardcoded tunable in skill module.
@param vel_rot Rotational top-speed. Upper limit: dito.
]==]

-- Tunables

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

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"TURN", SkillJumpState, skills={{motor_move}}, final_to="DRIVE", fail_to="FAILED"},
   {"DRIVE", SkillJumpState, skills={{motor_move}}, final_to="TURN_BACK", fail_to="FAILED"},
   {"TURN_BACK", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   closure={motor=motor},
   {"INIT", "TURN", cond="vars.puck and vars.dist_target.x < 0"},
}

function INIT:init()
   self.fsm.vars.mypos = tfm.transform({x=0, y=0, ori=0}, "/base_link", "/map")
   local x = self.fsm.vars.x or self.fsm.vars.mypos.x
   local y = self.fsm.vars.y or self.fsm.vars.mypos.y
   local ori = self.fsm.vars.ori or self.fsm.vars.mypos.ori
   if type(self.fsm.vars.place) == "string" then
      local node = navgraph:node(self.fsm.vars.place)
      if node then
         x = node:x()
         y = node:y()
         ori = node:has_property("orientation")
            and node:property_as_float("orientation")
            or self.fsm.vars.mypos.ori
      end
   end
   self.fsm.vars.target = { x=x, y=y, ori=ori }

   self.fsm.vars.dist_target = tfm.transform(target, "/map", "/base_link")
end

function TURN:init()
   self.skills[1].ori = math.tan(self.fsm.vars.target.x/self.fsm.vars.target.y)
   self.skills[1].frame = "/map"
end

function DRIVE:init()
   self.skills[1].x = self.fsm.vars.target.x
   self.skills[1].y = self.fsm.vars.terget.y
   self.skills[1].frame = "/map"
end

function TURN_BACK:init()
   self.skills[1].ori = self.fsm.vars.target.ori
   self.skills[1].frame = "/map"
end

