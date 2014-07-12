
----------------------------------------------------------------------------
--  motor_move_waypoints.lua - call motor_move for an array of waypoints
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
name               = "motor_move_waypoints"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = { "motor_move" }
depends_interfaces = { }

documentation      = [==[
@param waypoints A table of n entries of {x, y, ori} to drive along waypoints.
]==]

-- Initialize as skill module
skillenv.skill_module(_M )

function next_waypoint()
   return fsm.vars.cur_idx < #fsm.vars.waypoints
end

fsm:define_states{ export_to=_M, closure={ next_waypoint=next_waypoint },
   {"INIT", JumpState},
   {"NEXT_WAYPOINT", JumpState},
   {"SKILL_MOTOR_MOVE", SkillJumpState, skills={{motor_move}}, final_to="NEXT_WAYPOINT", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "NEXT_WAYPOINT", cond=true },
   {"NEXT_WAYPOINT", "SKILL_MOTOR_MOVE", cond=next_waypoint, desc="more waypoint(s)" },
   {"NEXT_WAYPOINT", "FINAL", cond="not next_waypoint()", desc="all waypoints done" },
}

function INIT:init()
   self.fsm.vars.cur_idx = 0
end

function SKILL_MOTOR_MOVE:init()
   self.fsm.vars.cur_idx = self.fsm.vars.cur_idx + 1

   self.skills[1].x = self.fsm.vars.waypoints[self.fsm.vars.cur_idx].x
   self.skills[1].y = self.fsm.vars.waypoints[self.fsm.vars.cur_idx].y
   self.skills[1].ori = self.fsm.vars.waypoints[self.fsm.vars.cur_idx].ori
end
