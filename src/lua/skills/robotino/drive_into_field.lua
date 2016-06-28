----------------------------------------------------------------------------
--  drive_into_field.lua
--
--  Created: Sat Jun 14 15:13:19 2014
--  Copyright  2015 Tobias Neumann
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
name               = "drive_into_field"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"goto"}
depends_interfaces = { }

documentation      = [==[Drives into field after given offset

Parameters:
   wait: time to wait before start to drive into field
   team: CYAN or MAGENTA
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT_UPPER_LIMIT = 60

fsm:define_states{ export_to=_M,
   --closure={wait=fsm.vars.wait},
   {"INIT",             JumpState},
   {"WAIT",             JumpState},
   {"DRIVE_INTO_FIELD", SkillJumpState, skills={{goto}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT",   "WAIT", cond=true},
   {"WAIT",   "DRIVE_INTO_FIELD", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
}

function INIT:init()
   if self.fsm.vars.team == "CYAN" then
      self.fsm.vars.waypoints = "C-ins-in"
   else
      self.fsm.vars.waypoints = "M-ins-in"
   end
end

function WAIT:init()
   self.timeout_time = self.fsm.vars.wait or 0                    -- this "resets" the timeout of the transition
end

function DRIVE_INTO_FIELD:init()
   self.args["goto"] = {place = self.fsm.vars.waypoints}
end
