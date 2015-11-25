
----------------------------------------------------------------------------
--  drive_to_global.lua
--
--  Created: Sat Jul 12 13:25:47 2014
--  Copyright  2008       Tim Niemueller [www.niemueller.de]
--             2014-2015  Tobias Neumann
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
name               = "drive_to_global"
fsm                = SkillHSM:new{name=name, start="DRIVE_TO", debug=false}
depends_skills     = { "drive_to" }
depends_interfaces = { }

documentation      = [==[Drive to point with colisoin avoidance and last part with global_motor_move

Parameters:
      place:        Where to drive to
      x:            x coordinate to drive to (if place is not set)
      y:            y coordinate to drive to (if place is not set)
      ori:          alternativ orientation
      just_ori:     if true => after ppgoto finished, global_motor_move is just called with ori (for exploration)
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
  closure={navgraph=navgraph, node_is_valid=node_is_valid},
  {"DRIVE_TO",  SkillJumpState, skills={{drive_to}},  final_to="FINAL", fail_to="FAILED"},
}

--fsm:add_transitions{
--}

function DRIVE_TO:init()
	 self.args["drive_to"] =
			{ x        = self.fsm.vars.x,
				y        = self.fsm.vars.y,
				ori      = self.fsm.vars.ori,
				place    = self.fsm.vars.place,
				just_ori = self.fsm.vars.just_ori
			}
end
