----------------------------------------------------------------------------
--  turn_to_search.lua
--
--  Created: Thu Jun 08 15:25:00 2023
--  Copyright  2023 Daniel Swoboda
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
name               = "turn_to_search"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move"}
depends_interfaces = {
     {v = "pose",      type="Position3DInterface", id="Pose"}
}

documentation      = [==[Turns and waits the robot in 45 degree intervals

]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT_UPPER_LIMIT = 5

fsm:define_states{ export_to=_M,
   {"INIT",             JumpState},
   {"WAIT1",             JumpState},
   {"TURN1", SkillJumpState, skills={{motor_move}}, final_to="WAIT2"},
   {"WAIT2",             JumpState},
   {"TURN2", SkillJumpState, skills={{motor_move}}, final_to="WAIT3"},
   {"WAIT3",             JumpState},
   {"TURN3", SkillJumpState, skills={{motor_move}}, final_to="WAIT4"},
   {"WAIT4",             JumpState},
   {"TURN4", SkillJumpState, skills={{motor_move}}, final_to="WAIT5"},
   {"WAIT5",             JumpState},
   {"TURN5", SkillJumpState, skills={{motor_move}}, final_to="FINAL"}
}

fsm:add_transitions{
   {"INIT",   "WAIT1", cond=true},
   {"WAIT1",   "TURN1", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
   {"TURN1",   "WAIT2", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
   {"WAIT2",   "TURN2", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
   {"TURN2",   "WAIT3", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
   {"WAIT3",   "TURN3", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
   {"TURN3",   "WAIT4", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
   {"WAIT4",   "TURN4", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
   {"TURN4",   "WAIT5", timeout=TIMEOUT_UPPER_LIMIT},   -- this just creates the transision
   {"WAIT5",   "TURN5", timeout=TIMEOUT_UPPER_LIMIT}   -- this just creates the transision
}

function WAIT1:init()
   self.timeout_time = 4
end

function TURN1:init()
   self.args["motor_move"] = {
      ori = 1.25663704
   }
end

function WAIT2:init()
   self.timeout_time = 4
end

function TURN2:init()
   self.args["motor_move"] = {
      ori = 1.25663704
   }
end

function WAIT3:init()
   self.timeout_time = 4
end

function TURN3:init()
   self.args["motor_move"] = {
      ori = 1.25663704
   }
end

function WAIT4:init()
   self.timeout_time = 4
end

function TURN4:init()
   self.args["motor_move"] = {
      ori = 1.25663704
   }
end

function WAIT5:init()
   self.timeout_time = 4
end

function TURN5:init()
   self.args["motor_move"] = {
      ori = 1.25663704
   }
end

