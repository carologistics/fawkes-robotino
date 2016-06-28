
----------------------------------------------------------------------------
--  test_conveyor_align.lua
--
--  Created: Thu Jun 28 15:49:47 2016
--  Copyright  2016  David Schmidt
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
name               = "test_conveyor_align"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"tagless_mps_align", "motor_move"}
depends_interfaces = {}

documentation      = [==[ test_conveyor_align

                          This skill does:
                             aligns with help of the laser data in front of the conveyor

                          @param input     bool if the approached side is input or not
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

-- CONSTANTS
Y_OFFSET_CONVEYOR = 0.025


fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"ALIGN_LASERLINE", SkillJumpState, skills={{tagless_mps_align}}, final_to="ALIGN_CONVEYOR", fail_to="FAILED"},
   {"ALIGN_CONVEYOR", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "ALIGN_LASERLINE", cond=true},
}

function ALIGN_LASERLINE:init()
   self.args["tagless_mps_align"].ori = math.pi/2
end

function ALIGN_CONVEYOR:init()
   if self.fsm.vars.input then
      self.args["motor_move"] = {y = Y_OFFSET_CONVEYOR, vel_trans = 0.2, tolerance = { x=0.002, y=0.002, ori=0.01 }}
   else
      self.args["motor_move"] = {y = -Y_OFFSET_CONVEYOR, vel_trans = 0.2, tolerance = { x=0.002, y=0.002, ori=0.01 }}
   end
end
