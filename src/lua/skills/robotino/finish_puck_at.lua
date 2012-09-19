
----------------------------------------------------------------------------
--  finish_puck_at.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
name               = "finish_puck_at"
fsm                = SkillHSM:new{name=name, start="SKILL_TAKE_PUCK", debug=true}
depends_skills     = { "take_puck_to", "determine_signal", "deposit_puck", "leave_area", "move_under_rfid", "motor_move", "deliver_puck" }
depends_interfaces = {{ v="Pose", type="Position3DInterface", id="Pose" },
            { v="light", type="RobotinoAmpelInterface", id="light" },
      }

documentation      = [==[Take puck to nearest target in goto_names and take appropriate action at target.]==]

local mpos = require 'machine_pos_module'

-- Initialize as skill module
skillenv.skill_module(_M)

function end_rfid()
   printf( mpos.delivery_goto[fsm.vars.goto_name].d_skill)
   return mpos.delivery_goto[fsm.vars.goto_name].d_skill == "move_under_rfid"
end

function end_deliver()
   printf( mpos.delivery_goto[fsm.vars.goto_name].d_skill)
   return mpos.delivery_goto[fsm.vars.goto_name].d_skill == "deliver"
end

function is_ampel_yellow()
   return light:state() == light.YELLOW
end

function is_not_yellow()
   return not is_ampel_yellow()
end

fsm:define_states{ export_to=_M,
   {"SKILL_TAKE_PUCK", SkillJumpState, skills={{take_puck_to}}, final_to="DECIDE_ENDSKILL",
      fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="SKILL_DETERMINE_SIGNAL",
      fail_to="FAILED"},
   {"SKILL_DETERMINE_SIGNAL", SkillJumpState, skills={{determine_signal}}, final_to="DECIDE",
      fail_to="FAILED"},
   {"DECIDE", JumpState},
   {"SKILL_DRIVE_LEFT", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_DEPOSIT", SkillJumpState, skills={{deposit_puck}}, final_to="SKILL_LEAVE",
      fail_to="FAILED"},
   {"SKILL_LEAVE", SkillJumpState, skills={{leave_area}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_DELIVER", SkillJumpState, skills={{deliver_puck}}, final_to="FINAL", fail_to="FAILED" },
}

fsm:add_transitions{
   { "DECIDE_ENDSKILL", "SKILL_RFID", cond=end_rfid, desc="move under rfid" },
   { "DECIDE_ENDSKILL", "SKILL_DELIVER", cond=end_deliver, desc="deliver" },
   { "DECIDE", "SKILL_DEPOSIT", cond=is_ampel_yellow },
   { "DECIDE", "SKILL_DRIVE_LEFT", cond=is_not_yellow},
}

function SKILL_TAKE_PUCK:init()
   local min_dist = 9999
   if self.fsm.vars.goto_names then
      for i,name in ipairs(self.fsm.vars.goto_names) do
         local gpos = mpos.delivery_goto[name]
         local d = math.sqrt((gpos.x - Pose:translation(0))^2
            + (gpos.y - Pose:translation(1))^2)
         if d < min_dist then
            min_dist = d
            self.fsm.vars.goto_name = name
         end
      end
   end
   self.skills[1].goto_name = self.fsm.vars.goto_name
end

function SKILL_DRIVE_LEFT:init()
   self.skills[1].x=0 
   self.skills[1].y=-0.5 
   self.skills[1].ori=0
end

