
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
depends_skills     = { "take_puck_to", "wait_produce", "deposit_puck", "move_under_rfid", "motor_move", "deliver_puck","global_motor_move" }
depends_interfaces = {{ v="Pose", type="Position3DInterface", id="Pose" },
   { v="light", type="RobotinoLightInterface", id="Light determined" },
}

documentation      = [==[Take puck to nearest target in goto_names and take appropriate action at target.

Parameters:
      place: Where to bring the puck to
      out_of_order: behavior when machine is out of order (can be "leave_puck" or "abort" or "ignore")
      mtype: type of machine (needed for production timeouts)
]==]

local mpos = require 'machine_pos_module'
local mtype = ""

-- Initialize as skill module
skillenv.skill_module(_M)
--fawkes.load_yaml_navgraph already searches in the cfg directory
graph = fawkes.load_yaml_navgraph("navgraph-llsf.yaml")

function end_rfid()
   printf(mpos.delivery_goto[fsm.vars.goto_name].d_skill)
   return mpos.delivery_goto[fsm.vars.goto_name].d_skill == "move_under_rfid"
end

function end_deliver()
   printf( mpos.delivery_goto[fsm.vars.goto_name].d_skill)
   return mpos.delivery_goto[fsm.vars.goto_name].d_skill == "deliver_puck"
end

function prod_unfinished()
   return light:yellow() == light.ON
      and light:green()  == light.OFF
      and light:red()    == light.OFF
end

function prod_finished()
   return light:green() == light.ON
      and light:yellow() == light.OFF
      and light:red() == light.OFF
end

function orange_blinking()
   return light:green() == light.OFF
      and light:yellow() == light.BLINKING
      and light:red() == light.OFF
end

function prod_in_progress()
   return light:green() == light.ON
      and light:yellow() == light.ON
      and light:red() == light.OFF
end

function out_of_order()
   return light:green() == light.OFF
      and light:yellow() == light.OFF
      and light:red() == light.ON
end

function out_of_order_abort()
   return out_of_order()
      and fsm.vars.out_of_order == "abort"
end

function out_of_order_leave()
   return out_of_order()
      and fsm.vars.out_of_order == "leave"
end

fsm:define_states{ export_to=_M,
   closure={end_rfid=end_rfid, end_deliver=end_deliver, light=light, orange_blinking=orange_blinking, out_of_order=out_of_order, out_of_order_abort=out_of_order_abort, out_of_order_leave=out_of_order_leave, prod_finished=prod_finished, prod_in_progress=prod_in_progress},
   {"SKILL_TAKE_PUCK", SkillJumpState, skills={{take_puck_to}}, final_to="TIMEOUT",
      fail_to="FAILED", timeout=1},
   {"TIMEOUT", JumpState},
   {"SKILL_GLOBAL_MOTOR_MOVE", SkillJumpState, skills={{global_motor_move}}, final_to="DECIDE_ENDSKILL", fail_to="DECIDE_ENDSKILL"},
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="SKILL_WAIT_PRODUCE",
      fail_to="SKILL_TAKE_PUCK"},
   {"SKILL_WAIT_PRODUCE", SkillJumpState, skills={{wait_produce}}, final_to="DECIDE_DEPOSIT",
      fail_to="PRODUCE_FAILED"},
   {"PRODUCE_FAILED", JumpState},
   {"DECIDE_DEPOSIT", JumpState},
   {"SKILL_DRIVE_LEFT", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_DEPOSIT", SkillJumpState, skills={{deposit_puck}}, final_to="FINAL",
      fail_to="FAILED"},
   {"DEPOSIT_THEN_FAIL", SkillJumpState, skills={{deposit_puck}}, final_to="FAILED", fail_to="FAILED"},
   {"SKILL_DELIVER", SkillJumpState, skills={{deliver_puck}}, final_to="FINAL", fail_to="FAILED" },
   {"LEAVE_PRODUCING_MACHINE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
   -- TODO: Introdude safety check if machine is still producing after leaving it
}

fsm:add_transitions{
   { "TIMEOUT", "FAILED", cond="vars.tries > 3" },
   { "TIMEOUT", "SKILL_GLOBAL_MOTOR_MOVE", timeout=1, desc="test purpose" },
   { "DECIDE_ENDSKILL", "SKILL_RFID", cond=end_rfid, desc="move under rfid" },
   { "DECIDE_ENDSKILL", "SKILL_DELIVER", cond=end_deliver, desc="deliver" },
   { "DECIDE_DEPOSIT", "SKILL_DEPOSIT", cond=prod_unfinished },
   { "DECIDE_DEPOSIT", "SKILL_DRIVE_LEFT", cond="vars.final_product and not orange_blinking()" },
   { "DECIDE_DEPOSIT", "SKILL_DEPOSIT", cond=orange_blinking, desc="just deposit the puck and try with a fresh S0" },
   { "DECIDE_DEPOSIT", "SKILL_DRIVE_LEFT", cond="prod_finished() or out_of_order_abort()"},
   { "DECIDE_DEPOSIT", "LEAVE_PRODUCING_MACHINE", cond="prod_in_progress() or out_of_order_leave()", desc="leave machine to come back and pick up the produced puck later"},
   { "PRODUCE_FAILED", "SKILL_DRIVE_LEFT", cond="vars.final_product"},
   { "PRODUCE_FAILED", "DEPOSIT_THEN_FAIL", cond=true},

}

function SKILL_TAKE_PUCK:init()
   self.fsm.vars.goto_name = self.fsm.vars.place or self.fsm.vars.goto_name
   self.skills[1].place = graph:closest_node_to(self.fsm.vars.place, "highway_exit"):name()
   if not self.fsm.vars.tries then
      self.fsm.vars.tries = 0
   end
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   mtype = self.fsm.vars.mtype
   
   --check out_of_order behavior
   if self.fsm.vars.out_of_order~="ignore" and self.fsm.vars.out_of_order~="leave" and self.fsm.vars.out_of_order~="abort" then
      printf("No/Unknown out_of_order behavior given\n")
      self.fsm.vars.out_of_order = "ignore"
   end
end

function SKILL_GLOBAL_MOTOR_MOVE:init()
   self.skills[1].place = self.fsm.vars.place
   self.skills[1].puck = true
end

function SKILL_DRIVE_LEFT:init()
   if graph:node(self.fsm.vars.goto_name):has_property("leave_right") then
      self.skills[1].y = -0.4
      self.skills[1].vel_rot = 1
      self.skills[1].ori = -1.3
   else
      self.skills[1].y = 0.4
      self.skills[1].vel_rot = 1
      self.skills[1].ori = 1.3
   end
end

function SKILL_DEPOSIT:init()
   self.skills[1].place = self.fsm.vars.place
end

function SKILL_RFID:init()
   self.skills[1].place = self.fsm.vars.place
end

function SKILL_WAIT_PRODUCE:init()
   self.skills[1].mtype = self.fsm.vars.mtype
   self.skills[1].dont_wait = self.fsm.vars.dont_wait
   if self.fsm.vars.out_of_order == "ignore" then
      self.skills[1].out_of_order = "ignore"
   else
      self.skills[1].out_of_order = "final"
   end
end

function LEAVE_PRODUCING_MACHINE:init()
   self.skills[1].x = -0.2
end

function SKILL_DELIVER:init()
   self.skills[1].place = self.fsm.vars.place
end
