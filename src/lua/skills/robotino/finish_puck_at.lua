
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
depends_skills     = { "take_puck_to", "wait_produce", "deposit_puck", "move_under_rfid", "motor_move", "deliver_puck","global_motor_move", "leave_area_recycle" }
depends_interfaces = {{ v="Pose", type="Position3DInterface", id="Pose" },
   { v="light", type="RobotinoLightInterface", id="Light determined" },
   { v="laser_cluster", type="LaserClusterInterface", id="laser-cluster" }
}

documentation      = [==[Take puck to nearest target in goto_names and take appropriate action at target.]==]

local mpos = require 'machine_pos_module'
local mtype = ""

-- Initialize as skill module
skillenv.skill_module(_M)
graph = fawkes.load_yaml_navgraph("/home/robotino/fawkes-robotino/cfg/navgraph-llsf.yaml")

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

function at_recycle_machine()
   return mtype == "RECYCLE"
end

fsm:define_states{ export_to=_M,
   {"SKILL_TAKE_PUCK", SkillJumpState, skills={{take_puck_to}}, final_to="TIMEOUT",
      fail_to="FAILED", timeout=1},
   {"TIMEOUT", JumpState},
   {"SKILL_GLOBAL_MOTOR_MOVE", SkillJumpState, skills={{global_motor_move}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_RFID", SkillJumpState, skills={{move_under_rfid}}, final_to="SKILL_WAIT_PRODUCE",
      fail_to="SKILL_TAKE_PUCK"},
   {"SKILL_WAIT_PRODUCE", SkillJumpState, skills={{wait_produce}}, final_to="DECIDE_DEPOSIT",
      fail_to="FAILED"},
   {"DECIDE_DEPOSIT", JumpState},
   {"SKILL_DRIVE_LEFT", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},
   {"LEAVE_RECYCLE_AREA", SkillJumpState, skills={{leave_area_recycle}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_DEPOSIT", SkillJumpState, skills={{deposit_puck}}, final_to="FINAL",
      fail_to="FAILED"},
   {"DEPOSIT_THEN_FAIL", SkillJumpState, skills={{deposit_puck}}, final_to="FAILED", fail_to="FAILED"},
   {"SKILL_DELIVER", SkillJumpState, skills={{deliver_puck}}, final_to="FINAL", fail_to="FAILED" }
}

fsm:add_transitions{
   { "TIMEOUT", "FAILED", cond="vars.tries > 3" },
   { "TIMEOUT", "SKILL_GLOBAL_MOTOR_MOVE", timeout=1, desc="test purpose" },
   { "DECIDE_ENDSKILL", "SKILL_RFID", timeout=1, cond=end_rfid, desc="move under rfid" },
   { "DECIDE_ENDSKILL", "SKILL_DELIVER", cond=end_deliver, desc="deliver" },
   { "DECIDE_DEPOSIT", "SKILL_DEPOSIT", cond=prod_unfinished },
   { "DECIDE_DEPOSIT", "LEAVE_RECYCLE_AREA", cond=at_recycle_machine },
   { "DECIDE_DEPOSIT", "SKILL_DEPOSIT", cond=orange_blinking, desc="just deposit the puck and try with a fresh S0" },
   { "DECIDE_DEPOSIT", "SKILL_DRIVE_LEFT", cond=prod_finished}
}

function SKILL_TAKE_PUCK:init()
   self.fsm.vars.goto_name = self.fsm.vars.place or self.fsm.vars.goto_name
   self.skills[1].place = graph:closest_node_to(self.fsm.vars.place, ""):name()
   if not self.fsm.vars.tries then
      self.fsm.vars.tries = 0
   end
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   mtype = self.fsm.vars.mtype
end

function SKILL_GLOBAL_MOTOR_MOVE:init()
   self.skills[1].place = self.fsm.vars.place
   self.skills[1].puck = true
end

function SKILL_DRIVE_LEFT:init()
   if graph:node(self.fsm.vars.goto_name):has_property("leave_right") then
      self.skills[1].y=-0.5
   else
      self.skills[1].y=0.5
   end
end

function SKILL_RFID:init()
   self.skills[1].place = self.fsm.vars.place
end
function SKILL_WAIT_PRODUCE:init()
   self.skills[1].mtype = self.fsm.vars.mtype
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.15))
end

function FINAL:init()
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.0))
end

function FAILED:init()
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.0))
end
  
