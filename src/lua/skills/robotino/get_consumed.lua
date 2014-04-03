----------------------------------------------------------------------------
--  get_consumed.lua
--
--  Created: Tue Oct 08 21:01:29 2013
--  Copyright  2013  Frederik Zwilling
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
name               = "get_consumed"
fsm                = SkillHSM:new{name=name, start="GOTO_MASCHINE", debug=false}
depends_skills     = {"motor_move", "ppgoto", "global_motor_move", "fetch_puck"}
depends_interfaces = {
   {v = "ppnavi", type = "NavigatorInterface"},
   {v = "sensor", type="RobotinoSensorInterface"},
   {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[Drive to Maschine and ged a consumed puck]==]

-- Initialize as skill module
skillenv.skill_module(_M)
graph = fawkes.load_yaml_navgraph("navgraph-llsf.yaml")


--fawkes.load_yaml_navgraph already searches in the cfg directory
graph = fawkes.load_yaml_navgraph("navgraph-llsf.yaml")

function have_puck()
    local curDistance = sensor:distance(8)
    if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
        printf("near: " .. curDistance)
        return true
    end
    return false
end

fsm:define_states{ export_to=_M,
   {"GOTO_MASCHINE", SkillJumpState, skills={{ppgoto}}, final_to="ADJUST_POS", fail_to="FAILED"},
   {"ADJUST_POS", SkillJumpState, skills={{global_motor_move}}, final_to="DRIVE_TO_SIDE", fail_to="FAILED"},
   {"DRIVE_TO_SIDE", SkillJumpState, skills={{motor_move}}, final_to="GRAP_CONSUMED",
      fail_to="FAILED"},
   {"GRAP_CONSUMED", SkillJumpState, skills={{fetch_puck}}, final_to="LEAVE_TURN", fail_to="FAILED"},
   {"LEAVE_TURN", SkillJumpState, skills={{motor_move}}, final_to="FINAL",
      fail_to="FAILED"},
}

function GOTO_MASCHINE:init()
   self.skills[1].place = graph:closest_node_to(self.fsm.vars.place, "highway_exit"):name()
end

function ADJUST_POS:init()
   self.skills[1].place = self.fsm.vars.place
end

function DRIVE_TO_SIDE:init()
   if graph:node(self.fsm.vars.place):has_property("leave_right") then
      self.skills[1].x=0.1
      self.skills[1].y=-0.4
      self.skills[1].ori=0.35
   else
      self.skills[1].x=0.1
      self.skills[1].y=0.4
      self.skills[1].ori=-0.35
   end
end

function LEAVE_TURN:init()
   if graph:node(self.fsm.vars.place):has_property("leave_right") then
      self.skills[1].x=0
      self.skills[1].y=-0.2
      self.skills[1].ori=-1
      self.skills[1].vel_rot=1
   else
      self.skills[1].x=0
      self.skills[1].y=0.2
      self.skills[1].ori=1
      self.skills[1].vel_rot=1
   end
end

