
----------------------------------------------------------------------------
--  get_produced.lua
--
--  Created: Mar 11 2014 
--  Copyright  2014 Johannes Rothe
--             2014 Tobias Neumann
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
name               = "get_produced"
fsm                = SkillHSM:new{name=name, start="GOTO_MACHINE", debug=true}
depends_skills     = {"motor_move", "ppgoto", "global_motor_move", "wait_produce"}
depends_interfaces = {
  {v = "sensor", type="RobotinoSensorInterface", id = "Robotino"},
  {v = "euclidean_cluster", type="Position3DInterface", id = "Euclidean Laser Cluster"},
  {v = "laserswitch", type="SwitchInterface", id="laser-cluster"},
  {v = "lightswitch", type="SwitchInterface", id="light_front_switch"},
  {v = "light", type ="RobotinoLightInterface", id = "Light_State"},
  {v = "laser_cluster", type="LaserClusterInterface", id="laser-cluster"},
}

documentation      = [==[Get a produced puck from under the RFID]==]

-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants
local LOSTPUCK_DIST = 0.08
local PUCK_SENSOR_INDEX = 8
if config:exists("/hardware/robotino/puck_sensor/trigger_dist") then
   LOSTPUCK_DIST = config:get_float("/hardware/robotino/puck_sensor/trigger_dist")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/trigger_dist FOUND! Using default value\n");
end
if config:exists("/hardware/robotino/puck_sensor/index") then
   -- you can find the config value in /cfg/host.yaml
   PUCK_SENSOR_INDEX = config:get_uint("/hardware/robotino/puck_sensor/index")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/index FOUND! Using default value\n");
end

--fawkes.load_yaml_navgraph already searches in the cfg directory
graph = fawkes.load_yaml_navgraph("navgraph-llsf.yaml")

local LASER_FORWARD_CORRECTION = 0.17
local LIGHT_SENSOR_DELAY_CORRECTION = 0.045
local MIN_VIS_HIST = 15

function get_ampel()
   local ampel_loc = {}
   ampel_loc.x = euclidean_cluster:translation(0)
   ampel_loc.y = euclidean_cluster:translation(1)
   ampel_loc.distance = math.sqrt(ampel_loc.x^2, ampel_loc.y^2)
   ampel_loc.angle = math.atan(ampel_loc.x, ampel_loc.y)
   return ampel_loc
end

function ampel()
   local ampel = get_ampel()
   return (ampel.distance > 0)
      and (ampel.distance < 1)
      and (euclidean_cluster:visibility_history() > MIN_VIS_HIST)
end

fsm:define_states{ export_to=_M, closure={producing_done = producing_done, sensor = sensor, LOSTPUCK_DIST = LOSTPUCK_DIST, PUCK_SENSOR_INDEX = PUCK_SENSOR_INDEX},
   {"GOTO_MACHINE", SkillJumpState, skills={{ppgoto}}, final_to="ADJUST_POS", fail_to="FAILED"},
   {"ADJUST_POS", SkillJumpState, skills={{global_motor_move}}, final_to="WAIT_PRODUCE", fail_to="FAILED"},
   {"WAIT_PRODUCE", SkillJumpState, skills={{wait_produce}}, final_to="SEE_AMPEL", fail_to="FAILED"},
   {"SEE_AMPEL", JumpState},
   {"TURN", SkillJumpState, skills = {{motor_move}}, final_to = "APPROACH_AMPEL", fail_to = "FAILED"},
   {"APPROACH_AMPEL", SkillJumpState, skills = {{motor_move}}, final_to = "LEAVE_AMPEL", fail_to = "FAILED"},
   {"LEAVE_AMPEL", SkillJumpState, skills={{motor_move}}, final_to="CHECK_PUCK", fail_to="FAILED"},
   {"CHECK_PUCK", JumpState},
   {"GET_RID_OF_PUCK", SkillJumpState, skills={{motor_move}}, final_to="GOTO_MACHINE", fail_to="FAILED"}
}

fsm:add_transitions{
   {"SEE_AMPEL", "FAILED", timeout = 5, desc = "No Ampel seen with laser"},
   {"SEE_AMPEL", "TURN", cond = ampel, desc = "Ampel seen with laser"},
   {"CHECK_PUCK", "FINAL", cond = "sensor:distance(PUCK_SENSOR_INDEX) <= LOSTPUCK_DIST", desc = "Final with puck"},
   {"CHECK_PUCK", "FAILED", cond = "sensor:distance(PUCK_SENSOR_INDEX) > LOSTPUCK_DIST", desc = "Failed without puck"},
   -- sensor:distance has to be > 0 due to jitter in the sensor. The sensor either shows 0.00 as distance or the correct value.
   {"GOTO_MACHINE", "GET_RID_OF_PUCK", cond = "sensor:distance(PUCK_SENSOR_INDEX) <= LOSTPUCK_DIST and sensor:distance(PUCK_SENSOR_INDEX) > 0", desc = "Picked up another puck while driving, escape"}
}

function GOTO_MACHINE:init()
   self.skills[1].place = graph:closest_node_to(self.fsm.vars.place, "highway_exit"):name()
end

function ADJUST_POS:init()
   self.skills[1].place = self.fsm.vars.place
end

function WAIT_PRODUCE:init()
   self.skills[1].mtype = self.fsm.vars.place
end

function SEE_AMPEL:init()
   laserswitch:msgq_enqueue_copy(laserswitch.EnableSwitchMessage:new())
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.0))
end

function TURN:init()
   local ampel = get_ampel()
   self.skills[1].ori = math.atan2(ampel.y, ampel.x)
end

function GET_RID_OF_PUCK:init()
   self.skills[1].x = -0.2
   self.skills[1].ori = 0.5
   self.skills[1].vel_trans = 0.8
end

function APPROACH_AMPEL:init()
   --TODO enable vision to see if there is really a puck under the RFID

   local ampel = get_ampel()
   self.skills[1].x = ampel.x - LASER_FORWARD_CORRECTION
   self.skills[1].y = ampel.y
   self.skills[1].vel_trans = 0.1
end

function LEAVE_AMPEL:init()
   if graph:node(self.fsm.vars.place):has_property("leave_right") then
      self.skills[1].y = -0.4
      self.skills[1].vel_rot = 1
   else
      self.skills[1].y = 0.4
      self.skills[1].vel_rot = 1
   end
end

function FINAL:init()
   laserswitch:msgq_enqueue_copy(laserswitch.DisableSwitchMessage:new())
end
