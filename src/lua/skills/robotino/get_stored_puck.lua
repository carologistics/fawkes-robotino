----------------------------------------------------------------------------
--  get_stored_puck.lua
--
--  Created: Sun Jun 29 16:02:02 2014
--  Copyright  2014 Frederik Zwilling
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
name               = "get_stored_puck"
fsm                = SkillHSM:new{name=name, start="GOTO", debug=false}
depends_skills     = {"ppgoto", "motor_move", "global_motor_move", "fetch_puck"}
depends_interfaces = {
   {v = "ppnavi", type = "NavigatorInterface"},
   {v = "motor", type = "MotorInterface", id="Robotino"},
   {v = "sensor", type="RobotinoSensorInterface", id = "Robotino"},
   {v = "puck_0", type="Position3DInterface", id="puck_0"}
}

documentation      = [==[
      Get a previously stored pucka puck from a given place

      Parameters:
        place: name of the storage place
]==]
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

function puck_visible()
   return puck_0:visibility_history() >= 1
end

fsm:define_states{ export_to=_M, closure={sensor = sensor, LOSTPUCK_DIST = LOSTPUCK_DIST, PUCK_SENSOR_INDEX = PUCK_SENSOR_INDEX},
   {"GOTO", SkillJumpState, skills={{ppgoto}}, final_to="ADJUST_POS", fail_to="FAILED"},
   {"ADJUST_POS", SkillJumpState, skills={{global_motor_move}}, final_to="GRAB", fail_to="FAILED"},
   {"GRAB", SkillJumpState, skills={{fetch_puck}}, final_to="BACK_UP", fail_to="FAILED"},
   {"BACK_UP", SkillJumpState, skills={{motor_move}}, final_to="LEAVE", fail_to="FAILED"},
   {"LEAVE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   -- sensor:distance has to be > 0 due to jitter in the sensor. The sensor either shows 0.00 as distance or the correct value.
   {"GOTO", "GET_RID_OF_PUCK", cond = "sensor:distance(PUCK_SENSOR_INDEX) <= LOSTPUCK_DIST and sensor:distance(PUCK_SENSOR_INDEX) > 0", desc = "Picked up another puck while driving, escape"}
}

function GOTO:init()
   if self.fsm.vars.place == nil then
      printf("Called store_puck without parameter place!")
   end
   self.skills[1].place = self.fsm.vars.place
end

function ADJUST_POS:init()
   self.skills[1].place = self.fsm.vars.place
end

function GRAB:init()
   -- is this needed? (stands in get_s0)
   ppnavi:msgq_enqueue_copy(ppnavi.StopMessage:new())
end

function BACK_UP:init()
   self.skills[1].ori = 3.14
end

function LEAVE:init()
   self.skills[1].x = 0.1
end
