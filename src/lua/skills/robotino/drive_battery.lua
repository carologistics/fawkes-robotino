
----------------------------------------------------------------------------
--  drive_battery.lua
--
--  Created: Tue Feb 15 
--  Copyright  2016  Johannes Rothe
--             
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
name               = "drive_battery"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"ppgoto_waypoints", "ppgoto", "dock_charge"}
depends_interfaces = {
   {v = "battery", type = "BatteryInterface", id="Robotino"},
}

documentation      = [==[Drives between the given list of navgraph-points

Parameters:
      pps: List of points to drive to e.g. drive_test{pps={"P64", "P92", "P73", "P62", "P93"}}
]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local charge_point = "P_CHARGE"
local battery_empty_threshold = 22000
local waypoints = {"P1", "P2"}
local charge_tag_id = 1

function battery_empty()
  print("battery voltage " .. battery:voltage())
  if battery:voltage() < battery_empty_threshold then
     return true
  end
end

function no_battery_writer()
   return not battery:has_writer()
end

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"GOTO", SkillJumpState, skills={{ppgoto_waypoints}}, final_to="GOTO", fail_to="FAILED"},
   {"GOTO_CHARGE", SkillJumpState, skills={{ppgoto}}, final_to="DOCK_CHARGE", fail_to="FAILED"},
   {"DOCK_CHARGE", SkillJumpState, skills={{dock_charge}}, final_to="CHARGE", fail_to="FAILED"},
   {"CHARGE", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond=no_battery_writer, desc="No writer for BatteryInterface"},
   {"INIT", "GOTO", cond=true},
   {"GOTO", "GOTO_CHARGE", cond=battery_empty},
   {"CHARGE", "GOTO", timeout=60},
   --TODO transition to check loading state/time
}

function GOTO:init()
   self.args["ppgoto_waypoints"].wp = waypoints
end

function GOTO_CHARGE:init()
   self.args["ppgoto"].place = charge_point
end

function DOCK_CHARGE:init()
   self.args["dock_charge"].tag_id = charge_tag_id
end
