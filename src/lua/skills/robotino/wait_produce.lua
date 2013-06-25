
----------------------------------------------------------------------------
--  leave_area.lua - generic global goto
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
----------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License,  or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful, 
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

-- Initialize module
module(...,  skillenv.module_init)

-- Crucial skill information
name               = "wait_produce"
fsm                = SkillHSM:new{name=name,  start="INIT",  debug=false}
depends_skills     = nil
depends_interfaces = {
   { v="plugin", type ="RobotinoLightInterface", id = "Light_State" },
   { v="output", type ="RobotinoLightInterface", id = "Light determined" },
   { v="laserswitch", type="SwitchInterface", id="laser-cluster" },
   { v="lightswitch", type="SwitchInterface", id="light_front_switch" },
   { v="laser_cluster", type="LaserClusterInterface", id="laser-cluster" }
}


documentation      = [==[
writes ampel data into the light interface 
]==]
-- Constants
local TIMEOUTS = {
   T1 = 12,
   T2 = 30,
   T3 = 70,
   T4 = 70,
   T5 = 50
}

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   closure={laserswitch=laserwitch, plugin=plugin}, 
   {"INIT", JumpState},
   {"WAIT", JumpState},
}

function plugin_missing()
   return not (laserswitch:has_writer() and plugin:has_writer())
end

function done()
   if plugin:is_ready() then
      return (plugin:green()      == plugin.ON
              and plugin:yellow() == plugin.OFF
              and plugin:red()    == plugin.OFF)
          or (plugin:green()      == plugin.OFF
              and plugin:yellow() == plugin.ON
              and plugin:red()    == plugin.OFF)
          or (plugin:green()      == plugin.OFF
              and plugin:yellow() == plugin.BLINKING
              and plugin:red()    == plugin.OFF)
   end
   return false
end

function out_of_order()
   return plugin:is_ready()
      and plugin:green()  == plugin.OFF
      and plugin:yellow() == plugin.OFF
      and plugin:red()    == plugin.ON
end

fsm:add_transitions{
   {"INIT", "FAILED", cond=plugin_missing, precond=true},
   {"INIT", "WAIT", timeout=1}, -- let vision settle
   {"WAIT", "WAIT", cond=out_of_order},
   {"WAIT", "FAILED", timeout=fsm.vars.mtype and TIMEOUTS[fsm.vars.mtype] or 70},
   {"WAIT", "FINAL", cond=done},
}

function INIT:init()
   laserswitch:msgq_enqueue_copy(laserswitch.EnableSwitchMessage:new())
   lightswitch:msgq_enqueue_copy(lightswitch.EnableSwitchMessage:new())
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.15))   
end

function WAIT:exit()
   laser_cluster:msgq_enqueue_copy(laser_cluster.SetMaxXMessage:new(0.0))   
end

function FINAL:init()
   laserswitch:msgq_enqueue_copy(laserswitch.DisableSwitchMessage:new())
   output:set_red(plugin:red())
   output:set_yellow(plugin:yellow())
   output:set_green(plugin:green())
   output:set_visibility_history(plugin:visibility_history())
   output:set_ready(plugin:is_ready())
end

