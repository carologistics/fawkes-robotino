
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
fsm                = SkillHSM:new{name=name,  start="INIT",  debug=true}
depends_skills     = nil
depends_interfaces = {
   { v="plugin", type ="RobotinoLightInterface", id = "Light_State" },
   { v="output", type ="RobotinoLightInterface", id = "Light determined" },
   { v="lightswitch", type="SwitchInterface", id="light_front_switch" },
}


documentation      = [==[
writes ampel data into the light interface 

Parameters:
      out_of_order: behavior when machine is out of order (can be "ignore" or "final")
      mtype: type of machine (needed for production timeouts)
]==]
-- Constants
local TIMEOUTS = {
   T1 = 12,
   T2 = 30,
   T3 = 70,
   T4 = 70,
   T5 = 50,
   RECYCLE = 10
}

-- Initialize as skill module
skillenv.skill_module(_M)

function plugin_missing()
   return not  plugin:has_writer()
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
          or (plugin:green()      == plugin.ON
              and plugin:yellow() == plugin.ON
              and plugin:red()    == plugin.OFF)
              and fsm.vars.dont_wait
   end
   return false
end

function out_of_order()  -- careful!!! no check if plugin light is ready
   return plugin:green()  == plugin.OFF
      and plugin:yellow() == plugin.OFF
      and plugin:red()    == plugin.ON
end

function final_when_out_of_order()
   return fsm.vars.out_of_order == "final"
end

fsm:define_states{ export_to=_M,
   closure={plugin=plugin, out_of_order=out_of_order, final_when_out_of_order=final_when_out_of_order},
   {"INIT", JumpState},
   {"OUT_OF_ORDER", JumpState},
   {"WAIT", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", precond=plugin_missing},
   {"INIT", "WAIT", timeout=2}, -- let vision settle
   {"WAIT", "OUT_OF_ORDER", cond="out_of_order() and plugin:is_ready()"},
   {"OUT_OF_ORDER", "WAIT", cond="not out_of_order() and plugin:is_ready()"},
   {"OUT_OF_ORDER", "WAIT", timeout=120},
   {"OUT_OF_ORDER", "FINAL", cond="final_when_out_of_order()"},
   {"WAIT", "FAILED", timeout=fsm.vars.mtype and TIMEOUTS[fsm.vars.mtype] or 70},
   {"WAIT", "FINAL", cond=done},
}

function INIT:init()
   lightswitch:msgq_enqueue_copy(lightswitch.EnableSwitchMessage:new())

   --check behavior when machine is out of order
   if self.fsm.vars.out_of_order~="ignore" and self.fsm.vars.out_of_order~="final" then
      printf("No/Unknown out_of_order behavior given\n")
      self.fsm.vars.out_of_order = "ignore"
   end
end

function WAIT:exit()
end

function cleanup()
   output:set_red(plugin:red())
   output:set_yellow(plugin:yellow())
   output:set_green(plugin:green())
   output:set_visibility_history(plugin:visibility_history())
   output:set_ready(plugin:is_ready())
end

function FINAL:init()
   cleanup()
end

function FAILED:init()
   cleanup()
end
