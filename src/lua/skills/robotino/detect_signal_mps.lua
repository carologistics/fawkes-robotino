
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
name               = "detect_signal_mps"
fsm                = SkillHSM:new{name=name,  start="INIT",  debug=true}
depends_skills     = nil
depends_interfaces = {
   { v="plugin", type ="RobotinoLightInterface", id = "/machine-signal/best" },
   { v="output", type ="RobotinoLightInterface", id = "Light determined" },
   { v="lightswitch", type="SwitchInterface", id="/machine-signal" },
   { v="bb_signal_hint", type="SignalHintInterface", id="/machine-signal/position-hint" }
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

local hint = require("signal_hint_module")

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

fsm:define_states{ export_to=_M,
   closure={plugin=plugin},
   {"INIT", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", precond=plugin_missing, desc="plugin missing"},
   {"INIT", "FINAL", cond=done},
   {"INIT", "FAILED", timeout=10}
}

function INIT:init()
   lightswitch:msgq_enqueue_copy(lightswitch.EnableSwitchMessage:new())
   hint.send_hint(bb_signal_hint, self.fsm.vars.mtype)
end

function cleanup()
   output:set_red(plugin:red())
   output:set_yellow(plugin:yellow())
   output:set_green(plugin:green())
   output:set_visibility_history(plugin:visibility_history())
   output:set_ready(plugin:is_ready())

--   lightswitch:msgq_enqueue_copy(lightswitch.DisableSwitchMessage:new())
end

function FINAL:init()
   cleanup()
end

function FAILED:init()
   cleanup()
end
