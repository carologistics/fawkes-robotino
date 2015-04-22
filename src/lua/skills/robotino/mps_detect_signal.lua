
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
name               = "mps_detect_signal"
fsm                = SkillHSM:new{name=name,  start="INIT",  debug=true}
depends_skills     = nil
depends_interfaces = {
   { v="bb_signal", type ="RobotinoLightInterface", id = "/machine-signal/best" },
   { v="bb_output", type ="RobotinoLightInterface", id = "Light determined" },
   { v="bb_sw_machine_signal", type="SwitchInterface", id="/machine-signal" },
   { v="bb_signal_hint", type="SignalHintInterface", id="/machine-signal/position-hint" },
   { v="bb_sw_laser", type="SwitchInterface", id="laser-lines" }
}



documentation      = [==[
writes ampel data into the light interface 

Parameters:
      place: navgraph name of the machine node, e.g. C-CS1.
]==]


-- Initialize as skill module
skillenv.skill_module(_M)


function done()
   return bb_signal:is_ready() and bb_signal:visibility_history() > 25
end

function invalid_node()
   return not navgraph:node(fsm.vars.place):is_valid()
end


fsm:define_states{ export_to=_M,
   closure={bb_signal=bb_signal, navgraph=navgraph},
   {"INIT", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", precond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", precond=invalid_node, desc="invalid node"},
   {"INIT", "FAILED", precond="not bb_signal:has_writer()", desc="bb_signal missing"},
   {"INIT", "FINAL", cond=done},
   {"INIT", "FAILED", timeout=10}
}


function INIT:init()
   bb_sw_machine_signal:msgq_enqueue_copy(bb_sw_machine_signal.EnableSwitchMessage:new())

   local node = navgraph:node(self.fsm.vars.place)
   local msg = bb_signal_hint.SignalPositionMessage:new()
   msg:set_translation(0, node:property_as_float("signal_hint_x"))
   msg:set_translation(1, node:property_as_float("signal_hint_y"))
   msg:set_translation(2, node:property_as_float("signal_hint_z"))
   bb_signal_hint:msgq_enqueue_copy(msg)

   bb_sw_laser:msgq_enqueue_copy(bb_sw_laser.EnableSwitchMessage:new())
end

function cleanup()
   bb_output:set_red(bb_signal:red())
   bb_output:set_yellow(bb_signal:yellow())
   bb_output:set_green(bb_signal:green())
   bb_output:set_visibility_history(bb_signal:visibility_history())
   bb_output:set_ready(bb_signal:is_ready())

   --bb_sw_machine_signal:msgq_enqueue_copy(bb_sw_machine_signal.DisableSwitchMessage:new())
   --bb_sw_laser:msgq_enqueue_copy(bb_sw_laser.DisableSwitchMessage:new())
end

function FINAL:init()
   cleanup()
end

function FAILED:init()
   cleanup()
end
