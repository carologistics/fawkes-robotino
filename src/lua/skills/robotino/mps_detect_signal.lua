
----------------------------------------------------------------------------
--  explore_signal.lua - Position and detect signal
--
--  Copyright  2015 Victor Mataré
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
name               = "mps_detect_signal"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"mps_align"}
depends_interfaces = {
   { v="bb_signal", type ="RobotinoLightInterface", id = "/machine-signal/best" },
   { v="bb_output", type ="RobotinoLightInterface", id = "Light determined" },
   { v="bb_sw_machine_signal", type="SwitchInterface", id="/machine-signal" },
   { v="bb_signal_hint", type="SignalHintInterface", id="/machine-signal/position-hint" },
   { v="bb_sw_laser", type="SwitchInterface", id="laser-lines" }
}

documentation      = [==[Detect signal at a certain MPS

Parameters:
      place: navgraph name of the machine node, e.g. C-CS1.
      wait_for: GREEN|YELLOW|RED, i.e. become FINAL iff the specified signal is seen.
]==]


-- Tunables
local MIN_VIS_HIST=40
local WAIT_TIMEOUT=120
local LOOK_TIMEOUT=5
local ALIGN_POS = {
   {x=0.5,},
   {x=0.6, ori= 0.07},
   {x=0.6, ori=-0.07},
   {x=0.4, ori= 0.07},
   {x=0.4, ori=-0.07},
   {x=0.5, ori= 0.075},
   {x=0.5, ori=-0.075}
}

-- Initialize as skill module
skillenv.skill_module(_M)

function done()
   return bb_signal:is_ready() and bb_signal:visibility_history() >= MIN_VIS_HIST
end

function invalid_node()
   return not navgraph:node(fsm.vars.place):is_valid()
end

function desired_signal()
   local green = (fsm.vars.wait_for == "GREEN" and bb_signal.ON) or bb_signal.OFF
   local yellow = (fsm.vars.wait_for == "YELLOW" and bb_signal.ON) or bb_signal.OFF
   local red = (fsm.vars.wait_for == "RED" and bb_signal.ON) or bb_signal.OFF

   return bb_signal:is_ready() and bb_signal:visibility_history() >= MIN_VIS_HIST
      and bb_signal:red() == red
      and bb_signal:yellow() == yellow
      and bb_signal:green() == green
end

function fail()
   if fsm.vars.giveup_time then
      return os.time() > fsm.vars.giveup_time
   else
      return fsm.vars.tries >= #ALIGN_POS
   end
end


-- States
fsm:define_states{
   export_to=_M,
   closure={ALIGN_POS=ALIGN_POS, bb_signal=bb_signal, navgraph=navgraph, done=done,
      WAIT_TIMEOUT=WAIT_TIMEOUT, os=os, MIN_VIS_HIST=MIN_VIS_HIST, desired_signal=desired_signal},
   {"INIT", JumpState},
   {"SKILL_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="LOOK", fail_to="FAILED"},
   {"LOOK", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"INIT", "FAILED", precond="not navgraph", desc="navgraph not available"},
   --{"INIT", "FAILED", precond=invalid_node, desc="invalid node"}, -- Agent may send invalid places. Undo when agent is fixed.
   {"INIT", "FAILED", precond="not bb_signal:has_writer()", desc="bb_signal missing"},
   
   {"INIT", "SKILL_ALIGN", cond=true},

   {"LOOK", "FINAL", cond="(not vars.wait_for) and done()"},
   {"LOOK", "FINAL", cond="vars.wait_for and desired_signal()"},
   {"LOOK", "FAILED", cond=fail},
   {"LOOK", "SKILL_ALIGN", cond="(os.time() > vars.look_until) and (vars.best_vis_hist < MIN_VIS_HIST)", desc="move"}
}

function INIT:init()
   self.fsm.vars.tries = 0
   if self.fsm.vars.wait_for then
      self.fsm.vars.giveup_time = os.time() + WAIT_TIMEOUT
   end

   bb_sw_machine_signal:msgq_enqueue_copy(bb_sw_machine_signal.EnableSwitchMessage:new())
 
   if not(self.fsm.vars.place and navgraph:node(self.fsm.vars.place):is_valid()) then
      self.fsm.vars.place = "place-default"
   end
   local node = navgraph:node(self.fsm.vars.place)
   local msg = bb_signal_hint.SignalPositionMessage:new()
   msg:set_translation(0, node:property_as_float("signal_hint_x"))
   msg:set_translation(1, node:property_as_float("signal_hint_y"))
   msg:set_translation(2, node:property_as_float("signal_hint_z"))
   bb_signal_hint:msgq_enqueue_copy(msg)

   bb_sw_laser:msgq_enqueue_copy(bb_sw_laser.EnableSwitchMessage:new())
end

function LOOK:init()
   self.fsm.vars.best_vis_hist = -1
   self.fsm.vars.look_until = os.time() + LOOK_TIMEOUT
end

function LOOK:loop()
   local vis_hist = bb_signal:visibility_history()
   if vis_hist > self.fsm.vars.best_vis_hist then
      self.fsm.vars.best_vis_hist = vis_hist
   end
end

function SKILL_ALIGN:init()
   self.fsm.vars.tries = self.fsm.vars.tries + 1
   printf("attempt #%d", self.fsm.vars.tries)
   self.skills[1].x = ALIGN_POS[self.fsm.vars.tries].x
   self.skills[1].y = ALIGN_POS[self.fsm.vars.tries].y
   self.skills[1].ori = ALIGN_POS[self.fsm.vars.tries].ori
end

function cleanup()
   bb_output:set_red(bb_signal:red())
   bb_output:set_yellow(bb_signal:yellow())
   bb_output:set_green(bb_signal:green())
   bb_output:set_visibility_history(bb_signal:visibility_history())
   bb_output:set_ready(bb_signal:is_ready())

   bb_sw_machine_signal:msgq_enqueue_copy(bb_sw_machine_signal.DisableSwitchMessage:new())
   --bb_sw_laser:msgq_enqueue_copy(bb_sw_laser.DisableSwitchMessage:new())
end

function FINAL:init()
   cleanup()
end

function FAILED:init()
   cleanup()
end
