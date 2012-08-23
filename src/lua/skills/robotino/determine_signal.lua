
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
name               = "determine_signal"
fsm                = SkillHSM:new{name=name,  start="INIT",  debug=false}
depends_skills     = nil
depends_interfaces = {
   {v ="ampelswitch",  type = "SwitchInterface",  id = "ampelswitch"},
   { v="ampel_green", type = "SwitchInterface",  id = "ampel_green"}, 
   { v="ampel_red", type = "SwitchInterface",  id = "ampel_red"}, 
   { v="ampel_orange", type = "SwitchInterface",  id = "ampel_orange"}, 
   { v="light", type ="RobotinoAmpelInterface",id = "light", writing=true }   
}


documentation      = [==[
<<<<<<< HEAD
mode € {EXP,  (TEST),  DELIVER,  RECYCLE,  NORMAL,  OUTOFORDER}  € = element aus  
=======
mode is one of {EXP,  (TEST),  DELIVER,  RECYCLE,  NORMAL,  OUTOFORDER}
>>>>>>> skills: rewritten determine_signal skill

writes ampel data into the light interface 

]==]
-- Constants


-- Initialize as skill module
skillenv.skill_module(...)

--functions

fsm:add_transitions{
   closure={motor=motor, ampelswitch=ampelswitch, ampel_red=ampel_red}, 
   {"INIT", "FAILED", cond="not ampelswitch:has_writer()", precond=true},
   {"INIT", "DETERMINE", wait_sec=1}, -- let vision settle
   {"INIT", "WAIT_OUT_OF_ORDER", cond="ampel_red:is_enabled()", precond=true},
   {"WAIT_OUT_OF_ORDER", "DETERMINE", cond="not ampel_red:is_enabled()"},
   {"DETERMINE", "CHECK_NO_CHANGE", wait_sec=2.0},
   {"DETERMINE", "WAIT_OUT_OF_ORDER", cond="ampel_red:is_enabled()"},
   {"DETERMINE", "STOP_VISION", cond="vars.determined"},
   {"CHECK_NO_CHANGE", "NO_CHANGE", "not vars.changed"},
   {"CHECK_NO_CHANGE", "DETERMINE", true},
   {"NO_CHANGE", "STOP_VISION", true},
   {"STOP_VISION","FINAL",true}
}

function INIT:init()
   local m = ampelswitch.EnableSwitchMessage:new()
   ampelswitch:msgq_enqueue_copy(m)
end

function STOP_VISION:init()
   local m = ampelswitch.DisableSwitchMessage:new()
   ampelswitch:msgq_enqueue_copy(m)
end

function DETERMINE:loop()
   if ampel_green:is_enabled() and ampel_orange:is_enabled() then
      self.fsm.vars.changed = true
   elseif ampel_green:is_enabled() then
      if self.fsm.vars.changed then
         light:set_state(light.GREEN) 
         self.fsm.vars.determined = true
      end
   else
      -- orange is "special", it might be flashing or solid
      if ampel_orange:is_enabled() then
         if self.fsm.vars.orange_on_time then
            local now = fawkes.Time:new()
            if now - self.fsm.vars.orange_on_time >= 0.5
               and self.fsm.vars.changed
            then
               light:set_state(light.YELLOW)
               self.fsm.vars.determined = true
            end
         else
            self.fsm.vars.orange_on_time = fawkes.Time:new()
         end
      else
         if self.fsm.vars.orange_on_time then
            -- it was on -> flashing
            light:set_state(light.YELLOW_FLASHING)
            self.fsm.vars.determined = true
         end
      end
   end
end

function NO_CHANGE:init()
   light:set_state(light.NO_CHANGE)
end
