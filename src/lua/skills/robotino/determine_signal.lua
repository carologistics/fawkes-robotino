
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
   { v="light", type ="RobotinoAmpelInterface",id = "light", writing =true  } 
  
}


documentation      = [==[
mode € {EXP,  (TEST),  DELIVER,  RECYCLE,  NORMAL,  OUTOFORDER}  € = element aus  

writes ampel data into the light interface 

]==]
-- Constants


-- Initialize as skill module
skillenv.skill_module(...)

--functions
function is_normal()
  	return self.fsm.vars.mode == "NORMAL"
end

function not_normal()
	if not is_normal() then
		ampelswitch:set_enabled(false)
	end 
	return not is_normal()
end

function flashing()
	if self.fsm.vars.yellow_flashing>=2 then
		light:set_state(light.YELLOW_FLASHING) 
		ampelswitch:set_enabled(false) 
	end
	return self.fsm.vars.yellow_flashing>=2
end

function is_processing()
	return ampel_orange:is_enabled() and ampel_green:is_enabled()
end

function is_idle()
	if ampel_green:is_enabled() then 
		light:set_state(light.GREEN) 
	end
	return ampel_green:is_enabled()
end

function is_waiting_4_good()
	if ampel_yellow:is_enabled() then
		light:set_state(light.YELLOW) 
		ampelswitch:set_enabled(false) 
	end
	return ampel_yellow:is_enabled()
end

function is_outoforder()
	if ampel_red:is_enabled() then
		light:set_state(light.RED) 
	end
	return ampel_red:is_enabled()
end
function check_no_change()
	return self.fsm.vars.no_change<=2
end
function no_change()
	if self.fsm.vars.last_color == "green" then
		if ampel_green:is_enabled() then
			light:set_state(light.NO_CHANGE)
			ampelswitch:set_enabled(false) 
			return true
		else 
			self.fsm.vars.no_change=3
		end
	elseif self.fsm.vars.last_color == "yellow" then
		if ampel_orange:is_enabled() then
			light:set_state(light.NO_CHANGE) 
			ampelswitch:set_enabled(false) 
			return true
		else 
			self.fsm.vars.no_change=3
		end
	end

	if ampel_green:is_enabled() and ampel_orange:is_enabled() then
		self.fsm.vars.no_change =3
	elseif ampel_green:is_enabled() then
		self.fsm.vars.last_color = "green"
	elseif ampel_orange:is_enabled() then
		self.fsm.vars.last_color = "yellow"
	else 
		self.fsm.vars.no_change =3
	end

end

function is_offline()
	
	ampelswitch:set_enabled(false) 
	return true
end

-- TODO nochange einbauen
fsm:add_transitions{
	closure={motor=motor}, 
   	{"INIT", "LOOP_START", cond =true}, 
	{"LOOP_START","CHECK",wait_sec = 1.0},
	{"CHECK", "FINAL", cond = not_normal},

	{"CHECK","CHECK_NO_CHANGE",cond = check_no_change},
	{"CHECK_NO_CHANGE","FINAL", cond = no_change},
	{"CHECK_NO_CHANGE","CHECK",wait_sec = 0.5},

	{"CHECK", "CHECK_YELLOW_FLASHING", cond = is_normal}, 
	{"CHECK_YELLOW_FLASHING", "CHECK_YELLOW_FLASHING_2", wait_sec = 0.25}, --TODO testen
	{"CHECK_YELLOW_FLASHING_2", "FINAL",  cond = flashing}, 
	{"CHECK_YELLOW_FLASHING_2", "CHECK_PROCESSING",  cond = true}, 
	{"CHECK_PROCESSING", "LOOP_START",  cond = is_processing}, 
	{"CHECK_PROCESSING", "CHECK_IDLE",  cond = true}, 
	{"CHECK_IDLE",  "FINAL",  cond = is_idle}, 
	{"CHECK_IDLE",  "CHECK_WAITING_4_GOOD",  cond = true}, 
	{"CHECK_WAITING_4_GOOD",  "FINAL",  cond = is_waiting_4_good}, 
	{"CHECK_WAITING_4_GOOD", "CHECK_OUTOFORDER", cond =true}, 
	{"CHECK_OUTOFORDER", "LOOP_START", cond = is_outoforder}, 
	{"CHECK_OUTOFORDER", "FAILED", cond = is_offline, desc = "all turned off"}

}

function INIT:init()
	ampelswitch:set_enabled(true)
	self.fsm.vars.no_change = 0
	self.fsm.vars.last_color = nil
end

function CHECK_NO_CHANGE:init()
	self.fsm.vars.no_change=self.fsm.vars.no_change+1
end

function CHECK:init()
	
	if self.fsm.vars.mode == "OUTOFORDER" then
		if ampel_red:is_enabled() then
	 		light:set_state(light.RED) 
		end
	elseif  self.fsm.vars.mode == "EXP" then	
		if ampel_green:is_enabled() and ampel_orange:is_enabled() then
			light:set_state(light.YELLOW_GREEN) 
		elseif ampel_green:is_enabled() then
			light:set_state(light.GREEN) 
		elseif ampel_orange:is_enabled() then
			light:set_state(light.YELLOW_FLASHING) 
		elseif ampel_orange:is_enabled() then 
			light:set_state(light.RED) 
		elseif not ampel_orange:is_enabled() and not ampel_green:is_enabled() and not ampel_red:is_enabled() then
			light:set_state(light.YELLOW_FLASHING) 
		end
	elseif self.fsm.vars.mode == "DELIVER" then
		if ampel_green:is_enabled() then
			light:set_state(light.GREEN) 
		else 
			light:set_state(light.RED) 	
		end
	elseif self.fsm.vars.mode == "TEST" then
		--to be implemented			
	elseif self.fsm.vars.mode == "RECYCLE" then
		if ampel_red:is_enabled() then
			light:set_state(light.RED) 
		elseif ampel_orange:is_enabled() then
			light:set_state(light.YELLOW_GREEN) 
		elseif ampel_green:is_enabled() then
			light:set_state(light.GREEN)  	
		end
	end
end



function CHECK_YELLOW_FLASHING:init()

	if ampel_orange:is_enabled() or (not ampel_green:is_enabled() and not ampel_orange:is_enabled() and not ampel_red:is_enabled() ) 	then
		self.fsm.vars.yellow_flashing = 1
		if ampel_orange:is_enabled() then
			self.fsm.vars.orange = true			
		else 
			self.fsm.vars.orange = false
		end
	end
end

function CHECK_YELLOW_FLASHING_2:init()
	if ampel_orange:is_enabled() or (not ampel_green:is_enabled() and not ampel_orange:is_enabled() and not ampel_red:is_enabled() ) then
		
		if ampel_orange:is_enabled() and self.fsm.vars.orange ~= true then
			self.fsm.vars.yellow_flashing = self.fsm.vars.yellow_flashing+1
		elseif (not ampel_green:is_enabled() and not ampel_orange:is_enabled() and not ampel_red:is_enabled() ) and 			self.fsm.vars.orange== true then
			self.fsm.vars.yellow_flashing = self.fsm.vars.yellow_flashing+1
			
		end
	end --TODO immer gegenteilig checken
end


