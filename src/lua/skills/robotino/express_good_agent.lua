
----------------------------------------------------------------------------
--  team Robocup_Mexico
--
--  Created: Wed Aug 17 15:23:56 2011
--  Copyright  2012  TEAM ROBOCUP_MEXICO
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
module(..., agentenv.module_init)

-- Crucial agent information
name               = "express_good_agent"
fsm                = AgentHSM:new{name=name, debug=true, start="INIT"}
depends_skills     = {"goto","move_under_rfid","determine_signal","deliver_puck","leave_area_with_puck","fetch_puck", "leave_area","take_puck_to"} --TODO goto durch den skill von victor/sebastian ersetzten
depends_interfaces = {
   { v="worldmodel", type="WorldModel"},
   { v="omnivisionSwitch",type = "SwitchInterface" , id = "omnivisionSwitch"}, --todo
   { v="OmniPuck1",type = "Position3DInterface", id = "OmniPuck1"},
   { v="light",type ="RobotinoAmpelInterface", id = "light" } 
  
}

documentation      = [==[Express Agent]==]
--self.skills[1].args = {}


-- Initialize as agent module
agentenv.agent_module(...)

--local Skill = AgentSkillExecJumpState

function exp_exist()
	return OmniPuck1:visibility_history() > 10	

end

function m_exp_does_exist()

	return worldmodel:getExpressMachine() ~=nil

end
function no_change()
	return light:state() == light.NO_CHANGE
end
function green()
	return  light:state() ==light.GREEN
end



function not_outOfOrder()

	return not  light:state() ==light.RED

end

function outOfOrder()
	return  light:state() ==light.RED
end


-- Setup FSM
fsm:add_transitions{
   closure={},

   {"INIT", "MOVE_TO_EGC", cond = true, desc="initialized/restarted"},
   {"MOVE_TO_EGC","WAIT_PUCK", fail_to ="INIT" ,skills={{"goto"}}},
   {"WAIT_PUCK","FETCH_PUCK", cond = exp_exist},
   {"FETCH_PUCK" , "M_EXP_EXIST" , fail_to  ="INIT" , skills = {{"fetch_puck"}} },
   {"M_EXP_EXIST","GOTO_M_EXP",cond = m_exp_does_exist},
   {"GOTO_M_EXP","CHECK_OUT_OF_ORDER_SKILL" , fail_to = "INIT" , skills = {{"take_puck_to"}} } ,
   {"CHECK_OUT_OF_ORDER_SKILL", "CHECK_OUT_OF_ORDER", fail_to = "INIT", skills = {{"determine_signal"}} ,
   {"CHECK_OUT_OF_ORDER", "MOVE_UNDER_RFID", cond = not_outOfOrder },
   {"CHECK_OUT_OF_ORDER", "GOTO_M_EXP", cond = outOfOrder},
   {"MOVE_UNDER_RFID","CHECK_LIGHT", fail_to = "INIT" ,skills = {{"move_under_rfid"}} },
   {"CHECK_LIGHT", "DECIDE" , fail_to = "INIT" ,skills = {{"determine_signal" }} }, 
   {"DECIDE","LEAVE_AREA_WITH_PUCK",cond = green},
  -- {"DECIDE","",cond = yellow}, gibt es bei exp nicht (wird auch nicht ausgewerted)
  -- {"DECIDE","",cond = red}, kann nicht zurück gegeben werden da ooo schon gescheckt wurde
  --{"DECIDE","",cond = yellow_flashing}, unwarscheinlich da vorher gecheckt wird ob es die richtige machine ist (sonst weltbild fehlerhaft)
   {"DECIDE","MOVE_TO_EGC",cond = no_change}, --kein puck dabei
   {"LEAVE_AREA_WITH_PUCK" , "GOTO_DEL" , fail_to = "INIT" , skills = {{ "leave_area_with_puck"}} },
   {"GOTO_DEL" , "DELIVER",fail_to  = "INIT" , skills = {{"take_puck_to" }} },
   {"DELIVER" , "INIT" , fail_to = "INIT" , skills = {{"deliver_puck"}} }


}
function INIT:init()
--init everything if needed
end

function WAIT_PUCK:init()

switch_4_omni:set_enabled(true)


end


function CHECK_OUT_OF_ORDER_SKILL:init()

	self.skills[1].args = {mode = "OUTOFORDER"}
end

function CHECK_OUT_OF_ORDER:init()
	

	if  light:state() ==light.RED then
		worldmodel:set_new_m_exp()

		self.skill[1].args = {goto_name = worldmodel:getExpressMachine() } 
	end
end

function CHECK_LIGHT:init()

	self.skills[1].args = { mode = "EXP"}


function GOTO_DEL:init()

	self.skills[1].args = {goto_name = "D1"}

end

function GOTO_M_EXP:init()
	if worldmodel:getExpressMachine() ~=nil then
		self.skills[1].args = {goto_name = worldmodel:getExpressMachine() } 	
	
		
	end

end

function MOVE_TO_PUCK:init() 
	self.skills[1].args = {goto_name = "EGI"}
end

