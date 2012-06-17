
----------------------------------------------------------------------------
--  chase_puck.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
name               = "deliver_puck"
fsm                = SkillHSM:new{name=name, start="CHECK_PUCK_FETCHED", debug=true}
depends_skills     = {"goto","move_under_rfid","determine_signal","leave_area"}
depends_interfaces = {
	{ v="light",type ="RobotinoAmpelInterface" } 
 

}

documentation     = [==[delivers already fetched puck to specified location]==]
-- Constants

local THRESHOLD_DISTANCE =0.1

-- Initialize as skill module
skillenv.skill_module(...)

function puck_not_infront()
        local curDistance = sensor:distance(0)
        if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
  
                return false
        end
        return true


end

function puck_infront()
local curDistance = sensor:distance(0)
        if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
                printf("puckgrapped")
                return true
        end
        return false



end

function ampel_green()

	return light:state() ==light.GREEN

end

function traffic_light_red()
return not traffic_light_green
end


fsm:add_transitions{
	closure={motor=motor},
	{"CHECK_PUCK_FETCHED", "FAILED", cond=puck_not_infront, desc="No puck seen by Infrared"},
	{"CHECK_PUCK_FETCHED", "CHECK_1ST_TRAFFIC_LIGHT", cond=puck_infront},
	{"CHECK_1ST_TRAFFIC_LIGHT","1ST_CHECKED",skill= determine_signal, fail_to="FAILED"},
	{"1ST_CHECKED" , "MOVE_UNDER_RFID", cond = ampel_green},
	{"1ST_CHECKED", "MOVE_TO_2ND" ,cond = true},
	{"MOVE_TO_2ND", "CHECK_2ND_TRAFFIC_LIGHT",skill = goto , fail_to = "FAILED"},
	{"CHECK_2ND_TRAFFIC_LIGHT" , "2ND_CHECKED",skill= determine_signal, fail_to="FAILED"},
	{"2ND_CHECKED" , "MOVE_UNDER_RFID", cond = ampel_green},
	{"2ND_CHECKED", "MOVE_TO_3RD", cond = true}, 
	{"MOVE_TO_3RD", "CHECK_3RD_TRAFFIC_LIGHT", skill = goto,fail_to ="FAILED"},
	{"CHECK_3RD_TRAFFIC_LIGHT" , "3RD_CHECKED",skill= determine_signal, fail_to="FAILED"},
	{"3RD_CHECKED" , "MOVE_UNDER_RFID", cond = ampel_green},
	{"3RD_CHECKED",  "FAILED",cond = true,desc = "all delivery spots appear unuseable"},

	{"MOVE_UNDER_RFID","LEAVE_AREA",skill = move_under_rfid, fail_to= "FAILED"},
	{"LEAVE_AREA" , "FINAL" , skill = leave_area, fail_to="FAILED"}



}

function CHECK_1ST_TRAFFIC_LIGHT:init()
	self.args = {mode = "DELIVER"}

end

function MOVE_TO_2ND:init()
	self.args = {goto_name = "D2"}
end
function MOVE_TO_3RD:init()
	self.args = {goto_name = "D3"}
end

