
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
depends_skills     = nil
depends_interfaces = {
--	{v = "OmniPuck1", type="Position3DInterface",id = "OmniPuck1"},
	{v = "sensor", type="RobotinoSensorInterface"},
	{v = "motor", type="MotorInterface"}
--	{v = "navigator", type="NavigatorInterface"}

}

documentation     = [==[delivers already fetched puck to specified location]==]
-- Constants

local THRESHOLD_DISTANCE =0.1
local traffic_light_green_var = false

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
print("bla")

local curDistance = sensor:distance(0)
        if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
                printf("puckgrapped")
                return true
        end
        return false



end

function traffic_light_green()

return traffic_light_green_var --von visiongruppe nachzureichen
end

function traffic_light_red()
return not traffic_light_green
end


fsm:add_transitions{
	closure={motor=motor},
	{"CHECK_PUCK_FETCHED", "FAILED", cond=puck_not_infront, desc="No puck seen by Infrared"},
	{"CHECK_PUCK_FETCHED", "CHECK_1ST_TRAFFIC_LIGHT", cond=puck_infront},
	{"CHECK_1ST_TRAFFIC_LIGHT","MOVE_UNDER_TRAFFIC_LIGHT",cond = traffic_light_green},
	{"CHECK_1ST_TRAFFIC_LIGHT","MOVE_TO_2ND",cond = traffic_light_red}, 
	{"MOVE_TO_2ND","CHECK_2ND_TRAFFIC_LIGHT",wait_sec = 1},--skill von victor

	{"CHECK_2ND_TRAFFIC_LIGHT","MOVE_UNDER_TRAFFIC_LIGHT",cond = traffic_light_green},
	{"CHECK_2ND_TRAFFIC_LIGHT","MOVE_TO_3RD",cond = traffic_light_red},

	{"MOVE_TO_3RD","CHECK_3RD_TRAFFIC_LIGHT",wait_sec = 1},
	{"CHECK_3RD_TRAFFIC_LIGHT","MOVE_UNDER_TRAFFIC_LIGHT",cond = traffic_light_green},
	{"CHECK_3RD_TRAFFIC_LIGHT","FAILED",cond = traffic_light_red,desc= "all deliveryspots appear unuseable"},

	{"MOVE_UNDER_TRAFFIC_LIGHT","MOVE_AWAY_FROM_TRAFFIC_LIGHT",wait_sec = 1},
	{"MOVE_AWAY_FROM_TRAFFIC_LIGHT","FINAL",wait_sec = 1 }



}

function MOVE_AWAY_FROM_TRAFFIC_LIGHT:init()

send_transrot(-0.2,0,0)

end


function MOVE_UNDER_TRAFFIC_LIGHT:init()
send_transrot(0.2,0,0)

end

function MOVE_TO_2ND:init()
trafic_light_green_var = true
send_transrot(0,0.2,0)

end

function MOVE_TO_3RD:init()

send_transrot(0,0.2,0)

end

function CHECK_1ST_TRAFFIC_LIGHT:init()

send_transrot(0,0,0)

end

function CHECK_2ND_TRAFFIC_LIGHT:init()

send_transrot(0,0,0)

end

function CHECK_3RD_TRAFFIC_LIGHT:init()

send_transrot(0,0,0)

end

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end
