
----------------------------------------------------------------------------
--  goto.lua - 
--
--  Created: Thu Aug 14 14:32:47 2008
--  modified by Victor Mataré
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
name               = "goto"
fsm                = SkillHSM:new{name=name, start="SEND_MOVE"}
depends_skills     = nil
depends_interfaces = {
   {v = "navigator", type = "NavigatorInterface" }
    {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[Move to a known (named) location.
@param goto_name Name of the place we want to go to.
@

Available names:
================
M1-M10 = m1-m10:              Machines 1-10,
R1, R2:                       Recycling gates 1 and 2,
T = TEST = Test:              Test station,
EGI = ExpressGoodInsertion:   Express good insertion area,
D1-D3 = Delivery1-Delivery3:  Delivery gates 1-3.
]==]

-- Constants
local margin = 0.605
--0.605 sind sicherheitsabstand etc zusammengerechnet
machine_pos = {
	M1 = {x = 1.68 -margin , y = 1.68 	   , ori = 0}, 
	M2 = {x = 3.9		, y = 1.68+margin , ori = -math.pi/2}, 
	M3 = {x = 0.56 		, y = 2.80-margin , ori =  math.pi/2},
	M4 = {x = 1.68 -margin	, y = 2.80  	   , ori = 0},
	M5 = {x = 2.80 +margin	, y = 2.80  	   , ori =  math.pi},
	M6 = {x = 3.92		, y = 2.80+margin , ori = -math.pi/2},
	M7 = {x = 5.04 +margin , y = 2.80 	   , ori =  math.pi},
	M8 = {x = 1.68		, y = 3.92-margin , ori =  math.pi/2},
	M9 = {x = 3.92		, y = 3.92+margin , ori = -math.pi/2},
	M10 = {x = 5.04-margin	, y = 3.92 	   , ori = 0},
	m1 = M1, m2 = M2, m3 = M3, m4 = M4, m5 = M5, m6 = M6, m7 = M7, m8 = M8, m9 = M9,
	m10 = M10,
	

	R1 = {x = 0.20 + margin, y = 0.20+margin ,ori = -math.pi*(3/4)},
	R2 = {x = 5.40- margin, y = 5.40-margin ,ori = math.pi/4 },

	T = {x = 5.40+margin, y = 0.20+margin ,ori = -math.pi/4},
	TEST = T, Test = T,

	EGI = {x = 3.60 + margin, y = 5.35 ,ori =math.pi},
	ExpressGoodInsertion = EGI

	D1 = {x = 3.15, y = 0.26+margin ,ori =-math.pi/2},
	D2 = {x = 2.80, y = 0.26+margin ,ori = -math.pi/2},
	D3 = {x = 2.45, y = 0.26+margin ,ori = -math.pi/2},
	Delivery1 = D1, Delivery2 = D2, Delivery3 = D3
}

--Input_Store = {x=0,y=0,ori=0}

-- Initialize as skill module
skillenv.skill_module(...)


fsm:add_transitions{
	closure={motor=motor},
	{"SEND_MOVE", "FINAL", skill = move_to_global,fail_to = "FAILED"}

}


function SEND_MOVE:INIT()
	local name = self.fsm.vars.goto_name
	self.fsm.args.global_goto = {
		x = machine_pos[name].x,
		y = machine_pos[name].y,
		ori = machine_pos[name].ori
	}
end

