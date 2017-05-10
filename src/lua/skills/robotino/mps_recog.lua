----------------------------------------------------------------------------
--  check_tag.lua -check if tag in front is the given tag_id 
--
--  Copyright 2015 The Carologistics Team
--
--  Author : Carsten Stoffels, Daniel Habering,Sebastian Schönitz
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

-- Initialize modul
module(..., skillenv.module_init)

-- Crucial skill information
name               = "mps_recog"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {}
depends_interfaces = {
	{v = "RecogIf", type = "MPSRecognitionInterface" ,id="/tag-vision/0"},
}

documentation      = [==[
This skill should check if the current picture of the RealSense can be identified by a machine using the mps_recog plugin 

]==]


-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   closure={tag_visible=tag_visible},
   {"INIT", JumpState},
   {"CHECK_MPS", JumpState},
}

fsm:add_transitions{
   closure={input_ok=input_ok},
   --{"INIT", "FAILED", cond="not vars.tag_id", desc="No tag_id given!"},
   {"INIT", "FAILED", cond=no_tag_vision, desc="Tag vision disabled"},
   {"INIT", "CHECK_TAG", cond=true},
   {"CHECK_TAG", "FINAL", cond=tag_visible, desc="The given tag_id is visible"},
   {"CHECK_TAG", "FAILED", timeout=1, desc="The given tag_id is not visible"},
}


-- TODO : finish

