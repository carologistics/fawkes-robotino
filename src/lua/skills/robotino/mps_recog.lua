----------------------------------------------------------------------------
--  check_tag.lua -check if tag in front is the given tag_id 
--
--  Copyright 2015 The Carologistics Team
--
--  Author : Carsten Stoffels, Daniel Habering
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
}

-- TODO : finish

