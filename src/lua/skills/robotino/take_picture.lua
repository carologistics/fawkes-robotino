-- take_picture.lua

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

module(..., skillenv.module_init)
documentation = [==[Take a Picture of the Realsense Camera.
]==]

-- Crucial skill information
name               = "take_picture"
fsm                = SkillHSM:new{name=name, start="TAKE_PICTURE", debug=false}
depends_skills     = {}
depends_interfaces = {
   {v = "if_picture_taker", type = "PictureTakerInterface", id="PictureTaker"},
}

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"TAKE_PICTURE", JumpState}
}

fsm:add_transitions{
   {"TAKE_PICTURE", "FINAL", cond=true},
}

function TAKE_PICTURE:init()

   print_info("Taking picture")

   if if_picture_taker:has_writer() then
     local msg = if_picture_taker.TakePictureMessage:new()
     if_picture_taker:msgq_enqueue_copy(msg)
   end
end
