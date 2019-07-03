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
@param mps: Name of the MPS e.g C-CS1 C-DS
@param side: input output
]==]

-- Crucial skill information
name               = "take_picture"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"markerless_mps_align"}
depends_interfaces = {
   {v = "if_picture_taker", type = "PictureTakerInterface", id="PictureTaker"},
}

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"ALIGN", SkillJumpState, skills={{markerless_mps_align}}, final_to="TAKE_PICTURE",fail_to="FAILED"},
   {"TAKE_PICTURE", JumpState}
}

fsm:add_transitions{
   {"INIT", "ALIGN", cond=true},
   {"TAKE_PICTURE", "ALIGN", cond="self.fsm.vars.index==3"},
   {"TAKE_PICTURE", "FINAL", cond=true},
}

function INIT:init()
  self.fsm.vars.index=0
  self.fsm.vars.pos_y = {-0.3,0.0,0.3}
  self.fsm.vars.pos_x = {0.5, 0.5, 0.5}
  self.fsm.vars.ori = {math.atan2(self.fsm.vars.pos_y[0],self.fsm.vars.pos_x[0]),
			math.atan2(self.fsm.vars.pos_y[1],self.fsm.vars.pos_x[1]),
			math.atan2(self.fsm.vars.pos_y[2],self.fsm.vars.pos_x[2])
		      }
end

function ALIGN:init()
  self.args["markerless_mps_align"].x = self.fsm.vars.pos_x[self.fsm.vars.index] 
  self.args["markerless_mps_align"].y = self.fsm.vars.pos_y[self.fsm.vars.index]
  self.args["markerless_mps_align"].ori = self.fsm.vars.ori[self.fsm.vars.index]
end

function TAKE_PICTURE:init()
   self.fsm.vars.index=self.fsm.vars.index+1	
   print_info("take_picture: mps: %s side: %s", self.fsm.vars.mps, self.fsm.vars.side)

   if if_picture_taker:has_writer() and self.fsm.vars.mps  and self.fsm.vars.side then
     local msg = if_picture_taker.TakePictureMessage:new(self.fsm.vars.mps,self.fsm.vars.side)
     if_picture_taker:msgq_enqueue_copy(msg)
   end
end
