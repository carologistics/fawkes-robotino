-- recognize_mps.lua

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
documentation = [==[Tries to recognize a MPS
]==]

-- Crucial skill information
name               = "recognize_mps"
fsm                = SkillHSM:new{name=name, start="PREPARE_TF", debug=false}
depends_skills     = {"markerless_mps_align"}
depends_interfaces = {
   {v = "if_tensorflow", type = "PictureTakerInterface", id="PictureTaker"},
}

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"PREPARE_TF", JumpState},
   {"INIT", JumpState},
   {"ALIGN", SkillJumpState, skills={{markerless_mps_align}}, final_to="SHORT_WAIT",fail_to="FAILED"},
   {"SHORT_WAIT", JumpState},
   {"TAKE_PICTURE", JumpState}
}

fsm:add_transitions{
   {"PREPARE_TF",
   {"INIT", "ALIGN", cond=true},
   {"SHORT_WAIT", "TAKE_PICTURE", timeout=0.5},
   {"TAKE_PICTURE", "ALIGN", cond="self.fsm.vars.index<=3"},
   {"TAKE_PICTURE", "FINAL", cond=true},
}

function INIT:init()
  self.fsm.vars.index=1
  self.fsm.vars.pos_y = {-0.35,0.0,0.35}
  self.fsm.vars.pos_x = {0.6, 0.6, 0.6}
  self.fsm.vars.ori = {-math.atan2(self.fsm.vars.pos_y[1],self.fsm.vars.pos_x[1]),
		-	math.atan2(self.fsm.vars.pos_y[2],self.fsm.vars.pos_x[2]),
		-	math.atan2(self.fsm.vars.pos_y[3],self.fsm.vars.pos_x[3])
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
