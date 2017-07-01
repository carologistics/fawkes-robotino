
----------------------------------------------------------------------------
--  drive_to_zones.lua
--
--  Created: Sat Jul 01 13:27:47 2017
--  Copyright 2017 Carsten Stoffels
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
name               = "drive_to_zone"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_recog", "goto"}
depends_interfaces = {
   {v = "speechsynth", type = "SpeechSynthInterface", id = "Flite"}
}

documentation      = [==[ drive_to_zone

                          This skill does:
                                drives to the corners of the zone and tries to recognize the mps                  

]==]


-- Initialize as skill module
skillenv.skill_module(_M)

-- CONSTANTS
MPS_LENGTH = 0.7
MPS_WIDTH = 0.35
BOT_RADIUS = 0.46/2
START_DIST_TO_MPS = 0.15+BOT_RADIUS

START_POS={-MPS_WIDTH/2-START_DIST_TO_MPS,0.,0.}


function int_div(a,b)
   return (a-a%b)/b
end

-- the two arguments are of both of the form (x,y,ori) and assumed relative in the same coordinate system
-- the result will be in the same format, and represent transform_pose in the coordinate system of base_pose
function transform_pose(transform_pose,base_pose)
   local delsx=transform_pose[1]-base_pose[1]
   local delsy=transform_pose[2]-base_pose[2]
   return {math.cos(base_pose[3])*delsx+math.sin(base_pose[3])*delsy,
          -math.sin(base_pose[3])*delsx+math.cos(base_pose[3])*delsy,
           transform_pose[3]-base_pose[3]}
end

function calc_xy_coordinates(self)
  if self.fsm.vars.zoneID<1 or self.fsm.vars.zoneID>24 then
     return false
  end
  local ylen=1.5
  local xlen=2.
  local botradius=0.4
  local id=self.fsm.vars.zoneID-1
  self.fsm.vars.y1=(id%4)*ylen
  self.fsm.vars.y2=self.fsm.vars.y1+ylen
  if id>=12 then
      self.fsm.vars.x2=-xlen*int_div(id-12,4)
      self.fsm.vars.x1=self.fsm.vars.x2-xlen
   else
      self.fsm.vars.x1=xlen*int_div(id,4)
      self.fsm.vars.x2=self.fsm.vars.x1+xlen
   end

   if self.fsm.vars.x1==-6. then
      self.fsm.vars.x1=-6.+botradius
   end
   if self.fsm.vars.x2==6. then
      self.fsm.vars.x2=6.-botradius
   end
   if self.fsm.vars.y1==0. then
      self.fsm.vars.y1=botradius
   end
   if self.fsm.vars.y2==6. then
      self.fsm.vars.y2=6.-botradius
   end

   self.fsm.vars.corner=3
   return true
end


fsm:define_states{ export_to=_M,
   {"INIT", JumpState},
   {"DRIVE_TO_START", SkillJumpState, skills={{goto}}, final_to="RECOGNIZE_MPS", fail_to="FAILED"},
   {"RECOGNIZE_MPS", SkillJumpState, skills={{mps_recog}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "DRIVE_TO_START", cond=calc_xy_coordinates},
   {"INIT", "FAILED", cond=true, desc="zoneID is not between 1 and 24"},
}



function DRIVE_TO_START:init()
   self.args["goto"].x = (self.fsm.vars.x1+self.fsm.vars.x2)/2
   self.args["goto"].y = (self.fsm.vars.y1+self.fsm.vars.y2)/2
   self.args["goto"].region_trans = 2
end

