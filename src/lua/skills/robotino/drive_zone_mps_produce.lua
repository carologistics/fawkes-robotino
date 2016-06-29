
----------------------------------------------------------------------------
--  drive_zone_mps_produce.lua
--
--  Created: Thu Jun 17 16:47:47 2016
--  Copyright  2016  David Schmidt
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
name               = "drive_zone_mps_produce"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"goto", "tagless_mps_align", "shelf_find_pick_put", "product_put", "product_pick", "motor_move"}
depends_interfaces = {}

documentation      = [==[ drive_zone_mps_recognize

                          This skill does:
                             drives to the corners of the zone and tries to recognize the mps

                          @param zoneID     id of the zone
]==]


-- Initialize as skill module
skillenv.skill_module(_M)

-- CONSTANTS
Y_OFFSET_CONVEYOR = 0.025

MPS_LENGTH = 0.7
MPS_WIDTH = 0.35
BOT_RADIUS = 0.46/2
START_DIST_TO_MPS = 0.15+BOT_RADIUS

START_POS={-MPS_WIDTH/2-START_DIST_TO_MPS,0.,0.}

-- Variables
local tfm = require("tf_module")

--Constants
local MACHINE = "C-CS1"


function int_div(a,b)
   return (a-a%b)/b
end

-- the two arguments are both of the form (x,y,ori) and assumed relative in the same coordinate system
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
   {"INSTRUCT_MACHINE_RETRIEVE", JumpState},
   {"KILLALL_RETRIEVE", JumpState},
   {"DRIVE_TO_START", SkillJumpState, skills={{goto}}, final_to="DRIVE_CORNER", fail_to="FAILED"},
   {"DRIVE_CORNER", SkillJumpState, skills={{goto}}, final_to="CHECK_FOR_LASERLINE", fail_to="FAILED"},
   {"LOWER_ACCURACY", SkillJumpState, skills={{goto}}, final_to="CHECK_FOR_LASERLINE", fail_to="FAILED"},
   {"CHECK_FOR_LASERLINE", SkillJumpState, skills={{tagless_mps_align}}, final_to="GET_PUCK", fail_to="DRIVE_CORNER"},
   {"GET_PUCK", SkillJumpState, skills={{shelf_find_pick_put}}, final_to="PUT_PUCK", fail_to="DRIVE_TO_OTHER_SIDE"},
   {"PUT_PUCK", SkillJumpState, skills={{product_put}}, final_to="DRIVE_TO_OUTPUT", fail_to="FAILED"},
   {"DRIVE_TO_OUTPUT", SkillJumpState, skills={{goto}}, final_to="ALIGN_LASERLINE", fail_to="FAILED"},
   {"ALIGN_LASERLINE", SkillJumpState, skills={{tagless_mps_align}}, final_to="ALIGN_CONVEYOR", fail_to="FAILED"},
   {"ALIGN_CONVEYOR", SkillJumpState, skills={{motor_move}}, final_to="RETRIEVE_PUCK", fail_to="FAILED"},
   {"RETRIEVE_PUCK", SkillJumpState, skills={{product_pick}}, final_to="INSTRUCT_MACHINE_MOUNT", fail_to="FAILED"},
   {"INSTRUCT_MACHINE_MOUNT", JumpState},
   {"KILLALL_MOUNT", JumpState},
   {"DRIVE_TO_OTHER_SIDE", SkillJumpState, skills={{goto}}, final_to="CHECK_FOR_LASERLINE", fail_to="FAILED"},
   {"DRIVE_TO_INPUT", SkillJumpState, skills={{goto}}, final_to="ALIGN_LASERLINE_2", fail_to="FAILED"},
   {"ALIGN_LASERLINE_2", SkillJumpState, skills={{tagless_mps_align}}, final_to="ALIGN_CONVEYOR_2", fail_to="FAILED"},
   {"ALIGN_CONVEYOR_2", SkillJumpState, skills={{motor_move}}, final_to="PUT_PUCK_2", fail_to="FAILED"},
   {"PUT_PUCK_2", SkillJumpState, skills={{product_put}}, final_to="DRIVE_TO_OUTPUT_2", fail_to="FAILED"},
   {"DRIVE_TO_OUTPUT_2", SkillJumpState, skills={{goto}}, final_to="ALIGN_LASERLINE_3", fail_to="FAILED"},
   {"ALIGN_LASERLINE_3", SkillJumpState, skills={{tagless_mps_align}}, final_to="ALIGN_CONVEYOR_3", fail_to="FAILED"},
   {"ALIGN_CONVEYOR_3", SkillJumpState, skills={{motor_move}}, final_to="RETRIEVE_PUCK_2", fail_to="FAILED"},
   {"RETRIEVE_PUCK_2", SkillJumpState, skills={{product_pick}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "INSTRUCT_MACHINE_RETRIEVE", cond=calc_xy_coordinates},
   {"INIT", "FAILED", cond=true, desc="zoneID is not between 1 and 24"},
   {"INSTRUCT_MACHINE_RETRIEVE", "KILLALL_RETRIEVE", timeout=3},
   {"KILLALL_RETRIEVE", "DRIVE_TO_START", cond=true},
   {"DRIVE_CORNER", "LOWER_ACCURACY", timeout=30},
   {"LOWER_ACCURACY", "DRIVE_CORNER", timeout=30},
   {"INSTRUCT_MACHINE_MOUNT", "KILLALL_MOUNT", timeout=3},
   {"KILLALL_MOUNT", "DRIVE_TO_OTHER_SIDE", cond=true},
}

function INIT:init()
   self.fsm.vars.machine = self.fsm.vars.machine or MACHINE
   self.fsm.vars.poi_idx = 0
   self.fsm.vars.data_taken = 0
   os.execute("../../llsf-refbox/bin/rcll-set-machine-state " .. self.fsm.vars.machine .. "  RESET &")
end

function DRIVE_TO_START:init()
   self.args["goto"].x = (self.fsm.vars.x1+self.fsm.vars.x2)/2
   self.args["goto"].y = (self.fsm.vars.y1+self.fsm.vars.y2)/2
   self.args["goto"].region_trans = 2
end

function DRIVE_CORNER:init()
   self.fsm.vars.corner = (self.fsm.vars.corner+1)%4
   self.args["goto"].ori = math.pi/4 + self.fsm.vars.corner*math.pi/2
   self.fsm.vars.accuracy = 0.2
   self.args["goto"].region_trans = self.fsm.vars.accuracy
   if 0==self.fsm.vars.corner or 1==self.fsm.vars.corner then
      self.args["goto"].y = self.fsm.vars.y1
   else
      self.args["goto"].y = self.fsm.vars.y2
   end
   if 0==self.fsm.vars.corner or 3==self.fsm.vars.corner then
      self.args["goto"].x = self.fsm.vars.x1
   else
      self.args["goto"].x = self.fsm.vars.x2
   end
end

function LOWER_ACCURACY:init()
   self.args["goto"].ori = math.pi/4 + self.fsm.vars.corner*math.pi/2
   self.fsm.vars.accuracy = 0.1+self.fsm.vars.accuracy
   self.args["goto"].region_trans = 1.
   if 0==self.fsm.vars.corner or 1==self.fsm.vars.corner then
      self.args["goto"].y = self.fsm.vars.y1
   else
      self.args["goto"].y = self.fsm.vars.y2
   end
   if 0==self.fsm.vars.corner or 3==self.fsm.vars.corner then
      self.args["goto"].x = self.fsm.vars.x1
   else
      self.args["goto"].x = self.fsm.vars.x2
   end
end

function CHECK_FOR_LASERLINE:init()
   self.args["tagless_mps_align"].ori = math.pi/2
   self.args["tagless_mps_align"].x1 = self.fsm.vars.x1
   self.args["tagless_mps_align"].x2 = self.fsm.vars.x2
   self.args["tagless_mps_align"].y1 = self.fsm.vars.y1
   self.args["tagless_mps_align"].y2 = self.fsm.vars.y2
end

function GET_PUCK:init()
   local mps_pose = transform_pose({0.,0.,0.},START_POS)
   self.fsm.vars.mps_pose = tfm.transform({x=mps_pose[1], y=mps_pose[2], ori=mps_pose[3]}, "/base_link", "/map")
   self.fsm.vars.mps_pose = {self.fsm.vars.mps_pose.x,self.fsm.vars.mps_pose.y,self.fsm.vars.mps_pose.ori}
end

function INSTRUCT_MACHINE_RETRIEVE:init()
   os.execute("../../llsf-refbox/bin/rcll-prepare-machine Carologistics " .. self.fsm.vars.machine .. "  RETRIEVE_CAP &")
end

function INSTRUCT_MACHINE_MOUNT:init()
   os.execute("../../llsf-refbox/bin/rcll-prepare-machine Carologistics " .. self.fsm.vars.machine .. "  MOUNT_CAP &")
end

function KILLALL_RETRIEVE:init()
   os.execute("killall -9 rcll-prepare-machine")
end

function KILLALL_MOUNT:init()
   os.execute("killall -9 rcll-prepare-machine")
end

function DRIVE_TO_OUTPUT:init()
   local outputpose = transform_pose({-START_POS[1],-START_POS[2],START_POS[3]+math.pi},transform_pose({0.,0.,0.},self.fsm.vars.mps_pose))
   self.args["goto"].x = outputpose[1]
   self.args["goto"].y = outputpose[2]
   self.args["goto"].ori = outputpose[3]
end

function DRIVE_TO_OUTPUT_2:init()
   local outputpose = transform_pose({-START_POS[1],-START_POS[2],START_POS[3]+math.pi},transform_pose({0.,0.,0.},self.fsm.vars.mps_pose))
   self.args["goto"].x = outputpose[1]
   self.args["goto"].y = outputpose[2]
   self.args["goto"].ori = outputpose[3]
end

function DRIVE_TO_INPUT:init()
   local inputpose = transform_pose({START_POS[1],START_POS[2],START_POS[3]},transform_pose({0.,0.,0.},self.fsm.vars.mps_pose))
   self.args["goto"].x = inputpose[1]
   self.args["goto"].y = inputpose[2]
   self.args["goto"].ori = inputpose[3]
end

function ALIGN_LASERLINE:init()
   self.args["tagless_mps_align"].ori = math.pi/2
   self.args["tagless_mps_align"].x1 = self.fsm.vars.x1
   self.args["tagless_mps_align"].x2 = self.fsm.vars.x2
   self.args["tagless_mps_align"].y1 = self.fsm.vars.y1
   self.args["tagless_mps_align"].y2 = self.fsm.vars.y2
end

function ALIGN_LASERLINE_2:init()
   self.args["tagless_mps_align"].ori = math.pi/2
   self.args["tagless_mps_align"].x1 = self.fsm.vars.x1
   self.args["tagless_mps_align"].x2 = self.fsm.vars.x2
   self.args["tagless_mps_align"].y1 = self.fsm.vars.y1
   self.args["tagless_mps_align"].y2 = self.fsm.vars.y2
end

function ALIGN_LASERLINE_3:init()
   self.args["tagless_mps_align"].ori = math.pi/2
   self.args["tagless_mps_align"].x1 = self.fsm.vars.x1
   self.args["tagless_mps_align"].x2 = self.fsm.vars.x2
   self.args["tagless_mps_align"].y1 = self.fsm.vars.y1
   self.args["tagless_mps_align"].y2 = self.fsm.vars.y2
end

function ALIGN_CONVEYOR:init()
   self.args["motor_move"] = {y = -Y_OFFSET_CONVEYOR, vel_trans = 0.2, tolerance = { x=0.002, y=0.002, ori=0.01 }}
end

function ALIGN_CONVEYOR_2:init()
   self.args["motor_move"] = {y = +Y_OFFSET_CONVEYOR, vel_trans = 0.2, tolerance = { x=0.002, y=0.002, ori=0.01 }}
end

function ALIGN_CONVEYOR_3:init()
   self.args["motor_move"] = {y = -Y_OFFSET_CONVEYOR, vel_trans = 0.2, tolerance = { x=0.002, y=0.002, ori=0.01 }}
end

function DRIVE_TO_OTHER_SIDE:init()
   local outputpose = transform_pose({-START_POS[1],-START_POS[2],START_POS[3]+math.pi},transform_pose({0.,0.,0.},self.fsm.vars.mps_pose))
   self.args["goto"].x = outputpose[1]
   self.args["goto"].y = outputpose[2]
   self.args["goto"].ori = outputpose[3]
end
