----------------------------------------------------------------------------
--  conveyor_align.lua - align orthogonal to the conveyor using the conveyor vision
--
--  Copyright  2013 The Carologistics Team
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
name               = "conveyor_align_rec"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"motor_move", "mps_align", "conveyor_align"}
depends_interfaces = {
	 {v="conveyor_pose_if", type="ConveyorPoseInterface", id="conveyor_pose/status"},
 }

documentation      = [==[aligns the robot to the mps and to the conveyor, while recording given topics in a loop
Parameters:
       @param mps_id bagpath topics
Example:
       conveyor_align_rec{mps_id=17, bagpath='/home/robotino/bagfiles/', topics='/fawkes_pcls/trimmed /fawkes_pcls/model'}

]==]


-- Initialize as skill module

skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   closure={},
   {"INIT", JumpState},
   {"MPS_ALIGN", SkillJumpState, skills={ {mps_align} }, final_to="REC_ALIGN", fail_to="FAILED"},
   {"REC_ALIGN", SkillJumpState, skills={ {conveyor_align} }, final_to="MOVE_BACK", fail_to="FAILED"},
   {"MOVE_BACK", SkillJumpState, skills={ {motor_move}}, final_to="INIT", fail_to="FAILED" },
}

fsm:add_transitions{
   {"INIT", "MPS_ALIGN", cond=true},
}

function INIT:init()
   --// TODO: check if something to do
end

function MPS_ALIGN:init()
   --// configure alignment to MPS
   mps_id = self.fsm.vars.mps_id
   self.args["mps_align"].tag_id = mps_id
   self.args["mps_align"].x = 0.4
end

function REC_ALIGN:init()
   --// start rosbag record
   bagpath = self.fsm.vars.bagpath
   topics = self.fsm.vars.topics
   pidstr =   io.popen('rosbag record ' .. topics .. ' ' .. ' -o ' .. bagpath .. ' __name:=recalign &> /dev/null & echo $!')

   --// configure alignment skill
   self.args["conveyor_align"].mps_type = conveyor_pose_if.CAP_STATION
   self.args["conveyor_align"].mps_target = conveyor_pose_if.INPUT_CONVEYOR
end

function REC_ALIGN:exit()
   --// stop rosbag record 
   io.popen('rosnode kill /recalign')
end

function MOVE_BACK:init()
   --// call skill motor_move with random target in range 
   self.args["motor_move"].x = -math.random(100, 400) / 1000.
   self.args["motor_move"].y = math.random(100, 400) / 1000. * ((math.random(0,2))-1.)
   self.args["motor_move"].ori = math.random(1, 120) / 100.
end

