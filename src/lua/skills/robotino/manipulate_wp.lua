
----------------------------------------------------------------------------
--  manipulate_wp.lua
--
--  Created: Wed Nov 17
--  Copyright  2021  Matteo Tschesche
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
name               = "manipulate_wp"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"goto","motor_move"}
depends_interfaces = {
   {v = "object_tracking_if", type = "ObjectTrackingInterface", id="object-tracking"},
}

documentation      = [==[
Uses visual servoing to fulfill every workpiece manipulation task.

Parameters:
      @param type    the type of the target object: (WORKPIECE | CONVEYOR | SLIDE)
      @param place   the name of the MPS (see navgraph)
      @param side    the side of the mps (INPUT_CONVEYOR | OUTPUT_CONVEYOR | SHELF_LEFT | SHELF_MIDDLE | SHELF_RIGHT | SLIDE)
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function found_object()
  return false
end

fsm:define_states{ export_to=_M, closure={found_object=found_object},
   {"INIT",                  JumpState},
   {"SEARCH",                SkillJumpState, skills={{goto}},          final_to="MOVE_BASE_AND_GRIPPER", fail_to="FAILED"},
   {"MOVE_BASE_AND_GRIPPER", SkillJumpState, skills={{motor_move}},    final_to="FINE_TUNE_GRIPPER", fail_to="SEARCH"},
   {"FINE_TUNE_GRIPPER",     JumpState},
}

fsm:add_transitions{
   {"INIT", "SEARCH", cond=true},
   {"FINE_TUNE_GRIPPER", "FINAL", cond="found_object()", desc="navgraph not available"},
}

function INIT:init()
  local msg = object_tracking_if.StartTrackingMessage:new(object_tracking_if.WORKPIECE, object_tracking_if.C_CS1, object_tracking_if.SHELF_LEFT)
  object_tracking_if:msgq_enqueue_copy(msg)
end

function SEARCH:init()
  self.args["goto"] = {x = 0.925032, y = 2.875027, ori = 4.711593, end_early = true}
end

function MOVE_BASE_AND_GRIPPER:init()
  self.args["motor_move"] = {x = object_tracking_if:base_frame(0),
                             y = object_tracking_if:base_frame(1),
                             ori = object_tracking_if:base_frame(5),
                             frame = "map",
                             visual_servoing = true}
end
