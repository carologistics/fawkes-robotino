
----------------------------------------------------------------------------
--  bring_product_to.lua
--
--  Created: Sat Apr 18
--  Copyright  2015  Johannes Rothe
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
name               = "bring_product_to"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"mps_align", "product_put", "drive_to","shelf_put","slide_put","conveyor_align","motor_move"}
depends_interfaces = {
}

documentation      = [==[ 
aligns to a machine and puts a product on the conveyor.
It will get the offsets and the align distance for the machine 
from the navgraph

Parameters:
      @param place   the name of the MPS (see navgraph)
      @param side    optional the side of the mps, default is input (give "output" to bring to output)
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
      @param slide   optional true if you want to put it on the slide
      @param atmps   optional position at mps shelf, default NO (not at mps at all) : ( NO | LEFT | MIDDLE | RIGHT | CONVEYOR )
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- Constants

function already_at_mps(self)
   return not (self.fsm.vars.atmps=="NO" or self.fsm.vars.atmps==nil)
end

function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph},
   {"INIT", JumpState},
   {"DRIVE_TO", SkillJumpState, skills={{drive_to}}, final_to="MPS_ALIGN", fail_to="FAILED"},
   {"MPS_ALIGN", SkillJumpState, skills={{mps_align}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"RE_MPS_ALIGN", SkillJumpState, skills={{motor_move}}, final_to="DECIDE_ENDSKILL", fail_to="FAILED"},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="DECIDE_ENDSKILL", fail_to="DECIDE_ENDSKILL"}, --TODO proper handling
   {"DECIDE_ENDSKILL", JumpState},
   {"SKILL_SHELF_PUT", SkillJumpState, skills={{shelf_put}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_SLIDE_PUT", SkillJumpState, skills={{slide_put}}, final_to="FINAL", fail_to="FAILED"},
   {"SKILL_PRODUCT_PUT", SkillJumpState, skills={{product_put}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "MPS_ALIGN", cond=already_at_conveyor, desc="At mps, skip drive_to"},
   {"INIT", "RE_MPS_ALIGN", cond=already_at_mps, desc="At mps, skip DRIVE and ALIGN"},
   {"INIT", "DRIVE_TO", cond=true, desc="Everything OK"},
   {"DECIDE_ENDSKILL", "MPS_ALIGN", cond="vars.counter <= 1", desc="Put on shelf"},
   {"DECIDE_ENDSKILL", "SKILL_SHELF_PUT", cond="vars.shelf", desc="Put on shelf"},
   {"DECIDE_ENDSKILL", "SKILL_SLIDE_PUT", cond="vars.slide", desc="Put on slide"},
   {"DECIDE_ENDSKILL", "SKILL_PRODUCT_PUT", cond=true, desc="Put on conveyor"}
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
   self.fsm.vars.counter = 0
end

function DRIVE_TO:init()
   if self.fsm.vars.side == "output" or self.fsm.vars.side == "OUTPUT" then
      self.args["drive_to"] = {place = self.fsm.vars.place .. "-O"}
   else
      self.args["drive_to"] = {place = self.fsm.vars.place .. "-I"}
   end
end

function MPS_ALIGN:init()
   self.fsm.vars.counter = self.fsm.vars.counter + 1
   -- align in front of the conveyor belt
   self.args["mps_align"] = {x = navgraph:node(self.fsm.vars.place):property_as_float("align_distance")}
   if self.fsm.vars.side == "output" or self.fsm.vars.side == "OUTPUT"  then
      if navgraph:node(self.fsm.vars.place):has_property("output_offset_y") then
         self.args["mps_align"].y = navgraph:node(self.fsm.vars.place):property_as_float("output_offset_y")
      else
         self.args["mps_align"].y = 0
      end
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
   else
      if navgraph:node(self.fsm.vars.place):has_property("input_offset_y") then
         self.args["mps_align"].y = navgraph:node(self.fsm.vars.place):property_as_float("input_offset_y")
      else
         self.args["mps_align"].y = 0
      end
      self.args["mps_align"].tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
   end
   self.args["mps_align"].ori = 0
end

function RE_MPS_ALIGN:init()
   self.fsm.vars.counter = self.fsm.vars.counter + 1
   local shelf_to_conveyor = 0.09 --TODO measure both values
   local shelf_distance = 0.09
   if self.fsm.vars.atmps == "LEFT" then
      dest_y = shelf_to_conveyor
   elseif self.fsm.vars.atmps == "MIDDLE" then
      dest_y = shelf_to_conveyor + shelf_distance
   elseif self.fsm.vars.atmps == "RIGHT" then
      dest_y = shelf_to_conveyor + 2*shelf_distance
   else
      dest_y = 0
      self.fsm:set_error("no shelf side set")
      self.fsm.vars.error = true
   end
   
   self.args["motor_move"] =
			{ y = dest_y,
				vel_trans = 0.2,
				tolerance = { x=0.002, y=0.002, ori=0.01 }
			}
end

function SKILL_PRODUCT_PUT:init()
   if self.fsm.vars.side == "output" or self.fsm.vars.side == "OUTPUT" then
      if navgraph:node(self.fsm.vars.place):has_property("output_offset_x") then
         self.args["product_put"].offset_x = navgraph:node(self.fsm.vars.place):property_as_float("output_offset_x")
      else
         self.args["product_put"].offset_x = 0 
      end 
   else
      if navgraph:node(self.fsm.vars.place):has_property("input_offset_x") then
         self.args["product_put"].offset_x = navgraph:node(self.fsm.vars.place):property_as_float("input_offset_x")
      else
         self.args["product_put"].offset_x = 0 
      end 
   end 
end

function SKILL_SHELF_PUT:init()
   -- Just hand through the Shelf position
   self.args["shelf_put"].slot = self.fsm.vars.shelf
end
