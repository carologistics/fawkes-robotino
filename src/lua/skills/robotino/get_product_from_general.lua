
----------------------------------------------------------------------------
--  get_product_fromgeneral.lua
--
--  Created:   Thu Apr 05
--  Copyright  2018 Carsten Stoffels
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
name               = "get_product_from_general"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"get_product_from", "get_product_from_new"}

documentation      = [==[ 
Decides wether the old or the new gripper skill is used, based on the information stored in the host.yaml
The following parameters are just passed to the corresponding sub-skill

Parameters:
      @param place   the name of the MPS (see navgraph)
      @param side    optional the side of the mps, default is input (give "output" to bring to output)
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
      @param slide   optional true if you want to put it on the slide
      @param atmps   optional position at mps shelf, default NO (not at mps at all) : ( NO | LEFT | MIDDLE | RIGHT | CONVEYOR )
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- constants



fsm:define_states{ export_to=_M, closure={navgraph=navgraph,gripper_if=gripper_if},
   {"INIT", JumpState},
   {"OLD_GET_PRODUCT", SkillJumpState, skills={{get_product_from}}, final_to="FINAL", fail_to="FAILED"},
   {"NEW_GET_PRODUCT", SkillJumpState, skills={{get_product_from_new}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "NEW_BRING_PRODUCT", cond=false , desc="host says: new gripper"},  
   {"INIT", "OLD_BRING_PRODUCT", cond=false , desc="host says: old gripper"}, 
   {"INIT", "FAILED", cond=true, desc= "gripper decision not set"}, 
}

function INIT:init()
   --TODO Read in gripper value  -- self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
end

function OLD_GET_PRODUCT:init()

      self.args["get_product_from"].place = self.fsm.vars.place 
      self.args["get_product_from"].side  = self.fsm.vars.side 
      self.args["get_product_from"].shelf = self.fsm.vars.shelf 
      self.args["get_product_from"].slide = self.fsm.vars.slide 
      self.args["get_product_from"].atmps = self.fsm.vars.atmps 
      
end

function NEW_GET_PRODUCT:init()

      self.args["get_product_from_new"].place = self.fsm.vars.place 
      self.args["get_product_from_new"].side  = self.fsm.vars.side 
      self.args["get_product_from_new"].shelf = self.fsm.vars.shelf 
      self.args["get_product_from_new"].slide = self.fsm.vars.slide 
      self.args["get_product_from_new"].atmps = self.fsm.vars.atmps 

end


