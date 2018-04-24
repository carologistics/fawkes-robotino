
----------------------------------------------------------------------------
--  bring_product_to_general.lua
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
name               = "bring_product_to_general"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"bring_product_to", "bring_product_to_new"}

documentation      = [==[
Decides wether the old or the new gripper skill is used, based on the information stored in the host.yaml
The following parameters are just passed to the corresponding sub-skill

Parameters:
      @param gripper name of the mounted gripper: (NEW_GRIPPER | OLD_GRIPPER)
      @param place   the name of the MPS (see navgraph)
      @param side    optional the side of the mps, default is input (give "output" to bring to output)
      @param shelf   optional position on shelf: ( LEFT | MIDDLE | RIGHT )
      @param slide   optional true if you want to put it on the slide
      @param atmps   optional position at mps shelf, default NO (not at mps at all) : ( NO | LEFT | MIDDLE | RIGHT | CONVEYOR )
]==]
-- Initialize as skill module
skillenv.skill_module(_M)
-- constants


function new_gripper(self)
     if self.fsm.vars.gripper == NEW_GRIPPER then
       return true
     end
     return false
end

function old_gripper(self)
     if self.fsm.vars.gripper == OLD_GRIPPER then
       return true
     end
     return false
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph,gripper_if=gripper_if},
   {"INIT", JumpState},
   {"OLD_BRING_PRODUCT", SkillJumpState, skills={{bring_product_to}}, final_to="FINAL", fail_to="FAILED"},
   {"NEW_BRING_PRODUCT", SkillJumpState, skills={{bring_product_to_new}}, final_to="FINAL", fail_to="FAILED"},
}

fsm:add_transitions{
   {"INIT", "NEW_BRING_PRODUCT", cond=false , desc="host says: new gripper"},  
   {"INIT", "OLD_BRING_PRODUCT", cond=false , desc="host says: old gripper"}, 
   {"INIT", "FAILED", cond=true, desc="No gripper decision set"},
}

function INIT:init()
   --TODO Read in gripper value 
end

function OLD_BRING_PRODUCT:init()

      self.args["bring_product_to"].place = self.fsm.vars.place 
      self.args["bring_product_to"].side  = self.fsm.vars.side 
      self.args["bring_product_to"].shelf = self.fsm.vars.shelf 
      self.args["bring_product_to"].slide = self.fsm.vars.slide 
      self.args["bring_product_to"].atmps = self.fsm.vars.atmps 
      
end

function NEW_BRING_PRODUCT:init()

      self.args["bring_product_to_new"].place = self.fsm.vars.place 
      self.args["bring_product_to_new"].side  = self.fsm.vars.side 
      self.args["bring_product_to_new"].shelf = self.fsm.vars.shelf 
      self.args["bring_product_to_new"].slide = self.fsm.vars.slide 
      self.args["bring_product_to_new"].atmps = self.fsm.vars.atmps 

end


