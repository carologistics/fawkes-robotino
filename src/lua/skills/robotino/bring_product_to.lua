
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"product_put", "drive_to_machine_point", "conveyor_align"}
depends_interfaces = {
   {v = "laserline_switch", type = "SwitchInterface", id="laser-lines"},
   {v = "robotino_sensor", type = "RobotinoSensorInterface", id="Robotino"} -- Interface to read I/O ports
 }

documentation      = [==[ 
aligns to a machine and puts a product on the conveyor.
It will get the offsets and the align distance for the machine 
from the navgraph

Parameters:
      @param place   the name of the MPS (see navgraph)
      @param side    optional the side of the mps, default is input (give "output" to bring to output)
      @param slide   optional true if you want to put it on the slide
      @param atmps   optional position at mps shelf, default NO (not at mps at all) : ( NO | LEFT | MIDDLE | RIGHT | CONVEYOR )
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
-- If this matches the desired x distance of conveyor align, conveyor align has the chance
-- of not needing to move at all.
local X_AT_MPS = 0.26


-- function to evaluate sensor data
function is_grabbed()
   return true
-- if not robotino_sensor:has_writer() then
--   print_warn("No robotino sensor writer to check sensor")
--   return true
-- end
-- if robotino_sensor:is_digital_in(0) == false and robotino_sensor:is_digital_in(1) == true then -- white cable on DI1 and black on DI2
--    return true
-- else
--   return false
-- end
end



function already_at_mps(self)
   return not (self.fsm.vars.atmps=="NO" or self.fsm.vars.atmps==nil)
end

function already_at_conveyor(self)
   return (self.fsm.vars.atmps == "CONVEYOR")
end

fsm:define_states{ export_to=_M, closure={navgraph=navgraph, is_grabbed = is_grabbed},
   {"INIT", JumpState},
   {"DRIVE_TO_MACHINE_POINT", SkillJumpState, skills={{drive_to_machine_point}}, final_to="CHECK_PUCK", fail_to="FAILED"},
   {"CHECK_PUCK", JumpState},
   {"CONVEYOR_ALIGN", SkillJumpState, skills={{conveyor_align}}, final_to="PRODUCT_PUT", fail_to="FAILED"},
   {"PRODUCT_PUT", SkillJumpState, skills={{product_put}}, final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not navgraph", desc="navgraph not available"},
   {"INIT", "FAILED", cond="not vars.node:is_valid()", desc="point invalid"},
   {"INIT", "CONVEYOR_ALIGN", cond=already_at_conveyor, desc="At mps, skip drive_to_local"},
   {"INIT", "DRIVE_TO_MACHINE_POINT", cond=true, desc="Everything OK"},
   {"CHECK_PUCK", "FAILED", cond="not is_grabbed()", desc="Lost Puck"},
   {"CHECK_PUCK", "CONVEYOR_ALIGN", cond=true}
}

function INIT:init()
   self.fsm.vars.node = navgraph:node(self.fsm.vars.place)
   if self.fsm.vars.side == nil then
     self.fsm.vars.side = "input"
   end
end

function DRIVE_TO_MACHINE_POINT:init()
   local option = "CONVEYOR"

   if self.fsm.vars.slide then
      option = "SLIDE"
   end

   if self.fsm.vars.side == "output" then
      self.fsm.vars.tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_output")
      self.args["drive_to_machine_point"] = {place = self.fsm.vars.place .. "-O", option = option, x_at_mps=X_AT_MPS, tag_id=self.fsm.vars.tag_id}
   else --if no side is given drive to output
      self.fsm.vars.tag_id = navgraph:node(self.fsm.vars.place):property_as_float("tag_input")
      self.args["drive_to_machine_point"] = {place = self.fsm.vars.place .. "-I", option = option, x_at_mps=X_AT_MPS, tag_id=self.fsm.vars.tag_id}
   end
end

function DRIVE_TO_MACHINE_POINT:exit()
  local dtmp_fsm = skillenv.get_skill_fsm("drive_to_machine_point")
  if dtmp_fsm.current == dtmp_fsm.states[dtmp_fsm.fail_state] then
    self.fsm.vars.error = "Drive To Machine Point Failed"
  end
end

function CONVEYOR_ALIGN:init()
    self.args["conveyor_align"].side = self.fsm.vars.side
    self.args["conveyor_align"].place = self.fsm.vars.place
    self.args["conveyor_align"].slide = self.fsm.vars.slide
end

function CONVEYOR_ALIGN:exit()
  local cv_fsm = skillenv.get_skill_fsm("conveyor_align")
  if cv_fsm.current == cv_fsm.states[cv_fsm.fail_state] then
    self.fsm.vars.error = "Conveyor Align Failed"
  end
end

function PRODUCT_PUT:init()
  self.args["product_put"].place = self.fsm.vars.place
  self.args["product_put"].slide = self.fsm.vars.slide
  self.args["product_put"].side = self.fsm.vars.side
end

function FINAL:init()
  laserline_switch:msgq_enqueue(laserline_switch.DisableSwitchMessage:new())
end

function FAILED:init()
  laserline_switch:msgq_enqueue(laserline_switch.DisableSwitchMessage:new())
end

function PRODUCT_PUT:exit()
  local pp_fsm = skillenv.get_skill_fsm("product_put")
  if pp_fsm.current == pp_fsm.states[pp_fsm.fail_state] then
    self.fsm.vars.error = "Product Put Failed"
  end
end

function FAILED:init()
  if self.fsm.vars.error then
    self.fsm:set_error(self.fsm.vars.error)
  end
end
