
----------------------------------------------------------------------------
--  test_tensorflow.lua - Skill to test the functionality of the tensorflow plugin
--
--  Created: Sun Jun 06 10:46:33 2019
--  Copyright  2019  Morian Sonnet
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
name               = "test_tensorflow"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=false}
depends_skills     = nil
depends_interfaces = {
    {v = "tensorflow", type = "TensorflowInterface", id="Tensorflow"},
}

documentation      = [==[
    @param command    can be : ( SET_GRAPH | SET_SOURCE_SHM | SET_SOURCE_FILE | SET_SOURCE_V4L2 | TRIGGER_RUN )
    All other params come directly from the TensorflowInterface
]==]



-- Initialize as skill module
skillenv.skill_module(_M)


-- States
fsm:define_states{
   export_to=_M,
   closure={tensorflow=tensorflow},
   {"COMMAND", JumpState},
   {"CHECK_WRITER", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"COMMAND", "FINAL", timeout=0.2},
   {"CHECK_WRITER", "FAILED", precond="not tensorflow:has_writer()", desc="No writer for interface"},
   {"CHECK_WRITER", "COMMAND", cond=true},
}

function COMMAND:init()

   if self.fsm.vars.command == "SET_GRAPH" then
     message = tensorflow.LoadGraphFileMessage:new()
     message:set_file_path(self.fsm.vars.file_path)
     message:set_input_node(self.fsm.vars.input_node)
     message:set_output_node(self.fsm.vars.output_node)
   elseif self.fsm.vars.command == "SET_SOURCE_SHM" then
     message = tensorflow.SetSourceImageSHMMessage:new()
     message:set_shm_id(self.fsm.vars.shm_id)
     message:set_hostname(self.fsm.vars.hostname)
     message:set_normalize(self.fsm.vars.normalize)
     message:set_norm_mean(self.fsm.vars.norm_mean)
     message:set_norm_std(self.fsm.vars.norm_std)
     message:set_width(self.fsm.vars.width)
     message:set_height(self.fsm.vars.height)
     message:set_image_dtype(self.fsm.vars.image_dtype)
   elseif self.fsm.vars.command == "SET_SOURCE_FILE" then
     message = tensorflow.SetSourceImageFileMessage:new()
     message:set_file_name(self.fsm.vars.file_name)
     message:set_normalize(self.fsm.vars.normalize)
     message:set_norm_mean(self.fsm.vars.norm_mean)
     message:set_norm_std(self.fsm.vars.norm_std)
     message:set_width(self.fsm.vars.width)
     message:set_height(self.fsm.vars.height)
     message:set_image_dtype(self.fsm.vars.image_dtype)
   elseif self.fsm.vars.command == "SET_SOURCE_V4L2" then
     message = tensorflow.SetSourceImageV4L2Message:new()
     message:set_file_name(self.fsm.vars.device_name)
     message:set_normalize(self.fsm.vars.normalize)
     message:set_norm_mean(self.fsm.vars.norm_mean)
     message:set_norm_std(self.fsm.vars.norm_std)
     message:set_width(self.fsm.vars.width)
     message:set_height(self.fsm.vars.height)
     message:set_image_dtype(self.fsm.vars.image_dtype)
   elseif self.fsm.vars.command == "TRIGGER_RUN" then
     message = tensorflow.TriggerRunMessage:new()
   end
   tensorflow:msgq_enqueue(message)
end
