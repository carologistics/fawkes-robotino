
----------------------------------------------------------------------------
--  gripper.lua - Skill to open or close Robotino AX12 gripper
--
--  Created: Sat Feb 28 10:46:33 2015
--  Copyright  2014  Sebastian Reuter
--             2014  Tim Niemueller
--             2015  Nicolas Limpert
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
name               = "gripper_commands_new"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=false}
depends_skills     = nil
depends_interfaces = {
    {v = "arduino", type = "ArduinoInterface", id="Arduino"},

}

documentation      = [==[
    @param command    can be : ( OPEN | CLOSE | GRAB | MOVEABS | CALIBRATE | MOVEGRIPPERABS ) or DOWN (UP or DOWN require the desired number of millimeters)
    @param x   absolute x position for gripper move
    @param y   absolute y position for gripper move
    @param z   absolute z position for gripper move
    @param wait if the skill should wait for the movement to finish

]==]



-- Initialize as skill module
skillenv.skill_module(_M)


function is_error()
  msgid = arduino:msgid()
  if msgid == nil then
    return false
  end
  if msgid ~= fsm.vars.msgid then 
    return false
  end
  errorstate = arduino:errorstate()
  if errorstate == 2 or errorstate == 3 or errorstate == 4 then
    return true
  end
  return false
end

function is_grab()
  return fsm.vars.command == "GRAB"
end

-- States
fsm:define_states{
   export_to=_M,
   closure={arduino=arduino, is_error=is_error,is_grab=is_grab},
   {"CHECK_WRITER", JumpState},
   {"COMMAND", JumpState},
   {"WAIT", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"CHECK_WRITER", "FAILED", precond="not arduino:has_writer()", desc="No writer for gripper"},
   {"CHECK_WRITER", "COMMAND", cond=true, desc="Writer ok got to command"},
   {"COMMAND", "WAIT", timeout=0.2},
   {"WAIT", "FAILED", cond="is_error()"},
   {"WAIT", "FINAL", cond="is_grab() and arduino:is_holds_puck() and arduino:is_final()"},
   {"WAIT", "FINAL", cond="vars.wait ~= nil and not vars.wait"},
   {"WAIT", "FINAL", cond="vars.restore"},
   {"WAIT", "FINAL", cond="arduino:is_final() and not is_grab()"},
--   {"WAIT", "FAILED", cond="vars.error"},
}

function COMMAND:init()

   if self.fsm.vars.command == "OPEN" then
      self.fsm.vars.open = true
      theOpenMessage = arduino.OpenGripperMessage:new()
      arduino:msgq_enqueue(theOpenMessage)

   elseif self.fsm.vars.command == "CLOSE" or self.fsm.vars.command == "GRAB" then
      self.fsm.vars.close = true
      theCloseMessage = arduino.CloseGripperMessage:new()
      arduino:msgq_enqueue(theCloseMessage)

   elseif self.fsm.vars.command == "MOVEABS" then

        target_frame = self.fsm.vars.target_frame or "gripper_home"

        move_abs_message = arduino.MoveXYZAbsClippedMessage:new()
        if self.fsm.vars.x ~= nil then
          move_abs_message:set_x(self.fsm.vars.x)
        end
        if self.fsm.vars.y ~= nil then
          move_abs_message:set_y(self.fsm.vars.y)
        end
        if self.fsm.vars.z ~= nil then
          move_abs_message:set_z(self.fsm.vars.z)
        end
        move_abs_message:set_target_frame(target_frame)
        self.fsm.vars.msgid = arduino:msgq_enqueue_copy(move_abs_message)

   elseif self.fsm.vars.command == "CALIBRATE" then
        calibrate_message = arduino.CalibrateMessage:new()
        arduino:msgq_enqueue_copy(calibrate_message)

   elseif self.fsm.vars.command == "MOVEGRIPPERABS" then
        move_gripper_abs_message = arduino.MoveGripperAbsMessage:new()
        a = self.fsm.vars.abs_gripper_pos or -1
        move_gripper_abs_message:set_a(a)
        self.fsm.vars.msgid = arduino:msgq_enqueue_copy(move_gripper_abs_message)
   end
end
