
----------------------------------------------------------------------------
--  gripper.lua - Skill to control the gripper
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
name               = "gripper_commands"
fsm                = SkillHSM:new{name=name, start="CHECK_WRITER", debug=false}
depends_skills     = nil
depends_interfaces = {
    {v = "arduino", type = "ArduinoInterface", id="Arduino"},
}

documentation      = [==[
    Control of the gripper.
    Setting an absolute position sets the position of gripper_z_dyn.
    By default, absolute positions are given in the frame of gripper_home.
    gripper_home and gripper_z_dyn coincide exactly then, when the absolute position is set 0,0,0.

    @param command    can be : ( OPEN | CLOSE | MOVEABS | MOVEREL | CALIBRATE )
    @param x   x position for gripper move
    @param y   y position for gripper move
    @param z   z position for gripper move
    @param x_rel Mocks MOVEREL by adding the value to current position
    @param y_rel Mocks MOVEREL by adding the value to current position
    @param z_rel Mocks MOVEREL by adding the value to current position
    *_rel params work only if target_frame is gripper_home or not defined
    @param target_frame   target frame of absolute coordinates
    @param wait (optional, default: true) force the skill to wait on arduino plugin

]==]



-- Initialize as skill module
skillenv.skill_module(_M)
local tfm = require("fawkes.tfutils")

function get_current_pos()
   local tmp = { x = 0,
                 y = 0,
                 z = 0,
                 ori = { x = 0, y = 0, z = 0, w = 0}
   }

   return tfm.transform6D(tmp,"gripper_z_dyn","gripper_home")
end

function clip_value_x(value)
  return math.max(0,math.min(value,fsm.vars.x_max))
end

function clip_value_y(value)
  return math.max(-fsm.vars.y_max/2,math.min(value,fsm.vars.y_max/2))
end

function clip_value_z(value)
  return math.max(0,math.min(value,fsm.vars.z_max))
end

function input_ok()
  if fsm.vars.command == "OPEN" or fsm.vars.command == "CLOSE" then
    return true
  end
  if fsm.vars.command == "MOVEABS" then
    if fsm.vars.x_rel or fsm.vars.y_rel or fsm.vars.z_rel then
      if fsm.vars.target_frame and fsm.vars.target_frame ~= "gripper_home" then
        print("Can use *_rel only with gripper_home as target_frame")
        return false
      end
    end
    if       (not fsm.vars.x and not fsm.vars.x_rel) 
          or (not fsm.vars.y and not fsm.vars.y_rel)
          or (not fsm.vars.z and not fsm.vars.z_rel) then
      print("Missing coordinates " .. fsm.vars.x .. " " .. fsm.vars.y .. " " ..fsm.vars.z)
      return false
    else 
      return true
    end
  end

  if fsm.vars.command == "MOVEREL" then
    if not fsm.vars.x or not fsm.vars.y or not fsm.vars.z then
      print("Missing coordinates " .. fsm.vars.x .. " " .. fsm.vars.y .. " " ..fsm.vars.z)
      return false
    elseif fsm.vars.target_frame then
      print("Target frame may not be specified for MOVEREL command")
      return false
    else
      return true
    end
  end

  if fsm.vars.command == "CALIBRATE" then
    return true
  end
  print("Unknown command: " .. fsm.vars.command)
  return false
end

function tf_ready()
-- Checks if the tf tree of the gripper is complete and up to date after moving
-- This is done by checking if the latest transform update is newer than the
-- arduino interface timestamp
  if not arduino:is_final() then
    return false
  end

  local bb_stamp = arduino:timestamp()
  if not tf:can_transform("gripper", "gripper_home", bb_stamp) then
    return false
  end

  local transform = fawkes.tf.StampedTransform:new()
  tf:lookup_transform("gripper", "gripper_home", transform)
  if transform.stamp:in_usec() < bb_stamp:in_usec() then
    return false
  end
  return true
end


function is_error()
  msgid = arduino:msgid()
  if msgid == nil then
    return false
  end
  if msgid ~= fsm.vars.msgid then 
    return false
  end
  status = arduino:status()
  if status == 2 or status == 3 or status == 4 then
    return true
  end
  return false
end

-- States
fsm:define_states{
   export_to=_M,
   closure={arduino=arduino, is_error=is_error, input_ok = input_ok, tf_ready=tf_ready, clip_value_x=clip_value_x, clip_value_y=clip_value_y, clip_value_z=clip_value_z, get_current_pos=get_current_pos},
   {"CHECK_WRITER", JumpState},
   {"COMMAND", JumpState},
   {"WAIT", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"CHECK_WRITER", "FAILED", cond="vars.error"},
   {"CHECK_WRITER", "FAILED", cond="not input_ok()", desc="Input not correct"},
   {"CHECK_WRITER", "FAILED", precond="not arduino:has_writer()", desc="No writer for gripper"},
   {"CHECK_WRITER", "COMMAND", cond=true, desc="Writer ok got to command"},
   {"COMMAND", "WAIT", timeout=0.2},
   {"WAIT", "FAILED", cond="is_error()"},
   {"WAIT", "FINAL", cond="vars.wait ~= nil and not vars.wait"},
   {"WAIT", "FINAL", cond="arduino:is_final() and tf_ready()"},
   {"WAIT", "FAILED", timeout=15},
}

function CHECK_WRITER:init()
   if config:exists("/arduino/x_max") then
       self.fsm.vars.x_max = config:get_float("/arduino/x_max")
   else
       self.fsm:set_error("arduino x_max config not found")
       self.fsm.vars.error = true
   end
   if config:exists("/arduino/y_max") then
       self.fsm.vars.y_max = config:get_float("/arduino/y_max")
   else
       self.fsm:set_error("arduino y_max config not found")
       self.fsm.vars.error = true
   end
   if config:exists("/arduino/z_max") then
       self.fsm.vars.z_max = config:get_float("/arduino/z_max")
   else
       self.fsm:set_error("arduino z_max config not found")
       self.fsm.vars.error = true
   end
end

function COMMAND:init()

   if self.fsm.vars.command == "OPEN" then
      theOpenMessage = arduino.OpenGripperMessage:new()
      arduino:msgq_enqueue(theOpenMessage)

   elseif self.fsm.vars.command == "CLOSE" then
      theCloseMessage = arduino.CloseGripperMessage:new()
      arduino:msgq_enqueue(theCloseMessage)

   elseif self.fsm.vars.command == "MOVEABS" or self.fsm.vars.command == "MOVEREL" then

        local x = 0.0
        local y = 0.0
        local z = 0.0

        current_pos = get_current_pos()

        if self.fsm.vars.command == "MOVEABS" then
          if self.fsm.vars.x then
            x = self.fsm.vars.x
          end
          if self.fsm.vars.x_rel then
            x = clip_value_x(current_pos.x + self.fsm.vars.x_rel)
          end

          if self.fsm.vars.y then
            y = self.fsm.vars.y
          end
          if self.fsm.vars.y_rel then
            y = clip_value_y(current_pos.y + self.fsm.vars.y_rel)
          end
           
          if self.fsm.vars.z then
            z = self.fsm.vars.z
          end
          if self.fsm.vars.z_rel then
            z = clip_value_z(current_pos.z + self.fsm.vars.z_rel)
          end
          target_frame = self.fsm.vars.target_frame or "gripper_home"
        else
          x = clip_value_x(current_pos.x + self.fsm.vars.x)
          y = clip_value_y(current_pos.y + self.fsm.vars.y)
          z = clip_value_z(current_pos.z + self.fsm.vars.z)
          target_frame = "gripper_home"
        end

        move_abs_message = arduino.MoveXYZAbsMessage:new()
        move_abs_message:set_x(x)
        move_abs_message:set_y(y)
        move_abs_message:set_z(z)
        move_abs_message:set_target_frame(target_frame)
        self.fsm.vars.msgid = arduino:msgq_enqueue_copy(move_abs_message)

   elseif self.fsm.vars.command == "CALIBRATE" then
        calibrate_message = arduino.CalibrateMessage:new()
        arduino:msgq_enqueue_copy(calibrate_message)
   end
end
