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
name = "gripper_commands"
fsm = SkillHSM:new{name = name, start = "CHECK_WRITER", debug = false}
depends_skills = nil
depends_interfaces = {
    {v = "arduino", type = "ArduinoInterface", id = "Arduino"}
}

documentation = [==[
    @param command    can be : ( OPEN | CLOSE | STOP | MOVEABS | MOVEREL | CALIBRATE | RESETUSB )
    @param x   x position for gripper move
    @param y   y position for gripper move
    @param z   z position for gripper move
    @param target_frame   target frame of absolute coordinates
    @param wait (optional, default: true) force the skill to wait on arduino plugin
    @param sense (optional, default: false) force the skill to end when detecting workpiece

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function input_ok()
    if fsm.vars.command == "OPEN" or fsm.vars.command == "STOP" or
        fsm.vars.command == "CLOSE" or fsm.vars.command == "RESETUSB" then
        return true
    end
    if fsm.vars.command == "MOVEABS" or fsm.vars.command == "MOVEREL" then
        if not fsm.vars.x or not fsm.vars.y or not fsm.vars.z then
            print("Missing coordinates " .. fsm.vars.x .. " " .. fsm.vars.y ..
                      " " .. fsm.vars.z)
            return false
        else
            return true
        end
    end

    -- handle optional sense run
    if fsm.vars.sense ~= true then fsm.vars.sense = false end

    if fsm.vars.command == "CALIBRATE" then return true end
    print("Unknown command: " .. fsm.vars.command)
    return false
end

function tf_ready()
    -- Checks if the tf tree of the gripper is complete and up to date after moving
    -- This is done by checking if the latest transform update is newer than the
    -- arduino interface timestamp
    if not arduino:is_final() then return false end

    local bb_stamp = arduino:timestamp()
    if not tf:can_transform("gripper", "end_effector_home", bb_stamp) then
        return false
    end

    local transform = fawkes.tf.StampedTransform:new()
    tf:lookup_transform("gripper", "end_effector_home", transform)
    if transform.stamp:in_usec() < bb_stamp:in_usec() then return false end
    return true
end

function is_error()
    msgid = arduino:msgid()
    if msgid == nil then return false end
    if msgid ~= fsm.vars.msgid then return false end
    status = arduino:status()
    if status == 2 or status == 3 or status == 4 then return true end
    return false
end

function sensed_wp() return fsm.vars.sense and arduino:is_wp_sensed() end

-- States
fsm:define_states{
    export_to = _M,
    closure = {
        arduino = arduino,
        is_error = is_error,
        input_ok = input_ok,
        tf_ready = tf_ready
    },
    {"CHECK_WRITER", JumpState},
    {"COMMAND", JumpState},
    {"WAIT", JumpState}
}

-- Transitions
fsm:add_transitions{
    {
        "CHECK_WRITER",
        "FAILED",
        cond = "not input_ok()",
        desc = "Input not correct"
    }, {
        "CHECK_WRITER",
        "FAILED",
        precond = "not arduino:has_writer()",
        desc = "No writer for gripper"
    },
    {"CHECK_WRITER", "COMMAND", cond = true, desc = "Writer ok got to command"},
    {"COMMAND", "WAIT", timeout = 0.2}, {"WAIT", "FAILED", cond = "is_error()"},
    {"WAIT", "FINAL", cond = sensed_wp},
    {"WAIT", "FINAL", cond = "vars.wait ~= nil and not vars.wait"},
    {"WAIT", "FINAL", cond = "arduino:is_final() and tf_ready()"},
    {"WAIT", "FAILED", timeout = 15}
}

function COMMAND:init()

    if self.fsm.vars.command == "OPEN" then
        theOpenMessage = arduino.OpenGripperMessage:new()
        arduino:msgq_enqueue(theOpenMessage)

    elseif self.fsm.vars.command == "CLOSE" then
        theCloseMessage = arduino.CloseGripperMessage:new()
        arduino:msgq_enqueue(theCloseMessage)

    elseif self.fsm.vars.command == "STOP" then
        theStopMessage = arduino.StopMessage:new()
        arduino:msgq_enqueue(theStopMessage)
    elseif self.fsm.vars.command == "RESETUSB" then
        theResetMessage = arduino.ResetUSBMessage:new()
        arduino:msgq_enqueue(theResetMessage)

    elseif self.fsm.vars.command == "MOVEABS" then

        x = self.fsm.vars.x
        y = self.fsm.vars.y
        z = self.fsm.vars.z
        target_frame = self.fsm.vars.target_frame or "end_effector_home"

        move_abs_message = arduino.MoveXYZAbsMessage:new()
        move_abs_message:set_x(x)
        move_abs_message:set_y(y)
        move_abs_message:set_z(z)
        move_abs_message:set_target_frame(target_frame)
        self.fsm.vars.msgid = arduino:msgq_enqueue_copy(move_abs_message)

    elseif self.fsm.vars.command == "MOVEREL" then
        x = self.fsm.vars.x
        y = self.fsm.vars.y
        z = self.fsm.vars.z
        move_rel_message = arduino.MoveXYZRelMessage:new()
        move_rel_message:set_x(x)
        move_rel_message:set_y(y)
        move_rel_message:set_z(z)
        self.fsm.vars.msgid = arduino:msgq_enqueue_copy(move_rel_message)

    elseif self.fsm.vars.command == "CALIBRATE" then
        calibrate_message = arduino.CalibrateMessage:new()
        arduino:msgq_enqueue_copy(calibrate_message)
    end
end
