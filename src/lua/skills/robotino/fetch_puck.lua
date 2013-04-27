
----------------------------------------------------------------------------
--  fetch_puck.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
name               = "fetch_puck"
fsm                = SkillHSM:new{name=name, start="WAIT_FOR_VISION", debug=true}
depends_skills     = { "motor_move" }
depends_interfaces = {
    {v = "omnivisionSwitch", type="SwitchInterface", id="omnivisionSwitch"},
    {v = "omnipuck", type="Position3DInterface", id="OmniPuck1"},
    {v = "sensor", type="RobotinoSensorInterface"},
    {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[Move to puck pickup position]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT = 15
local ORI_OFFSET = 0.03
local THRESHOLD_DISTANCE = 0.05
local MIN_VIS_HIST = 5

local tfm = require 'tf_module'

function puck_in_front()
   if math.abs(math.atan2(omnipuck:translation(1), omnipuck:translation(0))) < ORI_OFFSET then
      printf("front: %f, %f", omnipuck:translation(0), omnipuck:translation(1))
      return true
   end
   return false
end

function have_puck()
    local curDistance = sensor:distance(8)
    if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
        printf("near: " .. curDistance)
        return true
    end
    return false
end

function visible()
   return omnipuck:visibility_history() >= MIN_VIS_HIST
end

fsm:define_states{ export_to=_M, closure={have_puck=have_puck, omnipuck=omnipuck,
      visible=visible, puck_in_front=puck_in_front},
   {"WAIT_FOR_VISION", JumpState},
   {"TURN_TO_PUCK", SkillJumpState, skills={{motor_move}}, final_to="CHECK_TURN",
      fail_to="WAIT_FOR_VISION"},
   {"CHECK_TURN", JumpState},
   {"GRAB", SkillJumpState, skills={{motor_move}}, final_to="GRAB_DONE",
      fail_to="FAILED"},
   -- GRAB motor_move's too far and preempts if have_puck. If motor_move finishes we
   -- MOVE_MORE an extra 5 cm to be sure.
   {"GRAB_DONE", JumpState},
   {"MOVE_MORE", SkillJumpState, skills={{motor_move}}, final_to="FINAL",
      fail_to="FAILED"}
}

fsm:add_transitions{
   {"WAIT_FOR_VISION", "TURN_TO_PUCK", cond=visible },
   {"WAIT_FOR_VISION", "FAILED", timeout=TIMEOUT},
   {"CHECK_TURN", "GRAB", cond="visible() and puck_in_front()"},
   {"CHECK_TURN", "WAIT_FOR_VISION", cond="visible() and not puck_in_front()"},
   {"CHECK_TURN", "FAILED", timeout=TIMEOUT},
   {"GRAB", "MOVE_MORE", cond=have_puck},
   {"GRAB_DONE", "WAIT_FOR_VISION", cond="not have_puck()"},
   {"GRAB_DONE", "MOVE_MORE", cond=have_puck},
}

function MOVE_MORE:init()
   self.skills[1].x = 0.08
end

function WAIT_FOR_VISION:init()
   local msg = omnivisionSwitch.EnableSwitchMessage:new()
   omnivisionSwitch:msgq_enqueue_copy(msg)
end

function TURN_TO_PUCK:init()
   self.skills[1].ori = math.atan2(omnipuck:translation(1), omnipuck:translation(0))
end

function GRAB:init()
   local x = omnipuck:translation(0)
   local y = omnipuck:translation(1)
   printf("GRAB: %f,%f", x, y)
   self.skills[1].x = math.sqrt(x^2 + y^2) + 0.3 
end

function cleanup()
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(0, 0, 0))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
   local msg = omnivisionSwitch.DisableSwitchMessage:new()
   omnivisionSwitch:msgq_enqueue_copy(msg)   
end

function FAILED:init() cleanup() end

function FINAL:init() cleanup() end


