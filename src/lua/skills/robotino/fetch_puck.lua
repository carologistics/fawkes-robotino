
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
fsm                = SkillHSM:new{name=name, start="TURN_ON_OMNIVISION", debug=true}
depends_skills     = { "motor_move" }
depends_interfaces = {
    {v = "omnivisionSwitch", type="SwitchInterface", id="omnivisionSwitch"},
    {v = "omnipuck1", type="Position3DInterface", id="OmniPuck1"},
--    {v = "omnipuck2", type="Position3DInterface", id="OmniPuck2"},
--    {v = "omnipuck3", type="Position3DInterface", id="OmniPuck3"},
--    {v = "omnipuck4", type="Position3DInterface", id="OmniPuck4"},
--    {v = "omnipuck5", type="Position3DInterface", id="OmniPuck5"},
     {v = "sensor", type="RobotinoSensorInterface"},
     {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[Move to puck pickup position]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT = 30
local ORI_OFFSET = 0.03
local THRESHOLD_DISTANCE = 0.08
local omnipucks = { omnipuck1, omnipuck2, omnipuck3, omnipuck4, omnipuck5 }

local pm = require 'puck_loc_module'
local tfm = require 'tf_module'

function get_puck_loc()
--   local bestpuck = nil
--   local max_vis_hist = -1
--   local min_angle = 2*math.pi
--   for i,op in ipairs(omnipucks) do
      op = omnipuck1
      local p_bl = pm.get_puck_loc(op)
--      if p_bl then
--         p_bl.hist = op:visibility_history()
--         if not fsm.vars.puck_bl then
--            -- 1st time: prefer best visibility history
--            if p_bl.hist > max_vis_hist then
               bestpuck = p_bl
--            end
--         else
--            -- every other time: prefer puck with least angle
--            local angle = math.atan2(p_bl.y, p_bl.x)
--            if math.abs(angle) < min_angle then
--               bestpuck = p_bl
--            end
--         end
--      end
--   end
   if bestpuck then
      printf("bestpuck= %f, %f", bestpuck.x, bestpuck.y)
      fsm.vars.puck_bl = bestpuck
      fsm.vars.puck_loc = bestpuck
      return true
   end
   return false
end

function no_puck()
   if os.time() - fsm.vars.start_time > TIMEOUT and not fsm.vars.puck_loc then
      return true
   end
   return false
end

function puck()
   if get_puck_loc() then return true end
   return false
end

function puck_in_front()
   if get_puck_loc() and math.abs(math.atan2(
    fsm.vars.puck_bl.y, fsm.vars.puck_bl.x)) < ORI_OFFSET then
      printf("front: %f, %f", fsm.vars.puck_loc.x, fsm.vars.puck_loc.y)
      return true
   end
   return false
end

function puck_not_in_front()
   return not puck_in_front()
end

function have_puck()
    local curDistance = sensor:distance(8)
    if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
        printf("near: " .. curDistance)
        return true
    end
    return false
end

function dont_have_puck()
   return not have_puck()
end

function send_transrot(vx, vy, omega)
    local oc  = motor:controller()
    local ocn = motor:controller_thread_name()
    motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
    motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
    motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

fsm:define_states{ export_to=_M,
   {"TURN_ON_OMNIVISION", JumpState},
   {"WAIT_FOR_VISION", JumpState},
   {"SEE_PUCK", JumpState},
   {"OMNIFAIL", JumpState},
   {"TURN_TO_PUCK", SkillJumpState, skills={{motor_move}}, final_to="GRAB_PUCK",
      fail_to="SEE_PUCK"},
   {"GRAB_PUCK", SkillJumpState, skills={{motor_move}}, final_to="MOVE_DONE",
      fail_to="OMNIFAIL"},
   -- GRAB_PUCK motor_move's too far and preempts if have_puck. If motor_move finishes we
   -- MOVE_MORE an extra 5 cm to be sure.
   {"MOVE_DONE", JumpState},
   {"MOVE_MORE", SkillJumpState, skills={{motor_move}}, final_to="TURN_OFF_OMNIVISION",
      fail_to="OMNIFAIL"},
   {"TURN_OFF_OMNIVISION", JumpState}
}

fsm:add_transitions{
   {"TURN_ON_OMNIVISION", "WAIT_FOR_VISION", cond=true },
   {"WAIT_FOR_VISION", "SEE_PUCK", timeout=0.333 },
   {"SEE_PUCK", "OMNIFAIL", cond=no_puck, desc="No puck found by OmniVision"},
   {"SEE_PUCK", "TURN_TO_PUCK", cond=puck, desc="Found a puck"},
--   {"TURN_TO_PUCK", "MOVE_FORWARD", skill=motor_move, fail_to="SEE_PUCK"},
--   {"MOVE_FORWARD", "ARRIVED", skill=motor_move, fail_to="SEE_PUCK"},
--   {"ARRIVED", "SEE_PUCK", cond=puck_not_in_front, desc="Puck gone after approach"},
--   {"ARRIVED", "GRAB_PUCK", cond=puck_in_front},
   {"GRAB_PUCK", "MOVE_MORE", cond=have_puck },
   {"MOVE_DONE", "SEE_PUCK", cond=dont_have_puck },
   {"MOVE_DONE", "TURN_OFF_OMNIVISION", cond=have_puck },
   {"TURN_OFF_OMNIVISION", "FINAL", cond=true },
   {"OMNIFAIL", "FAILED", cond=true }
}

--function MOVE_FORWARD:init()
--   local d = math.sqrt(self.fsm.vars.puck_bl.x^2 + self.fsm.vars.puck_bl.y^2)
--   self.skills[1].motor_move.args = { x = d/3, y = 0, ori = 0 }
--end

function MOVE_MORE:init()
   self.skills[1].x = 0.05 
   self.skills[1].y = 0 
   self.skills[1].ori = 0
end

function OMNIFAIL:init()
    local msg = omnivisionSwitch.DisableSwitchMessage:new()
    omnivisionSwitch:msgq_enqueue_copy(msg)   
end

function TURN_ON_OMNIVISION:init()
   local msg = omnivisionSwitch.EnableSwitchMessage:new()
   omnivisionSwitch:msgq_enqueue_copy(msg)
   self.fsm.vars.start_time = os.time()
end


function TURN_TO_PUCK:init()
   self.skills[1].x = 0 
   self.skills[1].y = 0
   self.skills[1].ori = math.atan2(self.fsm.vars.puck_bl.y, self.fsm.vars.puck_bl.x)
end

function GRAB_PUCK:init()
   local x = math.sqrt(self.fsm.vars.puck_bl.x^2 + self.fsm.vars.puck_bl.y^2) + 0.3
   printf("dist=%f", x)
   self.skills[1].x = x 
   self.skills[1].y = 0 
   self.skills[1].ori = 0
end

function TURN_OFF_OMNIVISION:init()
   send_transrot(0, 0, 0)
   local msg = omnivisionSwitch.DisableSwitchMessage:new()
   omnivisionSwitch:msgq_enqueue_copy(msg)   
end




