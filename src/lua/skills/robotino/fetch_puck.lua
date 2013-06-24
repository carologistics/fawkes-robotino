
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
    {v = "omnipuck1", type="Position3DInterface", id="OmniPuck1"},
    {v = "omnipuck2", type="Position3DInterface", id="OmniPuck2"},
    {v = "omnipuck3", type="Position3DInterface", id="OmniPuck3"},
    {v = "omnipuck4", type="Position3DInterface", id="OmniPuck4"},
    {v = "omnipuck5", type="Position3DInterface", id="OmniPuck5"},
    {v = "omnipuck6", type="Position3DInterface", id="OmniPuck6"},
    {v = "omnipuck7", type="Position3DInterface", id="OmniPuck7"},
    {v = "omnipuck8", type="Position3DInterface", id="OmniPuck8"},
    {v = "omnipuck9", type="Position3DInterface", id="OmniPuck9"},
    {v = "omnipuck10", type="Position3DInterface", id="OmniPuck10"},
    {v = "omnipuck11", type="Position3DInterface", id="OmniPuck11"},
    {v = "omnipuck12", type="Position3DInterface", id="OmniPuck12"},
    {v = "omnipuck13", type="Position3DInterface", id="OmniPuck13"},
    {v = "omnipuck14", type="Position3DInterface", id="OmniPuck14"},
    {v = "omnipuck15", type="Position3DInterface", id="OmniPuck15"},
    {v = "omnipuck16", type="Position3DInterface", id="OmniPuck16"},
    {v = "omnipuck17", type="Position3DInterface", id="OmniPuck17"},
    {v = "omnipuck18", type="Position3DInterface", id="OmniPuck18"},
    {v = "omnipuck19", type="Position3DInterface", id="OmniPuck19"},
    {v = "omnipuck20", type="Position3DInterface", id="OmniPuck20"},
    {v = "sensor", type="RobotinoSensorInterface"},
    {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[Move to puck pickup position]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT = 3
local ORI_OFFSET = 0.03
-- you can find the config value in /cfg/host.yaml
local THRESHOLD_DISTANCE = config:get_float("/skills/fetch_puck/front_sensor_dist")
local MIN_VIS_HIST = 15
local EPSILON = 0.07
local omnipucks = {
   omnipuck1, 
   omnipuck2,
   omnipuck3,
   omnipuck4,
   omnipuck5,
   omnipuck6,
   omnipuck7,
   omnipuck8,
   omnipuck9,
   omnipuck10,
   omnipuck11, 
   omnipuck12,
   omnipuck13,
   omnipuck14,
   omnipuck15,
   omnipuck16,
   omnipuck17,
   omnipuck18,
   omnipuck19,
   omnipuck20
}


local tfm = require 'tf_module'

function have_puck()
    local curDistance = sensor:distance(8)
    if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
        printf("near: " .. curDistance)
        return true
    end
    return false
end

function visible()
   for _,o in ipairs(omnipucks) do
      if o:visibility_history() >= MIN_VIS_HIST then
         return true
      end
   end
   return false
end

fsm:define_states{ export_to=_M, closure={have_puck=have_puck, omnipuck=omnipuck,
      visible=visible},
   {"WAIT_FOR_VISION", JumpState},
   {"DRIVE_SIDEWAYS_TO_PUCK", SkillJumpState, skills={{motor_move}}, final_to="TURN_TO_PUCK",
      fail_to="WAIT_FOR_VISION"},
   {"TURN_TO_PUCK", SkillJumpState, skills={{motor_move}}, final_to="GRAB",
      fail_to="WAIT_FOR_VISION"},
   {"GRAB", SkillJumpState, skills={{motor_move}}, final_to="GRAB_DONE",
      fail_to="FAILED"},
   -- GRAB motor_move's too far and preempts if have_puck. If motor_move finishes we
   -- MOVE_MORE an extra 5 cm to be sure.
   {"GRAB_DONE", JumpState},
   {"MOVE_MORE", SkillJumpState, skills={{motor_move}}, final_to="FINAL",
      fail_to="FAILED"}
}

fsm:add_transitions{
   {"WAIT_FOR_VISION", "TURN_TO_PUCK", cond="visible() and not  vars.move_sideways" },
   {"WAIT_FOR_VISION", "DRIVE_SIDEWAYS_TO_PUCK", cond="visible() and vars.move_sideways" },
   {"WAIT_FOR_VISION", "FAILED", timeout=TIMEOUT},
   {"GRAB", "MOVE_MORE", cond=have_puck},
   {"GRAB_DONE", "WAIT_FOR_VISION", cond="not have_puck()"},
   {"GRAB_DONE", "MOVE_MORE", cond=have_puck},
}

function MOVE_MORE:init()
   self.skills[1].x = 0.05
end

function WAIT_FOR_VISION:init()
   local msg = omnivisionSwitch.EnableSwitchMessage:new()
   omnivisionSwitch:msgq_enqueue_copy(msg)
end

function TURN_TO_PUCK:init()
   local min_ori_abs = 10
   local min_ori = 0
   local target
   for _,o in ipairs(omnipucks) do
      if o:visibility_history() >= MIN_VIS_HIST then
         local x = o:translation(0)
         local y = o:translation(1)
         local ori = math.atan2(y, x)
         if math.abs(ori) < min_ori_abs then
            min_ori_abs = math.abs(ori)
            min_ori = ori
            target = {x = x, y = y}
         end
      end
   end
   self.skills[1].ori = min_ori
   self.fsm.vars.target = target
end

function DRIVE_SIDEWAYS_TO_PUCK:init()
   local min_x =  1000
   local max_y = -1000
   local candidates = {}
   for _,o in ipairs(omnipucks) do
      if o:visibility_history() >= MIN_VIS_HIST then
         local x = o:translation(0)
         local y = o:translation(1)
         if x > 0 and x < min_x then
            printf("Case 1: %f %f", x, y)
            local new_candidates = {{x = x, y = y}}
            for _,c in ipairs(candidates) do
               if c.x < x + EPSILON then
                  table.insert(new_candidates, c)
               end
            end
            candidates = new_candidates
            min_x = x
         elseif x > 0 and x < min_x + EPSILON then
            printf("Case 2: %f %f", x, y)
            table.insert(candidates, {x = x, y = y})
         end
      end
   end
   local chosen_candidate
   for _,c in ipairs(candidates) do
      if c.y > max_y then
         chosen_candidate = c
         max_y = c.y
      end
   end
   printf("GRAB local: %f,%f", chosen_candidate.x, chosen_candidate.y)
   self.skills[1].y = chosen_candidate.y
   self.fsm.vars.target = chosen_candidate
end

function GRAB:init()
   self.skills[1].x = math.sqrt(self.fsm.vars.target.x^2 + self.fsm.vars.target.y^2) + 0.3
   self.skills[1].vel_trans = 0.25 
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


