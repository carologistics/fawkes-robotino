
----------------------------------------------------------------------------
--  fetch_puck.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2014  Tobias Neumann
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
fsm                = SkillHSM:new{name=name, start="START", debug=true}
depends_skills     = { "motor_move" }
depends_interfaces = {
--    {v = "omnivisionSwitch", type="SwitchInterface", id="omnivisionSwitch"},
    {v = "puck_0", type="Position3DInterface", id="puck_0"},
    {v = "puck_1", type="Position3DInterface", id="puck_1"},
    {v = "puck_2", type="Position3DInterface", id="puck_2"},
    {v = "sensor", type="RobotinoSensorInterface"},
    {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[Move to puck pickup position]==]
-- Initialize as skill module
skillenv.skill_module(_M)

local TIMEOUT = 3
local ORI_OFFSET = 0.03
local MIN_VIS_HIST = 1
local THRESHOLD_DISTANCE = 0.07
if config:exists("/hardware/robotino/puck_sensor/trigger_dist") then
   THRESHOLD_DISTANCE = config:get_float("/hardware/robotino/puck_sensor/trigger_dist")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/trigger_dist FOUND! Using default value\n");
end
local PUCK_SENSOR_INDEX = 8
if config:exists("/hardware/robotino/puck_sensor/index") then
   -- you can find the config value in /cfg/host.yaml
   PUCK_SENSOR_INDEX = config:get_uint("/hardware/robotino/puck_sensor/index")
else
   printf("NO CONFIG FOR /hardware/robotino/puck_sensor/index FOUND! Using default value\n");
end
local EPSILON_X = 0.07                                                         -- allowed distance for pucks to be considered in first row
local EPSILON_PHI = 0.2                                                        -- allowed offset in angle to be considert in front
local OFFSET_X_SIDE_SEARCH = 0.30                                              -- offset to ignore pucks to close to the robot/grabber (baser_link -> grabber-top ~ 0.3m)
local pucks = {
   puck_0,
   puck_1,
   puck_2,
}


local tfm = require 'tf_module'

function have_puck()
    local curDistance = sensor:distance(PUCK_SENSOR_INDEX)
    if (curDistance > 0) and (curDistance <= THRESHOLD_DISTANCE) then
        printf("near: " .. curDistance)
        return true
    end
    return false
end

function visible()
   for _,o in ipairs(pucks) do
      if o:visibility_history() >= MIN_VIS_HIST then
         return true
      end
   end
   return false
end

function cant_see_puck()
   local cant_see_puck=true
   for _,o in ipairs(pucks) do
      if o:visibility_history() >= MIN_VIS_HIST then
         cant_see_puck=false
      end
   end
   return cant_see_puck
end

fsm:define_states{ export_to=_M, closure={have_puck=have_puck, visible=visible},
   {"START", JumpState},
   {"DRIVE_SIDEWAYS_TO_PUCK", SkillJumpState, skills={{motor_move}}, final_to="TURN_TO_PUCK", fail_to="START"},
   {"TURN_TO_PUCK", SkillJumpState, skills={{motor_move}}, final_to="GRAB", fail_to="START"},
   {"GRAB", SkillJumpState, skills={{motor_move}}, final_to="GRAB_DONE", fail_to="FAILED"},
   {"GRAB_DONE", JumpState},
   {"MOVE_MORE", SkillJumpState, skills={{motor_move}}, final_to="FINAL", fail_to="FAILED"},

}

fsm:add_transitions{
   {"START", "TURN_TO_PUCK", cond="visible() and vars.search_cycle" },
   {"START", "DRIVE_SIDEWAYS_TO_PUCK", cond="visible() and ( vars.search_right or vars.search_left )" },
   {"START", "FAILED", timeout=TIMEOUT},
   {"TURN_TO_PUCK", "FAILED", precond=cant_see_puck, desc="can't see any puck"},
   {"GRAB", "MOVE_MORE", cond=have_puck},
   {"GRAB_DONE", "START", cond="not have_puck()"},
   {"GRAB_DONE", "MOVE_MORE", cond=have_puck},
}

function START:init()
   self.fsm.vars.search_right = false                                          -- decide mode; how to search for the puck
   self.fsm.vars.search_left  = false
   self.fsm.vars.search_cycle = false
   if     self.fsm.vars.move_sideways == "right" then
      self.fsm.vars.search_right = true
   elseif self.fsm.vars.move_sideways == "left" then
      self.fsm.vars.search_left  = true
   else
      self.fsm.vars.search_cycle = true
   end
end

function TURN_TO_PUCK:init()
   local min_d = 10
   local target
   local candidates = {}

   -- search for candidates ( pucks that are in the given angle EPSILON_PHI )
   for _,o in ipairs(pucks) do
      if o:visibility_history() >= MIN_VIS_HIST then
         local x = o:translation(0)
         local y = o:translation(1)
         local ori = math.atan2(y, x)
         if math.abs(ori) < EPSILON_PHI then
            table.insert(candidates,  {x = x, y = y})
         end
      end
   end

   if #candidates == 0 then                                                    -- if there are no canidates in the area
      for _,o in ipairs(pucks) do                                              -- add all pucks with an high vis_hist
         if o:visibility_history() >= MIN_VIS_HIST then
            local x = o:translation(0)
            local y = o:translation(1)
            table.insert(candidates, {x = x, y = y})
         end
      end
   end

   -- choose closest candidate (distance)
   for _,o in ipairs(candidates) do
      local d = math.sqrt(o.y * o.y + o.x * o.x)
      if d < min_d then
         min_d = d
         target = o
      end
   end

   self.skills[1].ori = math.atan2(target.y, target.x)
   self.skills[1].tolerance = { x=0.05, y=0.05, ori=0.04 }
   self.fsm.vars.target = target
end

function DRIVE_SIDEWAYS_TO_PUCK:init()
   local min_x =  1000
   local max_y = -1000
   local min_y =  1000
   local candidates = {}                                                       -- pucks that are been considert to be in the first row

   -- search for pucks that are be considert to be in the first row
   for _,o in ipairs(pucks) do
      if o:visibility_history() >= MIN_VIS_HIST then
         local x = o:translation(0)
         local y = o:translation(1)
         if x > OFFSET_X_SIDE_SEARCH and x < min_x then                        -- if the new puck is not too close AND closer than pucks before
            printf("Case 1: %f %f", x, y)                                         -- then recheck pucks and create new candidates list
            local new_candidates = {{x = x, y = y}}
            for _,c in ipairs(candidates) do                                      -- just check for the candidates ( pucks that weren't candidates before are eaven more far away now ) if they still be considert as candidates
               if c.x < x + EPSILON_X then
                  table.insert(new_candidates, c)
               end
            end
            candidates = new_candidates
            min_x = x
         elseif x > OFFSET_X_SIDE_SEARCH and x < min_x + EPSILON_X then        -- if the puck is not too close AND close enough to be considert as to be in first row
            printf("Case 2: %f %f", x, y)
            table.insert(candidates, {x = x, y = y})
         end
      end
   end

   -- search for the puck that is most far in (x or y direction, given by the parameter)
   local chosen_candidate
   for _,c in ipairs(candidates) do
      if self.fsm.vars.search_left  == true and c.y > max_y
      or self.fsm.vars.search_right == true and c.y < min_y then
         chosen_candidate = c
         max_y = c.y
         min_y = c.y
      end
   end

   -- decide where to drive to
   if chosen_candidate ~= nil then
      printf("GRAB local: %f,%f", chosen_candidate.x, chosen_candidate.y)
      self.skills[1].y = chosen_candidate.y
--      self.fsm.vars.target = chosen_candidate
   else
      printf("NO PUCK IN FRONT")
      self.skills[1].y = 0.0
   end
end

function GRAB:init()
   self.skills[1].x = math.sqrt(self.fsm.vars.target.x^2 + self.fsm.vars.target.y^2) - 0.15
   self.skills[1].vel_trans = 0.25
   self.skills[1].tolerance = { x=0.01, y=0.01, ori=0.05 }
end

function MOVE_MORE:init()
   self.skills[1].tolerance = { x=0.01, y=0.01, ori=0.05 }
   self.skills[1].x = 0.05
end

function cleanup()
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(0, 0, 0))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
   --local msg = omnivisionSwitch.DisableSwitchMessage:new()
   --omnivisionSwitch:msgq_enqueue_copy(msg)
end

function FAILED:init() cleanup() end

function FINAL:init() cleanup() end

