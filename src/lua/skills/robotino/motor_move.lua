
----------------------------------------------------------------------------
--  motor_move.lua - stupidly move to some odometry position
--
--  Copyright  2013 The Carologistics Team
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
name               = "motor_move"
fsm                = SkillHSM:new{name=name, start="DRIVE"}
depends_skills     = nil
depends_interfaces = {
    {v = "motor", type = "MotorInterface", id="Robotino" }
}

documentation      = [==[Move somewhere using the motor and odometry directly
@param x The target X coordinate, relative to base frame
@param y dito
@param ori relative rotation. -pi <= ori <= pi.
]==]

-- Tunables
local TRANS_TOLERANCE = 0.03
local ORI_TOLERANCE = 0.03
local V_MAX = 0.3
local OMEGA_MAX = 0.6

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("tf_module")

function invalid_input()
   if fsm.vars.ori and math.abs(fsm.vars.ori) > math.pi then
      return true
   end
   return false
end

function send_transrot(vx, vy, omega)
   printf("vx=%f, vy=%f, omega=%f", vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function set_speed()
   diff = tfm.transform(
      { x   = fsm.vars.odo_target.x,
        y   = fsm.vars.odo_target.y,
        ori = fsm.vars.odo_target.ori },
      "/odom", "/base_link")
   
   -- init velocities with just the sign (i.e. direction)
   local vx=1
   local vy=1
   local omega=1
   if diff.x < 0 then vx = -1 end
   if diff.y < 0 then vy = -1 end
   if diff.ori < 0 then omega = -1 end

   local dx = math.abs(diff.x)
   local dy = math.abs(diff.y)
   local dori = math.abs(diff.ori)

   -- then scale velocities according to distance from target
   if true then
      if dx > TRANS_TOLERANCE then
         if dx > 2*TRANS_TOLERANCE then
            vx = vx * math.min(V_MAX, dx)
         else
            vx = vx * 0.04
         end
      else
         vx = 0
      end

      if dy > TRANS_TOLERANCE then
         if dy > 2*TRANS_TOLERANCE then
            vy = vy * math.min(V_MAX, dy)
         else
            vy = vy * 0.04
         end
      else
         vy = 0
      end

      if dori > ORI_TOLERANCE then
         if dori > 2*TRANS_TOLERANCE then
            omega = omega * math.min(OMEGA_MAX, dori)
         else
            omega = omega * 0.04
         end
      else
         omega = 0
      end
   else
      vx = math.min(V_MAX, 50 * diff.x ^ 2)
      if vx < 0.04 then vx = 0
      else if diff.x < 0 then vx = vx * -1 end
      end

      vy = math.min(V_MAX, 50 * diff.y ^ 2)
      if vy < 0.04 then vy = 0
      else if diff.y < 0 then vy = vy * -1 end
      end

      omega = math.min(OMEGA_MAX, 50 * diff.ori ^ 2)
      if omega < 0.04 then omega = 0
      else if diff.ori < 0 then omega = omega * -1 end
      end
   end
   
   send_transrot(vx, vy, omega)

   fsm.vars.speed = {vx=vx, vy=vy, omega=omega}
end

function target_reached()
   return fsm.vars.speed.vx == 0
      and fsm.vars.speed.vy == 0
      and fsm.vars.speed.omega == 0
end


fsm:define_states{ export_to=_M,
   {"DRIVE", JumpState},
}

fsm:add_transitions{
   closure={motor=motor},
   {"DRIVE", "FAILED", cond=invalid_input, precond=true, desc="ori must be < pi"},
   {"DRIVE", "FAILED", cond="not motor:has_writer()", precond=true},
   {"DRIVE", "FINAL", cond=target_reached},
}

function DRIVE:init()
   local x = self.fsm.vars.x or 0
   local y = self.fsm.vars.y or 0
   local ori = self.fsm.vars.ori or 0
   if ori == math.pi then ori = ori - 1e-6 end
   if ori == -math.pi then ori = ori + 1e-6 end
   
   self.fsm.vars.odo_target = tfm.transform({x=x, y=y, ori=ori}, "/base_link", "/odom")
   
   set_speed()
end

function DRIVE:loop()
   set_speed()
end

