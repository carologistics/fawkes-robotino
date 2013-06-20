
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

documentation      = [==[Move on a (kind of) straight line relative to /base_link.
@param x The target X coordinate
@param y Dito
@param ori Relative rotation. -pi <= ori <= pi.
@param frame Measure distances relative to this frame. Can be "/map" or "/odom" (default).
@param vel_trans Translational top-speed. Upper limit: hardcoded tunable in skill module.
@param vel_rot Rotational top-speed. Upper limit: dito.
]==]

-- Tunables
local V_MAX =     { x=0.4,  y=0.4,  ori=2.2 }    -- ultimate limit
local V_MIN =     { x=0.04, y=0.04, ori=0.15 }   -- below the motor won't even start
local TOLERANCE = { x=0.04, y=0.04, ori=0.05 } -- accuracy
local D_DECEL =   { x=0.07, y=0.07, ori=0.2 }    -- deceleration distance
local ACCEL =     { x=0.08, y=0.08, ori=0.15 }   -- accelerate by this factor every loop

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
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function set_speed(self)
   dist_target = tfm.transform(
      { x   = self.fsm.vars.target.x,
        y   = self.fsm.vars.target.y,
        ori = self.fsm.vars.target.ori },
      self.fsm.vars.frame, "/base_link")

   local v = { x=1, y=1, ori=1 }
   local a = { x=0, y=0, ori=0 }

   for k, _ in pairs(dist_target) do
      if math.abs(dist_target[k]) > self.fsm.vars.tolerance_arg[k] then
         if D_DECEL[k] > 0 then a[k] = V_MAX[k] / D_DECEL[k] end

         -- speed if we're accelerating.
         -- We cannot accelerate based on the distance from the starting
         -- point since odometry tends to lag by 0.5 seconds when accelerating,
         -- i.e. the distance driven will stay at 0 for a short while.
         v_acc = self.fsm.vars.cycle * ACCEL[k]

         -- speed if we're decelerating
         v_dec = a[k]/5 * math.abs(dist_target[k])
         
         -- decide if we wanna decelerate, accelerate or max out
         v[k] = math.min(
            self.fsm.vars.vmax_arg[k],
            math.max(V_MIN[k], math.min(V_MAX[k], v_acc, v_dec))
         )

         -- finally reverse if the target is behind us
         -- hopefully the deceleration function(dist_target[k])
         -- slowed us down a bit before doing this ;-)
         if dist_target[k] < 0 then v[k] = v[k] * -1 end

         -- printf("%s: d_t=%f, v_acc=%f, v_dec=%f, v=%f",
         --  k, dist_target[k], v_acc, v_dec, v[k])
      else
         v[k] = 0
      end
   end

   if self.fsm.vars.puck and v.x < 0 then v.x = 0 end
   
   self.fsm.vars.cycle = self.fsm.vars.cycle + 1

   send_transrot(v.x, v.y, v.ori)
   self.fsm.vars.speed = v
end

function drive_done()
   return fsm.vars.speed.x == 0
      and fsm.vars.speed.y == 0
      and fsm.vars.speed.ori == 0
end

fsm:define_states{ export_to=_M,
   {"DRIVE", JumpState},
}

fsm:add_transitions{
   closure={motor=motor},
   {"DRIVE", "FAILED", cond=invalid_input, precond=true, desc="ori must be < pi"},
--   {"DRIVE", "FAILED", cond="not motor:has_writer()", precond=true},
   {"DRIVE", "FINAL", cond=drive_done},
}

function DRIVE:init()
   local x = self.fsm.vars.x or 0
   local y = self.fsm.vars.y or 0
   local ori = self.fsm.vars.ori or 0
   
   if ori == math.pi then ori = ori - 1e-6 end
   if ori == -math.pi then ori = ori + 1e-6 end

   local frame = self.fsm.vars.frame or "/odom"
   self.fsm.vars.frame = frame

   self.fsm.vars.cycle = 0
   
   self.fsm.vars.target = tfm.transform(
      { x=x, y=y, ori=ori },
      "/base_link",
      self.fsm.vars.frame
   )
   if self.fsm.vars.global then
      -- Overwrite tf'd coords with values from arguments
      if self.fsm.vars.x then self.fsm.vars.target.x = self.fsm.vars.x end
      if self.fsm.vars.y then self.fsm.vars.target.y = self.fsm.vars.y end
      if self.fsm.vars.ori then self.fsm.vars.target.ori = self.fsm.vars.ori end
   end
  
   local vmax_arg = self.fsm.vars.vel_trans or math.max(V_MAX.x, V_MAX.y)
   local vmin_rot = self.fsm.vars.vel_rot or V_MAX.ori
   self.fsm.vars.vmax_arg = { x=vmax_arg, y=vmax_arg, ori=vmin_rot }

   self.fsm.vars.speed = { x=0, y=0, ori=0 }
   self.fsm.vars.tolerance_arg = self.fsm.vars.tolerance or TOLERANCE

   set_speed(self)
end

function DRIVE:loop()
   set_speed(self)
end

function DRIVE:exit()
   send_transrot(0, 0, 0)
end

