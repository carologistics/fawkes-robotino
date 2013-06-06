
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
local V_MAX =     { x=0.8,  y=0.8,  ori=1.5 }
local V_MIN =     { x=0.04, y=0.04, ori=0.15 }
local TOLERANCE = { x=0.01, y=0.01, ori=0.03 }
local D_ACCEL =   { x=0.07, y=0.07, ori=0.05 }
local ACCEL =     { x=0.08, y=0.08, ori=0.15 }

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
      { x   = self.fsm.vars.odo_target.x,
        y   = self.fsm.vars.odo_target.y,
        ori = self.fsm.vars.odo_target.ori },
      self.fsm.vars.frame, "/base_link")

   --dist_target.ori = get_ori_diff(self.fsm.vars.odo_target.ori, motor:odometry_orientation())

   local v = { x=1, y=1, ori=1 }
   local a = { x=0, y=0, ori=0 }

   for k, _ in pairs(dist_target) do
      if math.abs(dist_target[k]) > TOLERANCE[k] then
         if D_ACCEL[k] > 0 then a[k] = V_MAX[k] / D_ACCEL[k] end

         -- speed if we're accelerating.
         -- We cannot accelerate based on the distance from the starting
         -- point since odometry tends to lag by 0.5 seconds when accelerating,
         -- i.e. the distance driven will stay at 0 for a short while.
         v_acc = self.fsm.vars.cycle * ACCEL[k]

         -- speed if we're decelerating
         v_dec = a[k]/5 * math.abs(dist_target[k])
         
         -- decide if we wanna decelerate, accelerate or max out
         v[k] = math.min(
            self.fsm.vars.vmin_arg[k],
            math.max(V_MIN[k], math.min(V_MAX[k], v_acc, v_dec))
         )

         -- finally reverse if the target is behind us
         -- hopefully the deceleration function(dist_target[k])
         -- slowed us down a bit before doing this ;-)
         if dist_target[k] < 0 then v[k] = v[k] * -1 end

         printf("%s: d_t=%f, v_acc=%f, v_dec=%f, v=%f",
            k, dist_target[k], v_acc, v_dec, v[k])
      else
         v[k] = 0
      end
   end
   
--   printf("a=(%f,%f,%f), dist_start=(%f,%f,%f), dist_target=(%f,%f,%f)",
--      a.x, a.y, a.ori, dist_start.x, dist_start.y, dist_start.ori,
--      dist_target.x, dist_target.y, dist_target.ori)
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
   
   self.fsm.vars.bl_target = { x=x, y=y, ori=ori }
   self.fsm.vars.odo_target = tfm.transform(
      { x=x, y=y, ori=ori },
      "/base_link",
      self.fsm.vars.frame
   )
  
   local vmin_arg = self.fsm.vars.vel_trans or math.max(V_MAX.x, V_MAX.y)
   local vmin_rot = self.fsm.vars.vel_rot or V_MAX.ori
   self.fsm.vars.vmin_arg = { x=vmin_arg, y=vmin_arg, ori=vmin_rot }

   printf("%f, %f", self.fsm.vars.vel_trans, self.fsm.vars.vel_rot)

   self.fsm.vars.speed = { x=0, y=0, ori=0 }

   set_speed(self)
end

function DRIVE:loop()
   set_speed(self)
end

function DRIVE:exit()
   send_transrot(0, 0, 0)
end

