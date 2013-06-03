
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
local V_MAX =     { x=0.6,  y=0.6,  ori=1 }
local V_MIN =     { x=0.04, y=0.04, ori=0.15 }
local TOLERANCE = { x=0.02, y=0.02, ori=0.03 }
local D_ACCEL =   { x=0.07, y=0.07, ori=0.15 }

-- Initialize as skill module
skillenv.skill_module(_M)

local tfm = require("tf_module")

function round(x)
  if x%2 ~= 0.5 then
    return math.floor(x+0.5)
  end
  return x-0.5
end

function normalize_mirror_rad(x)
   if x < -math.pi or x >= math.pi then
      return ( x - 2*math.pi*round(x/(2*math.pi)))
   end
   return x
end

function normalize_rad(x)
   if x < 0 or x >= 2*math.pi then
      return x - 2*math.pi * math.floor(x/(2*math.pi))
   end
   return x
end

function invalid_input()
   if fsm.vars.ori and math.abs(fsm.vars.ori) > math.pi then
      return true
   end
   return false
end

function get_ori_diff(ori, is_ori)
    local diff = 0
    if ori > is_ori then
            if ori - is_ori < math.pi then
            diff = ori - is_ori
        else
            diff = -2.0 * math.pi + ori - is_ori
        end
    else
        if is_ori - ori < math.pi then
            diff = ori - is_ori
        else
            diff = 2.0 * math.pi - is_ori + ori;
            end
    end
    return diff
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
        ori = 0 },
      "/odom", "/base_link")

   dist_target.ori = get_ori_diff(self.fsm.vars.odo_target.ori, motor:odometry_orientation())

   dist_start = { x = self.fsm.vars.bl_target.x - dist_target.x,
              y = self.fsm.vars.bl_target.y - dist_target.y,
              ori = get_ori_diff(self.fsm.vars.bl_target.ori, dist_target.ori) }

   -- init velocities with just the sign (i.e. direction)
   local v = { x=1, y=1, ori=1 }
   if dist_target.x < 0 then v.x = -1 end
   if dist_target.y < 0 then v.y = -1 end
   if dist_target.ori < 0 then v.ori = -1 end

   -- a is the scaling factor for the quadratic (de/acc)celeration function
   -- We want:
   -- f(x=D_ACCEL) = V_MAX = a * x^2
   -- Where x is:
   --   The distance dist_start so far if it is less than d_accel
   --   The distance to the target if it is less than d_accel
   --   Where d_accel is:
   --     D_ACCEL if the distance to the target is greater than 2*D_ACCEL
   --     half the distance to the target otherwise.
   local d_accel = D_ACCEL
   local a = { x=0, y=0, ori=0 }
   local v = { x=0, y=0, ori=0 }
   local v_max = V_MAX
   for k, _ in pairs(dist_target) do
      if math.abs(dist_target[k]) > TOLERANCE[k] then
         --if self.fsm.vars.bl_target[k] < 2*D_ACCEL then
         --   d_accel[k] = math.abs(self.fsm.vars.bl_target[k]/2)
         --end

         if d_accel[k] > 0 then a[k] = V_MAX[k] / d_accel[k] end

         v_acc = a[k]*5 * math.abs(dist_start[k]) + V_MIN[k]
         v_dec = a[k]/5 * math.abs(dist_target[k])

         v[k] = math.max(V_MIN[k], math.min(V_MAX[k], v_acc, v_dec))

         printf("%s: v_acc=%f, v_dec=%f, v=%f",k, v_acc, v_dec, v[k])

         if dist_target[k] < 0 then v[k] = v[k] * -1 end

         --if v[k] < 0.04 then v[k] = 0
         --else
         --end
      end
   end
   
   printf("a=(%f,%f,%f), dist_start=(%f,%f,%f), dist_target=(%f,%f,%f)",
      a.x, a.y, a.ori, dist_start.x, dist_start.y, dist_start.ori,
      dist_target.x, dist_target.y, dist_target.ori)

   return v
end

function drive_done()
   return fsm.vars.speed.x == 0
      and fsm.vars.speed.y == 0
end

function turn_done()
   return fsm.vars.speed.ori == 0
end

fsm:define_states{ export_to=_M,
   {"DRIVE", JumpState},
   {"TURN", JumpState}
}

fsm:add_transitions{
   closure={motor=motor},
   {"DRIVE", "FAILED", cond=invalid_input, precond=true, desc="ori must be < pi"},
--   {"DRIVE", "FAILED", cond="not motor:has_writer()", precond=true},
   {"DRIVE", "TURN", cond=drive_done},
   {"TURN", "FINAL", cond=turn_done}
}

function DRIVE:init()
   local x = self.fsm.vars.x or 0
   local y = self.fsm.vars.y or 0
   local ori = self.fsm.vars.ori or 0
   if ori == math.pi then ori = ori - 1e-6 end
   if ori == -math.pi then ori = ori + 1e-6 end
   self.fsm.vars.bl_target = { x=x, y=y, ori=0 }
   self.fsm.vars.odo_target = tfm.transform({x=x, y=y, ori=0}, "/base_link", "/odom")
   local v = set_speed(self)
   send_transrot(v.x, v.y, v.ori)
   self.fsm.vars.speed = v

   local odo_ori = motor:odometry_orientation()
   if odo_ori + ori > math.pi then
      self.fsm.vars.tgt_ori = odo_ori + ori - 2*math.pi
   elseif odo_ori + ori < -math.pi then
      self.fsm.vars.tgt_ori = odo_ori + ori + 2*math.pi
   else
      self.fsm.vars.tgt_ori = odo_ori + ori
   end
   printf("===================== %f", self.fsm.vars.tgt_ori)
end

function DRIVE:loop()
   local v = set_speed(self)
   send_transrot(v.x, v.y, v.ori)
   self.fsm.vars.speed = v
end

function TURN:init()
   local ori = self.fsm.vars.ori or 0
   if ori == math.pi then ori = ori - 1e-6 end
   if ori == -math.pi then ori = ori + 1e-6 end
   self.fsm.vars.bl_target = { x=0, y=0, ori=ori }
   self.fsm.vars.odo_target = tfm.transform({x=0, y=0, ori=0}, "/base_link", "/odom")
   
   self.fsm.vars.odo_target.ori = self.fsm.vars.tgt_ori

   local v = set_speed(self)
   send_transrot(v.x, v.y, v.ori)
   self.fsm.vars.speed = v
end

function TURN:loop()
   local v = set_speed(self)
   send_transrot(v.x, v.y, v.ori)
   self.fsm.vars.speed = v
end

function DRIVE:exit()
   send_transrot(0, 0, 0)
end

function DRIVE:exit()
   send_transrot(0, 0, 0)
end

