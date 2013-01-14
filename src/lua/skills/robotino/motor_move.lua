
----------------------------------------------------------------------------
--  motor_move.lua - stupidly move to some odometry position
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2012 Victor Mataré
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

-- Constants

-- Initialize as skill module
skillenv.skill_module(_M)

fsm:define_states{ export_to=_M,
   {"DRIVE", JumpState},
   {"TURN", JumpState},
   {"STOP", JumpState}
}

local tfm = require("tf_module")

function target_reached(self)
   if math.abs(self.fsm.vars.dx) < 0.03 and math.abs(self.fsm.vars.dy) < 0.03 then
      return true
   end

--   printf("x: " .. self.fsm.vars.dx .. "\t" .. self.fsm.vars.motor_vx)
--   printf("y: " .. self.fsm.vars.dy .. "\t" .. self.fsm.vars.motor_vy)

   return false
end

function invalid_input()
   if fsm.vars.ori and math.abs(fsm.vars.ori) > math.pi then
      return true
   end
   return false
end

function ori_reached(self)
   if math.abs(self.fsm.vars.dist_ori) < 0.03 then
      return true
   end
   return false
end

fsm:add_transitions{
   closure={motor=motor},
   {"DRIVE", "FAILED", cond=invalid_input, precond=true, desc="ori must be < pi"},
   {"DRIVE", "TURN", cond=target_reached},
   {"TURN", "STOP", cond=ori_reached},
   {"STOP", "FINAL", cond=true}
}

function calc_status(self)

   local t_bl = tfm.transform({x=fsm.vars.motor_target_x, y=fsm.vars.motor_target_y, ori=0}, "/odom", "/base_link") 

   self.fsm.vars.dx = t_bl.x
   self.fsm.vars.dy = t_bl.y

   self.fsm.vars.dir_x = 0
   self.fsm.vars.dir_y = 0
   if self.fsm.vars.dx > 0 then
      self.fsm.vars.dir_x = 1
   elseif self.fsm.vars.dx < 0 then
      self.fsm.vars.dir_x = -1
   end
   if self.fsm.vars.dy > 0 then
      self.fsm.vars.dir_y = 1
   elseif self.fsm.vars.dy < 0 then
      self.fsm.vars.dir_y = -1
   end
end

function DRIVE:loop()

   calc_status(self)

   if math.abs(self.fsm.vars.dx) < 0.07 then
      self.fsm.vars.motor_vx = self.fsm.vars.dir_x * 0.04
      if math.abs(self.fsm.vars.dx) < 0.03 then
         self.fsm.vars.motor_vx = 0
      end
   end
   if math.abs(self.fsm.vars.dy) < 0.07 then
      self.fsm.vars.motor_vy = self.fsm.vars.dir_y * 0.04
      if math.abs(self.fsm.vars.dy) < 0.03 then
         self.fsm.vars.motor_vy = 0
      end
   end

   send_transrot(self.fsm.vars.motor_vx, self.fsm.vars.motor_vy, 0)
end

function TURN:init()
   self.fsm.vars.dist_ori = get_ori_diff(self.fsm.vars.motor_target_ori, motor:odometry_orientation())
   send_transrot(0, 0, 0)
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

function TURN:loop()
   local dir_ori = 0
   self.fsm.vars.dist_ori = get_ori_diff(self.fsm.vars.motor_target_ori, motor:odometry_orientation())
--   printf("ori=%f\t%f", motor:odometry_orientation(), self.fsm.vars.motor_omega)

   if self.fsm.vars.dist_ori < 0 then
      dir_ori = -1
   else
      dir_ori = 1
   end

   self.fsm.vars.motor_omega = 0.3 * dir_ori
   if math.abs(self.fsm.vars.dist_ori) < 0.2 then
      self.fsm.vars.motor_omega = dir_ori * 0.1
      if math.abs(self.fsm.vars.dist_ori) < 0.03 then
         self.fsm.vars.motor_omega = 0
      end
   end
   send_transrot(0, 0, self.fsm.vars.motor_omega)
end

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function DRIVE:init()
   local x = self.fsm.vars.x or 0
   local y = self.fsm.vars.y or 0
   local ori = self.fsm.vars.ori or 0
   if ori == math.pi then ori = ori - 1e-6 end
   if ori == -math.pi then ori = ori + 1e-6 end
   
   local odo_tgt = tfm.transform({x=x, y=y, ori=ori}, "/base_link", "/odom")

   self.fsm.vars.motor_target_x = odo_tgt.x
   self.fsm.vars.motor_target_y = odo_tgt.y

   printf("odo_tgt=%f, odo_now=%f", odo_tgt.ori, motor:odometry_orientation())

   self.fsm.vars.motor_vx = 0
   self.fsm.vars.motor_vy = 0

    local odo_ori = motor:odometry_orientation()
   if odo_ori + ori > math.pi then
      self.fsm.vars.motor_target_ori = odo_ori + ori - 2*math.pi
   elseif odo_ori + ori < -math.pi then
      self.fsm.vars.motor_target_ori = odo_ori + ori + 2*math.pi
   else
      self.fsm.vars.motor_target_ori = odo_ori + ori
   end
    
   if ori < 0 then
      self.fsm.vars.dir_ori = -1
   else
      self.fsm.vars.dir_ori = 1
   end

   calc_status(self)

   if self.fsm.vars.dx ~= 0 then
      self.fsm.vars.motor_vx = self.fsm.vars.dir_x * 0.1
   end
   if self.fsm.vars.dy ~= 0 then
      self.fsm.vars.motor_vy = self.fsm.vars.dir_y * 0.1
   end
   if self.fsm.vars.motor_target_ori - motor:odometry_orientation() > 0 then
      self.fsm.vars.motor_omega = 0.3
   elseif self.fsm.vars.motor_target_ori - motor:odometry_orientation() < 0 then
      self.fsm.vars.motor_omega = -0.3
   end

   send_transrot(self.fsm.vars.motor_vx, self.fsm.vars.motor_vy, 0)
end

function STOP:init()
   send_transrot(0,0,0)
end
   
