
----------------------------------------------------------------------------
--  motor_move.lua - stupidly move to some odometry position
--
--  Copyright  2014 Victor Mataré
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
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = nil
depends_interfaces = {
   {v = "motor", type = "MotorInterface", id="Robotino" },
   {v = "navigator", type="NavigatorInterface", id="Navigator"},
   {v = "tag_0", type = "Position3DInterface", id="/tag-vision/0"},
   {v = "tag_1", type = "Position3DInterface", id="/tag-vision/1"},
   {v = "tag_2", type = "Position3DInterface", id="/tag-vision/2"},
   {v = "tag_3", type = "Position3DInterface", id="/tag-vision/3"},
   {v = "tag_4", type = "Position3DInterface", id="/tag-vision/4"},
   {v = "tag_5", type = "Position3DInterface", id="/tag-vision/5"},
   {v = "tag_6", type = "Position3DInterface", id="/tag-vision/6"},
   {v = "tag_7", type = "Position3DInterface", id="/tag-vision/7"},
   {v = "tag_8", type = "Position3DInterface", id="/tag-vision/8"},
   {v = "tag_9", type = "Position3DInterface", id="/tag-vision/9"},
   {v = "tag_10", type = "Position3DInterface", id="/tag-vision/10"},
   {v = "tag_11", type = "Position3DInterface", id="/tag-vision/11"},
   {v = "tag_12", type = "Position3DInterface", id="/tag-vision/12"},
   {v = "tag_13", type = "Position3DInterface", id="/tag-vision/13"},
   {v = "tag_14", type = "Position3DInterface", id="/tag-vision/14"},
   {v = "tag_15", type = "Position3DInterface", id="/tag-vision/15"},
   {v = "if_front_dist", type = "Position3DInterface", id="front_dist"},
   {v = "object_tracking_if", type = "ObjectTrackingInterface", id="object-tracking"},
}

documentation      = [==[Move on a (kind of) straight line to the given coordinates.
@param x (Optional) The target X coordinate, relative to base_link or to the frame argument (if specified)
@param y (Optional) Dito
@param ori (Optional) Rotation. -math.pi <= ori <= math.pi
@param frame (Optional) Reference frame for input coordinates. Defaults to base_link.
@param vel_trans (Optional) Translational top-speed. Upper limit: hardcoded tunable in skill module.
@param vel_rot (Optional) Rotational top-speed. Upper limit: dito.
@param visual_servoing (Optional) Updates the target coordinates based on object tracking data.
]==]

-- Tunables
local V_MAX =         { x=0.35, y=0.35, ori=1.4 }    -- ultimate limit
local V_MAX_CAM =     { x=0.06, y=0.06, ori=0.3 }
local V_MIN =         { x=0.006, y=0.006, ori=0.02 }   -- below the motor won't even start
local TOLERANCE =     { x=0.02, y=0.02, ori=0.025 } -- accuracy
local TOLERANCE_VS = { x=0.005, y=0.0015, ori=0.01 }
local TOLERANCE_CAM = { x=0.005, y=0.0015, ori=0.01 }
local D_DECEL =       { x=0.035, y=0.035, ori=0.15 }    -- deceleration distance
local ACCEL =         { x=0.06, y=0.06, ori=0.21 }   -- accelerate by this factor every loop
local MONITOR_LEN     = 15   -- STUCK monitor: Watch distance moved over this many loops
local STUCK_MAX       = 120  -- STUCK timeout: Fail after being stuck for this many loops
local STUCK_THRESHOLD = 0.6  -- STUCK threshold: Consider ourselves stuck if we moved less than
                             --                  this factor times V_MIN speed during the
                             --                  last MONITOR_LEN loops
local MISSING_MAX     = 3    -- limit for missing object detections in a row

-- Initialize as skill module
skillenv.skill_module(_M )

local tfm = require("fawkes.tfutils")

local laser_front_dist_frame = "/motor_move_frame_empty"
if config:exists("/plugins/laser-front-dist/target_frame") then
   laser_front_dist_frame = config:get_string("/plugins/laser-front-dist/target_frame")
end


function invalid_params(self)
   return self.fsm.vars.ori <= -2 * math.pi or self.fsm.vars.ori >= 2 * math.pi
      or self.fsm.vars.frame == "/base_link"
end

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue(motor.AcquireControlMessage:new())
   motor:msgq_enqueue(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue(motor.AcquireControlMessage:new(oc, ocn))
end

function scalar(x)
   if tfm.is_quaternion(x) then
      return fawkes.tf.get_yaw(x)
   else
      return x
   end
end


function set_speed(self)
   local v = {}

   local dist_target = tfm.transform6D(
      self.fsm.vars.target,
      self.fsm.vars.target_frame, "/base_link")

   if not dist_target then
      self.fsm.vars.tf_failed = true
      v = { x=0, y=0, ori=0 }
   else
      local d_ori = fawkes.tf.get_yaw(dist_target.ori)
      --[[ if self.fsm.vars.ori > math.pi and d_ori < -self.fsm.vars.tolerance_arg.ori then
         d_ori = 2*math.pi + d_ori
         dist_target.ori = fawkes.tf.Quaternion:new(0, d_ori, 0)
      elseif self.fsm.vars.ori < -math.pi and d_ori > self.fsm.vars.tolerance_arg.ori then
         d_ori = -2*math.pi + d_ori
         dist_target.ori = fawkes.tf.Quaternion:new(0, d_ori, 0)
      end ]]--

      local a = { x=0, y=0, ori=0 }

      self.fsm.vars.monitor_idx = (self.fsm.vars.monitor_idx % MONITOR_LEN) + 1
      self.fsm.vars.moved_dist[self.fsm.vars.monitor_idx] = { x=0, y=0, ori=0 }

      for k, _ in pairs(dist_target) do
         -- Ignore z axis: no way to move up & down in /base_link!
         if k ~= "z" then
            local delta_dist = math.abs(self.fsm.vars.last_dist_target[k] - scalar(dist_target[k]))
            self.fsm.vars.moved_dist[self.fsm.vars.monitor_idx][k] = delta_dist
            self.fsm.vars.last_dist_target[k] = scalar(dist_target[k])

            if math.abs(scalar(dist_target[k])) > self.fsm.vars.tolerance_arg[k] then
               if D_DECEL[k] > 0 then a[k] = V_MAX[k] / D_DECEL[k] end

               -- speed if we're accelerating.
               -- We cannot accelerate based on the distance from the starting
               -- point since odometry tends to lag by 0.5 seconds when accelerating,
               -- i.e. the distance driven will stay at 0 for a short while.
               v_acc = self.fsm.vars.cycle * ACCEL[k]

               -- speed if we're decelerating
               v_dec = a[k]/self.fsm.vars.decel_factor * math.abs(scalar(dist_target[k]))
               
               -- decide if we wanna decelerate, accelerate or max out
               v[k] = math.min(
                  self.fsm.vars.vmax_arg[k],
                  math.max(V_MIN[k], math.min(V_MAX[k], v_acc, v_dec))
               )

               if #self.fsm.vars.moved_dist == MONITOR_LEN then
                  local dist_sum = 0
                  for i = 1,MONITOR_LEN do
                     dist_sum = dist_sum + self.fsm.vars.moved_dist[i][k]
                  end
                  if dist_sum < STUCK_THRESHOLD * (V_MIN[k] + self.fsm.vars.tolerance_arg[k]) then
                     self.fsm.vars.stuck_count = self.fsm.vars.stuck_count + 1
                     v[k] = v[k] + self.fsm.vars.stuck_count * 0.5 * V_MIN[k]
                     printf("motor_move: STUCK #%d: increasing speed by %f.",
                        self.fsm.vars.stuck_count,
                        self.fsm.vars.stuck_count * V_MIN[k])
                  end
               end

               -- finally reverse if the target is behind us
               -- hopefully the deceleration function(dist_target[k])
               -- slowed us down a bit before doing this ;-)
               if scalar(dist_target[k]) < 0 then v[k] = v[k] * -1 end
            else
               v[k] = 0
            end
         end
      end
   end

   if self.fsm.vars.frame and self.fsm.vars.target_frame ~= "/odom" then 
      -- save target in odom for fallback
      local tgt_odom = tfm.transform6D(
         {x=self.fsm.vars.target.x, y=self.fsm.vars.target.y, z=self.fsm.vars.target.z, ori=self.fsm.vars.target.ori},
         self.fsm.vars.target_frame, "/odom")
      if tgt_odom then
         self.fsm.vars.fallback_target_odom = tgt_odom
      end
   end

   
   self.fsm.vars.cycle = self.fsm.vars.cycle + 1
   
   if self.fsm.vars.stuck_count > 0 then
      print_debug("motor_move: dist_target=(%f, %f, %f) V=(%f, %f, %f)", v.x, v.y, v.ori, dist_target.x, dist_target.y,
         scalar(dist_target.ori))
   end
   send_transrot(v.x, v.y, v.ori)
   self.fsm.vars.speed = v
end

function drive_done()
   return fsm.vars.speed.x == 0
      and fsm.vars.speed.y == 0
      and fsm.vars.speed.ori == 0
end

function pos3d_iface(frame)
   -- return Position3DInterface of frame if it can be used for control, otherwise false
   if frame and string.sub(frame, 1, 4) == "/tag" then
      local idx = tonumber(string.sub(frame, 6))
      return fsm.vars.tags[idx+1]
   end
   if frame == laser_front_dist_frame then
      return if_front_dist
   end
   return false
end

function cam_frame_visible(frame)
   pos_iface = pos3d_iface(frame)
   return pos_iface and pos_iface:visibility_history() > 0
end

fsm:define_states{ export_to=_M,
   closure={motor=motor, navigator=navigator, pos3d_iface=pos3d_iface, cam_frame_visible=cam_frame_visible,
      STUCK_MAX=STUCK_MAX, MISSING_MAX=MISSING_MAX},
   {"INIT", JumpState},
   {"DRIVE", JumpState},
   {"DRIVE_CAM", JumpState},
   {"STOP_NAVIGATOR", JumpState},
   {"FALLBACK_TO_ODOM", JumpState},
   {"RECOVER_TO_FRAME", JumpState},
}

fsm:add_transitions{
   {"INIT", "FAILED", cond=invalid_params, desc="|ori| >= 2*PI", desc="invalid params"},
   {"INIT", "FAILED", precond="not motor:has_writer()", desc="No writer for motor"},
   {"INIT", "FAILED", cond="not vars.target", desc="target TF failed"},
   {"INIT", "FAILED", cond="vars.stop_attempts > 5", desc="Navigator won't stop"},
   {"INIT", "STOP_NAVIGATOR", cond="navigator:has_writer() and not navigator:is_final()"},
   {"INIT", "DRIVE_CAM", cond="pos3d_iface(vars.frame)"},
   {"INIT", "DRIVE", cond=true},

   {"STOP_NAVIGATOR", "INIT", cond="navigator:is_final()"},
   
   {"DRIVE", "FAILED", cond="not motor:has_writer()", desc="No writer for motor"},
   {"DRIVE", "FAILED", cond="vars.tf_failed", desc="dist TF failed"},
   {"DRIVE", "FAILED", cond="vars.stuck_count > STUCK_MAX", desc="STUCK"},
   {"DRIVE", "FAILED", cond="vars.missing_detections > MISSING_MAX", desc="Object cannot be found"},
   {"DRIVE", "FINAL", cond=drive_done},

   {"DRIVE_CAM", "FALLBACK_TO_ODOM", cond="not cam_frame_visible(vars.frame)", desc="Lost frame"},
   {"DRIVE_CAM", "FALLBACK_TO_ODOM", cond="vars.tf_failed", desc="dist TF failed"},
   {"DRIVE_CAM", "FAILED", cond="not motor:has_writer()", desc="No writer for motor"},
   {"DRIVE_CAM", "FAILED", cond="vars.stuck_count > STUCK_MAX", desc="STUCK"},
   {"DRIVE_CAM", "FINAL", cond=drive_done},

   {"FALLBACK_TO_ODOM", "DRIVE", cond=true},
   {"DRIVE", "RECOVER_TO_FRAME", cond="cam_frame_visible(vars.frame)", desc="frame visible again"},
   {"RECOVER_TO_FRAME", "FAILED", cond="vars.num_fallbacks > 5", desc="Too many fallbacks"},
   {"RECOVER_TO_FRAME", "DRIVE_CAM", cond=true},
}

function INIT:init()
   self.fsm.vars.msgid = 0
   self.fsm.vars.missing_detections = 0

   self.fsm.vars.tags = { tag_0, tag_1, tag_2, tag_3, tag_4, tag_5, tag_6, tag_7,
      tag_8, tag_9, tag_10, tag_11, tag_12, tag_13, tag_14, tag_15 }

   if self.fsm.vars.puck then
      print("WARNING: motor_move: puck argument is deprecated!")
   end
   
   self.fsm.vars.target_frame = self.fsm.vars.frame or "/odom"

   -- If frame arg is specified, input coordinates are relative to it
   -- If unspecified, input coordinates are relative to /base_link
   self.fsm.vars.arg_frame = self.fsm.vars.frame or "/base_link"

   -- Initialize unspecified arguments with default values
   if self.fsm.vars.frame then
      local cur_pos = tfm.transform6D(
         { x=0, y=0, z=0, ori=fawkes.tf.create_quaternion_from_yaw(0) },
         "/base_link",
         self.fsm.vars.target_frame)
      if not cur_pos then
         print_error("Failed to transfrom base_link to " .. self.fsm.vars.target_frame)
      end
      self.fsm.vars.x = self.fsm.vars.x or cur_pos.x
      self.fsm.vars.y = self.fsm.vars.y or cur_pos.y
      self.fsm.vars.z = self.fsm.vars.z or cur_pos.z
      self.fsm.vars.ori = self.fsm.vars.ori or fawkes.tf.get_yaw(cur_pos.ori)
      print("special motor_move frame given: " .. self.fsm.vars.frame)
   else
      self.fsm.vars.x = self.fsm.vars.x or 0
      self.fsm.vars.y = self.fsm.vars.y or 0
      self.fsm.vars.z = self.fsm.vars.z or 0
      self.fsm.vars.ori = self.fsm.vars.ori or 0
   end
   
   -- Make sure that ori ~= |math.pi|
   if self.fsm.vars.ori == math.pi then self.fsm.vars.ori = self.fsm.vars.ori - 1e-6 end
   if self.fsm.vars.ori == -math.pi then self.fsm.vars.ori = self.fsm.vars.ori + 1e-6 end
   
   self.fsm.vars.qori = fawkes.tf.create_quaternion_from_yaw(self.fsm.vars.ori)
   
   self.fsm.vars.cycle = 0
   self.fsm.vars.stop_attempts = self.fsm.vars.stop_attempts or 0
   self.fsm.vars.num_fallbacks = 0

   self.fsm.vars.target = tfm.transform6D(
      { x=self.fsm.vars.x, y=self.fsm.vars.y, z=self.fsm.vars.z, ori=self.fsm.vars.qori },
      self.fsm.vars.arg_frame, self.fsm.vars.target_frame)
   if not self.fsm.vars.target then
      print_error("Failed to transfrom " .. self.fsm.vars.arg_frame .. " to " .. self.fsm.vars.target_frame)
   end
   self.fsm.vars.last_dist_target = {
      x = self.fsm.vars.target.x,
      y = self.fsm.vars.target.y,
      ori = scalar(self.fsm.vars.target.ori)
   }

   if self.fsm.vars.frame then
      -- save target in odom for fallback
      self.fsm.vars.fallback_target_odom = tfm.transform6D(
         {x=self.fsm.vars.target.x, y=self.fsm.vars.target.y, z=self.fsm.vars.target.z, ori=self.fsm.vars.target.ori},
         self.fsm.vars.target_frame, "/odom")
      if not self.fsm.vars.fallback_target_odom then
         print_error("Failed to transfrom " .. self.fsm.vars.target_frame .. " to odom")
      end
      self.fsm.vars.recover_target_frame = self.fsm.vars.target
      self.fsm.vars.last_dist_target = {
         x = self.fsm.vars.fallback_target_odom.x,
         y = self.fsm.vars.fallback_target_odom.y,
         ori = scalar(self.fsm.vars.fallback_target_odom.ori)
      }
   end

   self.fsm.vars.monitor_idx = 0

   printf("Target %s: %f, %f, %f, %f", self.fsm.vars.target_frame, self.fsm.vars.target.x,
      self.fsm.vars.target.y, self.fsm.vars.target.z, fawkes.tf.get_yaw(self.fsm.vars.target.ori))

   self.fsm.vars.speed = { x=0, y=0, ori=0 }
   self.fsm.vars.moved_dist = { }
   self.fsm.vars.stuck_count = 0
end

function DRIVE:init()
   self.fsm.vars.vmax_arg = {
      x = math.min(V_MAX.x, self.fsm.vars.vel_trans or V_MAX.x),
      y = math.min(V_MAX.y, self.fsm.vars.vel_trans or V_MAX.y),
      ori = math.min(V_MAX.ori, self.fsm.vars.vel_rot or V_MAX.ori)
   }

   if self.fsm.vars.visual_servoing then
      self.fsm.vars.tolerance = self.fsm.vars.tolerance or {}
      self.fsm.vars.tolerance_arg = {
         x = self.fsm.vars.tolerance.x or TOLERANCE_VS.x,
         y = self.fsm.vars.tolerance.y or TOLERANCE_VS.y,
         ori = self.fsm.vars.tolerance.ori or TOLERANCE_VS.ori
      }
   else
      self.fsm.vars.tolerance = self.fsm.vars.tolerance or {}
      self.fsm.vars.tolerance_arg = {
         x = self.fsm.vars.tolerance.x or TOLERANCE.x,
         y = self.fsm.vars.tolerance.y or TOLERANCE.y,
         ori = self.fsm.vars.tolerance.ori or TOLERANCE.ori
      }
   end
   print_info("motor_move tolerance x: %f, y: %f, ori: %f",
      self.fsm.vars.tolerance_arg.x,
      self.fsm.vars.tolerance_arg.y,
      self.fsm.vars.tolerance_arg.ori
   )
   
   -- "Magic", i.e. heuristic multiplier that determines how much we brake
   -- when approaching target. Important to avoid overshooting.
   self.fsm.vars.decel_factor = 5
   
   set_speed(self)
end

function DRIVE:loop()
   if self.fsm.vars.visual_servoing then
      --TODO: set max speed

      if self.fsm.vars.msgid ~= object_tracking_if:msgid() then
         self.fsm.vars.msgid = object_tracking_if:msgid()
         if object_tracking_if:is_detected() then
            self.fsm.vars.missing_detections = 0
         else
            self.fsm.vars.missing_detections = self.fsm.vars.missing_detections + 1
         end
      end

      self.fsm.vars.target.x = object_tracking_if:base_frame(0)
      self.fsm.vars.target.y = object_tracking_if:base_frame(1)
      self.fsm.vars.target.ori = fawkes.tf.create_quaternion_from_yaw(object_tracking_if:base_frame(5))
   end
   set_speed(self)
end

function DRIVE:exit()
   send_transrot(0, 0, 0)
end

function DRIVE_CAM:init()
   self.fsm.vars.vmax_arg = {
      x = math.min(V_MAX_CAM.x, self.fsm.vars.vel_trans or V_MAX_CAM.x),
      y = math.min(V_MAX_CAM.y, self.fsm.vars.vel_trans or V_MAX_CAM.y),
      ori = math.min(V_MAX_CAM.ori, self.fsm.vars.vel_rot or V_MAX_CAM.ori)
   }
   self.fsm.vars.tolerance_arg = self.fsm.vars.tolerance or TOLERANCE_CAM

   -- "Magic", i.e. heuristic multiplier that determines how much we brake
   -- when approaching target. Important to avoid overshooting.
   self.fsm.vars.decel_factor = 10
   
   set_speed(self)
end

function DRIVE_CAM:loop()
   set_speed(self)
end

function DRIVE_CAM:exit()
   send_transrot(0, 0, 0)
end

function STOP_NAVIGATOR:init()
   local msg = navigator.StopMessage:new( )
   navigator:msgq_enqueue(msg)
   self.fsm.vars.stop_attempts = self.fsm.vars.stop_attempts + 1
end

function FALLBACK_TO_ODOM:init()
   printf("motor_move: lost target frame %s, falling back to odom frame", self.fsm.vars.target_frame)
   
   self.fsm.vars.target = self.fsm.vars.fallback_target_odom
   self.fsm.vars.target_frame = "/odom"
   self.fsm.vars.num_fallbacks = self.fsm.vars.num_fallbacks + 1
end

function RECOVER_TO_FRAME:init()
   printf("motor_move: found original target frame %s again, using it", self.fsm.vars.frame)
   
   self.fsm.vars.target = self.fsm.vars.recover_target_frame
   self.fsm.vars.target_frame = self.fsm.vars.frame
end
