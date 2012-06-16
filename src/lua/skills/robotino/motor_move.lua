
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
skillenv.skill_module(...)


function calc_status()
	dist_x = fsm.vars.motor_target_x - motor:odometry_position_x()
	dist_y = fsm.vars.motor_target_y - motor:odometry_position_y()
	dist_ori = fsm.vars.motor_target_ori - motor:odometry_orientation()
	
	dir_x = 0
	dir_y = 0
	dir_ori = 0

	if dist_x > 0 then
		dir_x = 1
	elseif dist_x < 0 then
		dir_x = -1
	end
	if dist_y > 0 then
		dir_y = 1
	elseif dist_y < 0 then
		dir_y = -1
	end
	if dist_ori > 0 then
		dir_ori = 1
	elseif dist_ori < 0 then
		dir_ori = -1
	end
end


function target_reached()
	calc_status()

	if fsm.vars.x and math.abs(dist_x) < 0.07 then
		fsm.vars.motor_vx = dir_x * 0.04
		if math.abs(dist_x) < 0.03 then
			fsm.vars.motor_vx = 0
		end
	end
	if fsm.vars.y and math.abs(dist_y) < 0.07 then
		fsm.vars.motor_vy = dir_y * 0.04
		if math.abs(dist_y) < 0.03 then
			fsm.vars.motor_vy = 0
		end
	end
	if fsm.vars.ori and math.abs(dist_ori) < 0.2 then
		fsm.vars.motor_omega = dir_ori * 0.1
		if math.abs(dist_ori) < 0.03 then
			fsm.vars.motor_omega = 0
		end
	end

	printf("x: " .. dist_x .. "\t" .. fsm.vars.motor_vx)
	printf("y: " .. dist_y .. "\t" .. fsm.vars.motor_vy)
	printf("ori: " .. dist_ori .. "\t" .. fsm.vars.motor_omega)
	send_transrot(fsm.vars.motor_vx, fsm.vars.motor_vy, fsm.vars.motor_omega)

	if fsm.vars.motor_vx == 0 and fsm.vars.motor_vy == 0 and fsm.vars.motor_omega == 0 then
		return true
	end
	return false
end

function invalid_input()
	if fsm.vars.ori and math.abs(fsm.vars.ori) > math.pi then
		return true
	end
	return false
end

fsm:add_transitions{
	closure={motor=motor},
	{"DRIVE", "FAILED", cond=invalid_input, precond=true, desc="ori must be < pi"},
	{"DRIVE", "STOP", cond=target_reached},
	{"STOP", "FINAL", cond=true}
}

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
	local ori = self.fsm.vars.ori or math.atan2(y, x)


	local baselink_t = fawkes.tf.Vector3:new(x, y, 0)
        local baselink_r = fawkes.tf.Quaternion:new(ori, 0, 0)
        local baselink_p = fawkes.tf.Pose:new(baselink_r, baselink_t)
        local odom_sp = fawkes.tf.StampedPose:new()
        local baselink_sp = fawkes.tf.StampedPose(baselink_p, fawkes.Time:new(0, 0), "/base_link")
	tf:transform_pose("/robotino_odometry", baselink_sp, odom_sp)
	local odom_t = odom_sp:getOrigin()
        local odom_r = odom_sp:getRotation()
	printf("Baselink: (%f,%f,%f) (%f,%f,%f,%f)  Odom: (%f,%f,%f)  (%f,%f,%f,%f)",
               baselink_t:x(), baselink_t:y(), baselink_t:z(),
               baselink_r:x(), baselink_r:y(), baselink_r:z(), baselink_r:w(),
               odom_t:x(), odom_t:y(), odom_t:z(),
               odom_r:x(), odom_r:y(), odom_r:z(), odom_r:w())


	self.fsm.vars.motor_target_x = odom_t:x()
	self.fsm.vars.motor_target_y = odom_t:y()
	local odo_ori = motor:odometry_orientation()
	if odo_ori < 0 then
		if odo_ori + ori < -math.pi then
			self.fsm.vars.motor_target_ori = odo_ori - ori
		else
			self.fsm.vars.motor_target_ori = odo_ori + ori
		end
	else
		if odo_ori + ori > math.pi then
			self.fsm.vars.motor_target_ori = odo_ori - ori
		else
			self.fsm.vars.motor_target_ori = odo_ori + ori
		end
	end
	printf("t_ori=" .. self.fsm.vars.motor_target_ori)
	self.fsm.vars.motor_vx = 0
	self.fsm.vars.motor_vy = 0
	self.fsm.vars.motor_omega = 0

	calc_status()

	if x ~= 0 then
		self.fsm.vars.motor_vx = dir_x * 0.1
	end
	if y ~= 0 then
		self.fsm.vars.motor_vy = dir_y * 0.1
	end
	if ori ~= 0 then
		self.fsm.vars.motor_omega = dir_ori * 0.3
	end

	send_transrot(self.fsm.vars.motor_vx, self.fsm.vars.motor_vy, self.fsm.vars.motor_omega)
end

function STOP:init()
	send_transrot(0,0,0)
end
   
