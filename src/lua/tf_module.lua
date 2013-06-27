require("fawkes.modinit")
local tf = tf
module(..., fawkes.modinit.module_init)

function transform(src_pos, from_sys, to_sys)

	if not tf:can_transform(from_sys, to_sys, fawkes.Time:new(0, 0)) then
		-- no transform currently available from from_sys to to_sys
		return nil
	end

	local from_t = fawkes.tf.Vector3:new(src_pos.x, src_pos.y, 0)
	local from_r = fawkes.tf.Quaternion:new(src_pos.ori, 0, 0)
	local from_p = fawkes.tf.Pose:new(from_r, from_t)
	local from_sp = fawkes.tf.StampedPose(from_p, fawkes.Time:new(0, 0), from_sys)
	local to_sp = fawkes.tf.StampedPose:new()
	tf:transform_pose(to_sys, from_sp, to_sp)

	return { x = to_sp:getOrigin():x(),
            y = to_sp:getOrigin():y(),
            ori = fawkes.tf.get_yaw(to_sp:getRotation())
   }
end

