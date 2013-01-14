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
	local to_t = to_sp:getOrigin()
	local to_r = to_sp:getRotation()

	local rv = {
		x = to_t:x(),
		y = to_t:y(),
		ori = 2*math.acos(to_r:w())
	}

--	printf("Transform %s (%f,%f,%f) to %s (%f,%f,%f).",
--	 from_sys, from_t:x(), from_t:y(), 2*math.acos(from_r:w()),
--	 to_sys, rv.x, rv.y, rv.ori)

	return rv
end


