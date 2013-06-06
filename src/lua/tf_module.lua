require("fawkes.modinit")
module(..., fawkes.modinit.module_init)

function _transform(src_pos, from_sys, to_sys)

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

	return to_sp
end

function transform(src_pos, from_sys, to_sys)
   local stamped_pose = _transform(src_pos, from_sys, to_sys)
   return { x = stamped_pose:getOrigin():x(),
            y = stamped_pose:getOrigin():y(),
            ori = 2*math.acos(stamped_pose:getRotation():w())
   }
end

function transform_mirror_rad(src_pos, from_sys, to_sys)
   local sp = _transform(src_pos, from_sys, to_sys)
   local q = sp:getRotation()
   local yaw = math.atan2( 2*( q:x()*q:w() + q:y()*q:z() ),
                           1 - 2*( q:z()^2 + q:w()^2 ) )
   return { x = sp:getOrigin():x(),
            y = sp:getOrigin():y(),
            ori = yaw
   }
end

