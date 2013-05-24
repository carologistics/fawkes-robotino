require("fawkes.modinit")

module(..., fawkes.modinit.module_init)

function get_puck_loc(omnipuck)
	local rv
	if omnipuck:visibility_history() >= 5 then
		rv = {
			x = omnipuck:translation(0),
			y = omnipuck:translation(1)
		}
	end
	return rv
end

function get_puck_dist(omnipuck)
   local loc = get_puck_loc(omnipuck)
   return math.sqrt(loc.x^2 + loc.y^2)
end
