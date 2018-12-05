
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
