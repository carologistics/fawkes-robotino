local module {

	function get_puck_loc(omnipuck)
		local rv
		if omnipuck:visibility_history() >= 5 then
			rv = {
				x = omnipuck.translation(0),
				y = omnipuck.translation(1)
			}
		end
		return rv
	end

}

return module
