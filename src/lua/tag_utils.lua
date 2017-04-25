require("fawkes.modinit")
module(..., fawkes.modinit.register_all)

local tfm = require("fawkes.tfutils")
local llutils = require("fawkes.laser-lines_utils")

function iface_for_id(tag_iface_list, tag_info_iface, tag_id)
   for i = 0,15 do
      if tag_id == tag_info_iface:tag_id(i) then
         return tag_iface_list[i+1]
      end
   end
   return nil
end

function bad_tags(tag_iface_list, tag_info_iface, tag_id)
   rv = {}
   if tag_id then
      for i = 0,15 do
         if tag_info_iface:tag_id(i) ~= 0 and tag_info_iface:tag_id(i) ~= tag_id then
            table.insert(rv, tag_iface_list[i+1])
         end
      end
   end
   return rv
end

function matching_tags(tag_iface_list, tag_info_iface, tag_id_set)
   rv = {}
   for i = 0,15 do
      if not tag_id_set or tag_id_set[tag_info_iface:tag_id(i)] then
         rv[tag_info_iface:tag_id(i)] = tag_iface_list[i+1]
      end
   end
   return rv
end

function some_tag(tag_iface_list)
   for i,iface in ipairs(tag_iface_list) do
      if iface:visibility_history() > 0 then
         return iface
      end
   end
   return nil
end

function frame_for_id(tag_iface_list, tag_info_iface, tag_id)
   local tag = iface_for_id(tag_iface_list, tag_info_iface, tag_id)
   if tag then
      local tag_idx = string.sub(tag:id(), 13) 
      return "/tag_" .. tag_idx   
   end
   return nil
end

-- Match tag to laser line
function match_line(tag, lines)
   local matched_line = nil
   if tag and tag:visibility_history() >= MIN_VIS_HIST_TAG then
      local tag_laser = tfm.transform6D(
         { x=tag:translation(0), y=tag:translation(1), z=tag:translation(2),
            ori = { x=tag:rotation(0), y=tag:rotation(1), z=tag:rotation(2), w=tag:rotation(3)  }
         }, tag:frame(), "/base_laser"
      )
      local min_dist = 1000
      for k,line in pairs(lines) do
         local line_center = llutils.center(line, 0)
         local dist = math.vec_length(tag_laser.x - line_center.x, tag_laser.y - line_center.y)
         if line:visibility_history() >= MIN_VIS_HIST_LINE
            and dist < LINE_MATCH_TOLERANCE
            and dist < min_dist
         then
            min_dist = dist
            matched_line = line
         end
      end
   end

   return matched_line
end

