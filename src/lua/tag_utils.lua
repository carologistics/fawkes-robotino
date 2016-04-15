require("fawkes.modinit")
module(..., fawkes.modinit.register_all)

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
   for i = 0,15 do
      if tag_info_iface:tag_id(i) ~= 0 and tag_info_iface:tag_id(i) ~= tag_id then
         table.insert(rv, tag_iface_list[i+1])
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
