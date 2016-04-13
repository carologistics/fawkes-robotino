require("fawkes.modinit")
module(..., fawkes.modinit.register_all)

function iface_for_id(tag_iface_list, tag_info_iface, tag_id)
   for i,id in ipairs(tag_info_iface:tag_id()) do
      if id == tag_id then
         return tag_iface_list[i]
      end
   end
   return nil
end

function some_tag(tag_iface_list)
   for i,iface in ipairs(tag_iface_list) do
      if iface:visibility_history() > 0 then
         return iface
      end
   end
   return nil
end
