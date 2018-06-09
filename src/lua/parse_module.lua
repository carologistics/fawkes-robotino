require("fawkes.modinit")
module(..., fawkes.modinit.module_init)

local type
local target

function parse_to_type_target(if_conveyor_pose,place,side,shelf,slide)
  if string.sub(place,3,3) == "B" then
    type = if_conveyor_pose.BASE_STATION
  end
  if string.sub(place,3,3) == "C" then
    type = if_conveyor_pose.CAP_STATION
  end
  if string.sub(place,3,3) == "R" then
    type = if_conveyor_pose.RING_STATION
  end
  if string.sub(place,3,3) == "S" then
    type = if_conveyor_pose.STORAGE_STATION
  end
  if string.sub(place,3,3) == "D" then
    type = if_conveyor_pose.DELIVERY_STATION
  end

  if side == "input" then
    target = if_conveyor_pose.INPUT_CONVEYOR
  end
  if side == "output" then
    target = if_conveyor_pose.OUTPUT_CONVEYOR
  end

  if shelf == "MIDDLE" then
    target = if_conveyor_pose.SHELF_MIDDLE
  end

  if shelf == "RIGHT" then
    target = if_conveyor_pose.SHELF_RIGHT
  end

  if shelf == "LEFT" then
    target = if_conveyor_pose.SHELF_LEFT
  end

  if slide then
    target = if_conveyor_pose.SLIDE
  end

  return { mps_type = type, mps_target = target}
end
