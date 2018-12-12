
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

-- id depends on Color of team. If cyan, the ids are 3 and 4.
-- For magenta team however the ids are 1 and 2.
function parse_to_type_id_target(if_conveyor_pose,place,side,shelf,slide)
  type_target = parse_to_type_target(if_conveyor_pose, place, side, shelf, slide)
  if string.sub(place,5,5) == "1" then
    id = 1
  end
  if string.sub(place,5,5) == "2" then
    id = 2
  end
  if string.sub(place,1,1) == "C" then
    id = id + 2
  end

  return { mps_type = type_target.type, mps_target = type_target.target, mps_id = id}
end
