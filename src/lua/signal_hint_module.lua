
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

-- Table height above the laser scanner plane
local TABLE_HEIGHT = 0.53

-- Signal positions relative to the top-left-front edge of the MPS table
local SIGNAL_POS = {
   RS = { x=0.045, y=-0.1, z=0.185 },
   DS = { x=0.05,  y=-0.1, z=0.185 },
   BS = { x=0.04,  y=-0.1, z=0.185 }
}

function send_hint(bb_signal_hint, machine)
   local msg = bb_signal_hint.SignalPositionMessage:new()
   msg:set_translation(0, SIGNAL_POS[machine].x)
   msg:set_translation(1, SIGNAL_POS[machine].y)
   msg:set_translation(2, SIGNAL_POS[machine].z + TABLE_HEIGHT)
   bb_signal_hint:msgq_enqueue_copy(msg)
end
