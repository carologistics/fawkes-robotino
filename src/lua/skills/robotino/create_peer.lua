
----------------------------------------------------------------------------
--  create_peer.lua
--
--  (c) 2019 Victor Mataré
--
----------------------------------------------------------------------------

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

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "create_peer"
fsm                = SkillHSM:new{name=name, start="SEND_MSG"}
depends_skills     = { }
depends_interfaces = {
   {v = "peer_iface", type="ProtobufPeerInterface", id="/protoboard/peers"},
}

documentation      = [==[Create and connect to a ProtoBuf peer

@param address      The peer's IP address (string.
@param port         The port to send to & receive on.
@param send_to_port The port to send to. Mutually exclusive with the @a port param.
@param recv_on_port The port to receive on. Mutually exclusive with the @a port param.
@param cipher       (Optional) Cipher name
@param crypto_key   (Optional) The encrypthon key

]==]

-- Initialize as skill module
skillenv.skill_module(_M)


function input_ok()
   local port_ok = fsm.vars.port or (fsm.vars.send_to_port and fsm.vars.recv_on_port) and not(
      fsm.vars.port and (fsm.vars.send_to_port or fsm.vars.recv_on_port)
   )
   local crypto_ok = (not not fsm.vars.cipher == not not fsm.vars.crypto_key)
   return fsm.vars.address and port_ok and crypto_ok
end

fsm:define_states{ export_to=_M,
   closure={ input_ok=input_ok },
   {"SEND_MSG", JumpState}
}

fsm:add_transitions{
   {"SEND_MSG", "FAILED", precond="not input_ok()", desc="wrong input params"},
   {"SEND_MSG", "FINAL", cond=true}
}

function SEND_MSG:init()
   local msg
   if fsm.vars.send_to_port then
      if fsm.vars.cipher then
         msg = peer_iface.CreatePeerLocalCryptoMessage:new(
            fsm.vars.address,
            fsm.vars.send_to_port,
            fsm.vars.recv_on_port,
            fsm.vars.crypto_key,
            fsm.vars.cipher
         )
      else
         msg = peer_iface.CreatePeerLocalMessage:new(
            fsm.vars.address,
            fsm.vars.send_to_port,
            fsm.vars.recv_on_port
         )
      end
   else
      if fsm.vars.cipher then
         msg = peer_iface.CreatePeerCryptoMessage:new(
            fsm.vars.address,
            fsm.vars.port,
            fsm.vars.crypto_key,
            fsm.vars.cipher
         )
      else
         msg = peer_iface.CreatePeerMessage:new(
            fsm.vars.address,
            fsm.vars.port
         )
      end
   end
   peer_iface:msgq_enqueue_copy(msg)
end

