
/***************************************************************************
 *  RobotinoWorldModelInterface.cpp - Fawkes BlackBoard Interface - RobotinoWorldModelInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012  Daniel Ewert
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <interfaces/RobotinoWorldModelInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class RobotinoWorldModelInterface <interfaces/RobotinoWorldModelInterface.h>
 * RobotinoWorldModelInterface Fawkes BlackBoard Interface.
 * Complete world model for the Logistics Competition.
    There are
    13 Machines, the data for each machine can be accessed via the
    related fields of the arrays
    
 * @ingroup FawkesInterfaces
 */


/** Test constant */
const uint32_t RobotinoWorldModelInterface::Test = 0u;
/** m1 constant */
const uint32_t RobotinoWorldModelInterface::m1 = 1u;
/** m2 constant */
const uint32_t RobotinoWorldModelInterface::m2 = 2u;
/** m3 constant */
const uint32_t RobotinoWorldModelInterface::m3 = 3u;
/** m4 constant */
const uint32_t RobotinoWorldModelInterface::m4 = 4u;
/** m5 constant */
const uint32_t RobotinoWorldModelInterface::m5 = 5u;
/** m6 constant */
const uint32_t RobotinoWorldModelInterface::m6 = 6u;
/** m7 constant */
const uint32_t RobotinoWorldModelInterface::m7 = 7u;
/** m8 constant */
const uint32_t RobotinoWorldModelInterface::m8 = 8u;
/** m9 constant */
const uint32_t RobotinoWorldModelInterface::m9 = 9u;
/** m10 constant */
const uint32_t RobotinoWorldModelInterface::m10 = 10u;
/** r1 constant */
const uint32_t RobotinoWorldModelInterface::r1 = 11u;
/** r2 constant */
const uint32_t RobotinoWorldModelInterface::r2 = 12u;

/** Constructor */
RobotinoWorldModelInterface::RobotinoWorldModelInterface() : Interface()
{
  data_size = sizeof(RobotinoWorldModelInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (RobotinoWorldModelInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_machine_type_t[(int)TYPE_UNKNOWN] = "TYPE_UNKNOWN";
  enum_map_machine_type_t[(int)M1_EXPRESS] = "M1_EXPRESS";
  enum_map_machine_type_t[(int)M1] = "M1";
  enum_map_machine_type_t[(int)M2] = "M2";
  enum_map_machine_type_t[(int)M3] = "M3";
  enum_map_machine_type_t[(int)M1_2] = "M1_2";
  enum_map_machine_type_t[(int)M2_3] = "M2_3";
  enum_map_machine_type_t[(int)RECYCLING] = "RECYCLING";
  enum_map_machine_type_t[(int)TEST] = "TEST";
  enum_map_machine_type_t[(int)IGNORED] = "IGNORED";
  enum_map_machine_state_t[(int)STATE_UNKNOWN] = "STATE_UNKNOWN";
  enum_map_machine_state_t[(int)EMPTY] = "EMPTY";
  enum_map_machine_state_t[(int)CONSUMED_1] = "CONSUMED_1";
  enum_map_machine_state_t[(int)CONSUMED_2] = "CONSUMED_2";
  enum_map_machine_state_t[(int)S0_ONLY] = "S0_ONLY";
  enum_map_machine_state_t[(int)S1_ONLY] = "S1_ONLY";
  enum_map_machine_state_t[(int)S2_ONLY] = "S2_ONLY";
  enum_map_machine_state_t[(int)P] = "P";
  enum_map_machine_state_t[(int)S1_S2] = "S1_S2";
  enum_map_machine_state_t[(int)S0_S1] = "S0_S1";
  enum_map_machine_state_t[(int)S0_S2] = "S0_S2";
  add_fieldinfo(IFT_ENUM, "machine_types", 13, &data->machine_types, "machine_type_t", &enum_map_machine_type_t);
  add_fieldinfo(IFT_ENUM, "machine_states", 13, &data->machine_states, "machine_state_t", &enum_map_machine_state_t);
  add_fieldinfo(IFT_UINT32, "express_machine", 1, &data->express_machine);
  unsigned char tmp_hash[] = {0x6a, 0xfe, 0xe1, 0xa, 0x28, 0xe, 0x19, 0xcd, 0x9f, 0x6, 0x5b, 0xc, 0x7f, 0x42, 0x47, 0x6f};
  set_hash(tmp_hash);
}

/** Destructor */
RobotinoWorldModelInterface::~RobotinoWorldModelInterface()
{
  free(data_ptr);
}
/** Convert machine_type_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
RobotinoWorldModelInterface::tostring_machine_type_t(machine_type_t value) const
{
  switch (value) {
  case TYPE_UNKNOWN: return "TYPE_UNKNOWN";
  case M1_EXPRESS: return "M1_EXPRESS";
  case M1: return "M1";
  case M2: return "M2";
  case M3: return "M3";
  case M1_2: return "M1_2";
  case M2_3: return "M2_3";
  case RECYCLING: return "RECYCLING";
  case TEST: return "TEST";
  case IGNORED: return "IGNORED";
  default: return "UNKNOWN";
  }
}
/** Convert machine_state_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
RobotinoWorldModelInterface::tostring_machine_state_t(machine_state_t value) const
{
  switch (value) {
  case STATE_UNKNOWN: return "STATE_UNKNOWN";
  case EMPTY: return "EMPTY";
  case CONSUMED_1: return "CONSUMED_1";
  case CONSUMED_2: return "CONSUMED_2";
  case S0_ONLY: return "S0_ONLY";
  case S1_ONLY: return "S1_ONLY";
  case S2_ONLY: return "S2_ONLY";
  case P: return "P";
  case S1_S2: return "S1_S2";
  case S0_S1: return "S0_S1";
  case S0_S2: return "S0_S2";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get machine_types value.
 * Types
    of the machines on the field
 * @return machine_types value
 */
RobotinoWorldModelInterface::machine_type_t *
RobotinoWorldModelInterface::machine_types() const
{
  return (RobotinoWorldModelInterface::machine_type_t *)data->machine_types;
}

/** Get machine_types value at given index.
 * Types
    of the machines on the field
 * @param index index of value
 * @return machine_types value
 * @exception Exception thrown if index is out of bounds
 */
RobotinoWorldModelInterface::machine_type_t
RobotinoWorldModelInterface::machine_types(unsigned int index) const
{
  if (index > 13) {
    throw Exception("Index value %u out of bounds (0..13)", index);
  }
  return (RobotinoWorldModelInterface::machine_type_t)data->machine_types[index];
}

/** Get maximum length of machine_types value.
 * @return length of machine_types value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoWorldModelInterface::maxlenof_machine_types() const
{
  return 13;
}

/** Set machine_types value.
 * Types
    of the machines on the field
 * @param new_machine_types new machine_types value
 */
void
RobotinoWorldModelInterface::set_machine_types(const machine_type_t * new_machine_types)
{
  memcpy(data->machine_types, new_machine_types, sizeof(machine_type_t) * 13);
  data_changed = true;
}

/** Set machine_types value at given index.
 * Types
    of the machines on the field
 * @param new_machine_types new machine_types value
 * @param index index for of the value
 */
void
RobotinoWorldModelInterface::set_machine_types(unsigned int index, const machine_type_t new_machine_types)
{
  if (index > 13) {
    throw Exception("Index value %u out of bounds (0..13)", index);
  }
  data->machine_types[index] = new_machine_types;
  data_changed = true;
}
/** Get machine_states value.
 * States
    of all machines on the field
 * @return machine_states value
 */
RobotinoWorldModelInterface::machine_state_t *
RobotinoWorldModelInterface::machine_states() const
{
  return (RobotinoWorldModelInterface::machine_state_t *)data->machine_states;
}

/** Get machine_states value at given index.
 * States
    of all machines on the field
 * @param index index of value
 * @return machine_states value
 * @exception Exception thrown if index is out of bounds
 */
RobotinoWorldModelInterface::machine_state_t
RobotinoWorldModelInterface::machine_states(unsigned int index) const
{
  if (index > 13) {
    throw Exception("Index value %u out of bounds (0..13)", index);
  }
  return (RobotinoWorldModelInterface::machine_state_t)data->machine_states[index];
}

/** Get maximum length of machine_states value.
 * @return length of machine_states value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoWorldModelInterface::maxlenof_machine_states() const
{
  return 13;
}

/** Set machine_states value.
 * States
    of all machines on the field
 * @param new_machine_states new machine_states value
 */
void
RobotinoWorldModelInterface::set_machine_states(const machine_state_t * new_machine_states)
{
  memcpy(data->machine_states, new_machine_states, sizeof(machine_state_t) * 13);
  data_changed = true;
}

/** Set machine_states value at given index.
 * States
    of all machines on the field
 * @param new_machine_states new machine_states value
 * @param index index for of the value
 */
void
RobotinoWorldModelInterface::set_machine_states(unsigned int index, const machine_state_t new_machine_states)
{
  if (index > 13) {
    throw Exception("Index value %u out of bounds (0..13)", index);
  }
  data->machine_states[index] = new_machine_states;
  data_changed = true;
}
/** Get express_machine value.
 * The machine reserved for the
    express
    route
 * @return express_machine value
 */
uint32_t
RobotinoWorldModelInterface::express_machine() const
{
  return data->express_machine;
}

/** Get maximum length of express_machine value.
 * @return length of express_machine value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoWorldModelInterface::maxlenof_express_machine() const
{
  return 1;
}

/** Set express_machine value.
 * The machine reserved for the
    express
    route
 * @param new_express_machine new express_machine value
 */
void
RobotinoWorldModelInterface::set_express_machine(const uint32_t new_express_machine)
{
  data->express_machine = new_express_machine;
  data_changed = true;
}

/* =========== message create =========== */
Message *
RobotinoWorldModelInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
RobotinoWorldModelInterface::copy_values(const Interface *other)
{
  const RobotinoWorldModelInterface *oi = dynamic_cast<const RobotinoWorldModelInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(RobotinoWorldModelInterface_data_t));
}

const char *
RobotinoWorldModelInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "machine_type_t") == 0) {
    return tostring_machine_type_t((machine_type_t)val);
  }
  if (strcmp(enumtype, "machine_state_t") == 0) {
    return tostring_machine_state_t((machine_state_t)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
RobotinoWorldModelInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(RobotinoWorldModelInterface)
/// @endcond


} // end namespace fawkes
