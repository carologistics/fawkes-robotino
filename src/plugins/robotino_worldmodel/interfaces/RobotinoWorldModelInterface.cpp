
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

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class RobotinoWorldModelInterface <interfaces/RobotinoWorldModelInterface.h>
 * RobotinoWorldModelInterface Fawkes BlackBoard Interface.
 * Complete world model for the Logistics Competition
 * @ingroup FawkesInterfaces
 */



/** Constructor */
RobotinoWorldModelInterface::RobotinoWorldModelInterface() : Interface()
{
  data_size = sizeof(RobotinoWorldModelInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (RobotinoWorldModelInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_ENUM, "machine_names", 13, &data->machine_names, "machine_name_t");
  add_fieldinfo(IFT_ENUM, "machine_states", 13, &data->machine_states, "machine_state_t");
  add_fieldinfo(IFT_STRING, "express_machine", 1, data->express_machine);
  unsigned char tmp_hash[] = {0x4c, 0x38, 0x80, 0x43, 0xe1, 0x95, 0x91, 0x77, 0x10, 0x93, 0x40, 0x5d, 0x3c, 0xb1, 0xca, 0x2b};
  set_hash(tmp_hash);
}

/** Destructor */
RobotinoWorldModelInterface::~RobotinoWorldModelInterface()
{
  free(data_ptr);
}
/** Convert machine_name_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
RobotinoWorldModelInterface::tostring_machine_name_t(machine_name_t value) const
{
  switch (value) {
  case EXPRESS_MACHINE: return "EXPRESS_MACHINE";
  case M1: return "M1";
  case M2: return "M2";
  case M3: return "M3";
  case M1_2: return "M1_2";
  case M2_3: return "M2_3";
  case UNKNOWN: return "UNKNOWN";
  case RECYCLING: return "RECYCLING";
  case TEST: return "TEST";
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
  case S0_ONLY: return "S0_ONLY";
  case S1_ONLY: return "S1_ONLY";
  case S2_ONLY: return "S2_ONLY";
  case P: return "P";
  case S1_S2: return "S1_S2";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get machine_names value.
 * Names
			of the machines on the field
 * @return machine_names value
 */
RobotinoWorldModelInterface::machine_name_t *
RobotinoWorldModelInterface::machine_names() const
{
  return (RobotinoWorldModelInterface::machine_name_t *)data->machine_names;
}

/** Get machine_names value at given index.
 * Names
			of the machines on the field
 * @param index index of value
 * @return machine_names value
 * @exception Exception thrown if index is out of bounds
 */
RobotinoWorldModelInterface::machine_name_t
RobotinoWorldModelInterface::machine_names(unsigned int index) const
{
  if (index > 13) {
    throw Exception("Index value %u out of bounds (0..13)", index);
  }
  return (RobotinoWorldModelInterface::machine_name_t)data->machine_names[index];
}

/** Get maximum length of machine_names value.
 * @return length of machine_names value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoWorldModelInterface::maxlenof_machine_names() const
{
  return 13;
}

/** Set machine_names value.
 * Names
			of the machines on the field
 * @param new_machine_names new machine_names value
 */
void
RobotinoWorldModelInterface::set_machine_names(const machine_name_t * new_machine_names)
{
  memcpy(data->machine_names, new_machine_names, sizeof(machine_name_t) * 13);
  data_changed = true;
}

/** Set machine_names value at given index.
 * Names
			of the machines on the field
 * @param new_machine_names new machine_names value
 * @param index index for of the value
 */
void
RobotinoWorldModelInterface::set_machine_names(unsigned int index, const machine_name_t new_machine_names)
{
  if (index > 13) {
    throw Exception("Index value %u out of bounds (0..13)", index);
  }
  data->machine_names[index] = new_machine_names;
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
}
/** Get express_machine value.
 * Name of the machine reserved for the
			express route
 * @return express_machine value
 */
char *
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
 * Name of the machine reserved for the
			express route
 * @param new_express_machine new express_machine value
 */
void
RobotinoWorldModelInterface::set_express_machine(const char * new_express_machine)
{
  strncpy(data->express_machine, new_express_machine, sizeof(data->express_machine));
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
  if (strcmp(enumtype, "machine_name_t") == 0) {
    return tostring_machine_name_t((machine_name_t)val);
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
