
/***************************************************************************
 *  RobotinoAmpelInterface.cpp - Fawkes BlackBoard Interface - RobotinoAmpelInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012  Jens Wittmeyer
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

#include <interfaces/RobotinoAmpelInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class RobotinoAmpelInterface <interfaces/RobotinoAmpelInterface.h>
 * RobotinoAmpelInterface Fawkes BlackBoard Interface.
 * 
     Storage about the Ampel.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
RobotinoAmpelInterface::RobotinoAmpelInterface() : Interface()
{
  data_size = sizeof(RobotinoAmpelInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (RobotinoAmpelInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_ENUM, "state", 1, &data->state, "AmpelState");
  unsigned char tmp_hash[] = {0xd9, 0x87, 0x6d, 0xe2, 0x91, 0x55, 0xc2, 0x11, 0x8, 0x94, 0xf5, 0xd6, 0x83, 0xdd, 0xb5, 0x3c};
  set_hash(tmp_hash);
}

/** Destructor */
RobotinoAmpelInterface::~RobotinoAmpelInterface()
{
  free(data_ptr);
}
/** Convert AmpelState constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
RobotinoAmpelInterface::tostring_AmpelState(AmpelState value) const
{
  switch (value) {
  case YELLOW: return "YELLOW";
  case GREEN: return "GREEN";
  case RED: return "RED";
  case YELLOW_FLASHING: return "YELLOW_FLASHING";
  case NO_CHANGE: return "NO_CHANGE";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get state value.
 * 

 * @return state value
 */
RobotinoAmpelInterface::AmpelState
RobotinoAmpelInterface::state() const
{
  return (RobotinoAmpelInterface::AmpelState)data->state;
}

/** Get maximum length of state value.
 * @return length of state value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoAmpelInterface::maxlenof_state() const
{
  return 1;
}

/** Set state value.
 * 

 * @param new_state new state value
 */
void
RobotinoAmpelInterface::set_state(const AmpelState new_state)
{
  data->state = new_state;
  data_changed = true;
}

/* =========== message create =========== */
Message *
RobotinoAmpelInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
RobotinoAmpelInterface::copy_values(const Interface *other)
{
  const RobotinoAmpelInterface *oi = dynamic_cast<const RobotinoAmpelInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(RobotinoAmpelInterface_data_t));
}

const char *
RobotinoAmpelInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "AmpelState") == 0) {
    return tostring_AmpelState((AmpelState)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
RobotinoAmpelInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(RobotinoAmpelInterface)
/// @endcond


} // end namespace fawkes
