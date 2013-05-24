
/***************************************************************************
 *  RobotinoLightInterface.cpp - Fawkes BlackBoard Interface - RobotinoLightInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Florian Nolden, Viktor Mataré, Tobias Neumann
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

#include <interfaces/RobotinoLightInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class RobotinoLightInterface <interfaces/RobotinoLightInterface.h>
 * RobotinoLightInterface Fawkes BlackBoard Interface.
 * 
     Storage about the Ampel.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
RobotinoLightInterface::RobotinoLightInterface() : Interface()
{
  data_size = sizeof(RobotinoLightInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (RobotinoLightInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_ENUM, "red", 1, &data->red, "LightState");
  add_fieldinfo(IFT_ENUM, "yellow", 1, &data->yellow, "LightState");
  add_fieldinfo(IFT_ENUM, "green", 1, &data->green, "LightState");
  add_fieldinfo(IFT_INT32, "visibility_history", 1, &data->visibility_history);
  add_fieldinfo(IFT_BOOL, "ready", 1, &data->ready);
  unsigned char tmp_hash[] = {0x83, 0xd8, 0xe6, 0xf5, 0x26, 0xc, 0xaa, 0xad, 0x7a, 0x8c, 0x44, 0xd5, 0xec, 0x92, 0x93, 0x53};
  set_hash(tmp_hash);
}

/** Destructor */
RobotinoLightInterface::~RobotinoLightInterface()
{
  free(data_ptr);
}
/** Convert LightState constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
RobotinoLightInterface::tostring_LightState(LightState value) const
{
  switch (value) {
  case ON: return "ON";
  case OFF: return "OFF";
  case BLINKING: return "BLINKING";
  case UNKNOWN: return "UNKNOWN";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get red value.
 * State of red light
 * @return red value
 */
RobotinoLightInterface::LightState
RobotinoLightInterface::red() const
{
  return (RobotinoLightInterface::LightState)data->red;
}

/** Get maximum length of red value.
 * @return length of red value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoLightInterface::maxlenof_red() const
{
  return 1;
}

/** Set red value.
 * State of red light
 * @param new_red new red value
 */
void
RobotinoLightInterface::set_red(const LightState new_red)
{
  data->red = new_red;
  data_changed = true;
}

/** Get yellow value.
 * State of yellow light
 * @return yellow value
 */
RobotinoLightInterface::LightState
RobotinoLightInterface::yellow() const
{
  return (RobotinoLightInterface::LightState)data->yellow;
}

/** Get maximum length of yellow value.
 * @return length of yellow value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoLightInterface::maxlenof_yellow() const
{
  return 1;
}

/** Set yellow value.
 * State of yellow light
 * @param new_yellow new yellow value
 */
void
RobotinoLightInterface::set_yellow(const LightState new_yellow)
{
  data->yellow = new_yellow;
  data_changed = true;
}

/** Get green value.
 * State of green light
 * @return green value
 */
RobotinoLightInterface::LightState
RobotinoLightInterface::green() const
{
  return (RobotinoLightInterface::LightState)data->green;
}

/** Get maximum length of green value.
 * @return length of green value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoLightInterface::maxlenof_green() const
{
  return 1;
}

/** Set green value.
 * State of green light
 * @param new_green new green value
 */
void
RobotinoLightInterface::set_green(const LightState new_green)
{
  data->green = new_green;
  data_changed = true;
}

/** Get visibility_history value.
 * visibility history
 * @return visibility_history value
 */
int32_t
RobotinoLightInterface::visibility_history() const
{
  return data->visibility_history;
}

/** Get maximum length of visibility_history value.
 * @return length of visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoLightInterface::maxlenof_visibility_history() const
{
  return 1;
}

/** Set visibility_history value.
 * visibility history
 * @param new_visibility_history new visibility_history value
 */
void
RobotinoLightInterface::set_visibility_history(const int32_t new_visibility_history)
{
  data->visibility_history = new_visibility_history;
  data_changed = true;
}

/** Get ready value.
 * Data valid and ready
 * @return ready value
 */
bool
RobotinoLightInterface::is_ready() const
{
  return data->ready;
}

/** Get maximum length of ready value.
 * @return length of ready value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
RobotinoLightInterface::maxlenof_ready() const
{
  return 1;
}

/** Set ready value.
 * Data valid and ready
 * @param new_ready new ready value
 */
void
RobotinoLightInterface::set_ready(const bool new_ready)
{
  data->ready = new_ready;
  data_changed = true;
}

/* =========== message create =========== */
Message *
RobotinoLightInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
RobotinoLightInterface::copy_values(const Interface *other)
{
  const RobotinoLightInterface *oi = dynamic_cast<const RobotinoLightInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(RobotinoLightInterface_data_t));
}

const char *
RobotinoLightInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "LightState") == 0) {
    return tostring_LightState((LightState)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
RobotinoLightInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(RobotinoLightInterface)
/// @endcond


} // end namespace fawkes
