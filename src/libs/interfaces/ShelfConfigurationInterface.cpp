
/***************************************************************************
 *  ShelfConfigurationInterface.cpp - Fawkes BlackBoard Interface - ShelfConfigurationInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2017  Till Hofmann
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

#include <interfaces/ShelfConfigurationInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ShelfConfigurationInterface <interfaces/ShelfConfigurationInterface.h>
 * ShelfConfigurationInterface Fawkes BlackBoard Interface.
 * 
     A configuration of a shelf, determines the color of each of its three
     slots.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
ShelfConfigurationInterface::ShelfConfigurationInterface() : Interface()
{
  data_size = sizeof(ShelfConfigurationInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ShelfConfigurationInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_SlotColor[(int)RED] = "RED";
  enum_map_SlotColor[(int)BLACK] = "BLACK";
  enum_map_SlotColor[(int)EMPTY] = "EMPTY";
  add_fieldinfo(IFT_ENUM, "left", 1, &data->left, "SlotColor", &enum_map_SlotColor);
  add_fieldinfo(IFT_ENUM, "center", 1, &data->center, "SlotColor", &enum_map_SlotColor);
  add_fieldinfo(IFT_ENUM, "right", 1, &data->right, "SlotColor", &enum_map_SlotColor);
  add_fieldinfo(IFT_BOOL, "ready", 1, &data->ready);
  unsigned char tmp_hash[] = {0x3d, 0x19, 0x1c, 0x59, 0xa2, 0xc2, 0x6b, 0xda, 0x40, 0xf, 0x82, 0x71, 0x17, 0x93, 0xef, 0x7d};
  set_hash(tmp_hash);
}

/** Destructor */
ShelfConfigurationInterface::~ShelfConfigurationInterface()
{
  free(data_ptr);
}
/** Convert SlotColor constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
ShelfConfigurationInterface::tostring_SlotColor(SlotColor value) const
{
  switch (value) {
  case RED: return "RED";
  case BLACK: return "BLACK";
  case EMPTY: return "EMPTY";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get left value.
 * 
    The color of the left slot.
  
 * @return left value
 */
ShelfConfigurationInterface::SlotColor
ShelfConfigurationInterface::left() const
{
  return (ShelfConfigurationInterface::SlotColor)data->left;
}

/** Get maximum length of left value.
 * @return length of left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ShelfConfigurationInterface::maxlenof_left() const
{
  return 1;
}

/** Set left value.
 * 
    The color of the left slot.
  
 * @param new_left new left value
 */
void
ShelfConfigurationInterface::set_left(const SlotColor new_left)
{
  data->left = new_left;
  data_changed = true;
}

/** Get center value.
 * 
    The color of the center slot.
  
 * @return center value
 */
ShelfConfigurationInterface::SlotColor
ShelfConfigurationInterface::center() const
{
  return (ShelfConfigurationInterface::SlotColor)data->center;
}

/** Get maximum length of center value.
 * @return length of center value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ShelfConfigurationInterface::maxlenof_center() const
{
  return 1;
}

/** Set center value.
 * 
    The color of the center slot.
  
 * @param new_center new center value
 */
void
ShelfConfigurationInterface::set_center(const SlotColor new_center)
{
  data->center = new_center;
  data_changed = true;
}

/** Get right value.
 * 
    The color of the right slot.
  
 * @return right value
 */
ShelfConfigurationInterface::SlotColor
ShelfConfigurationInterface::right() const
{
  return (ShelfConfigurationInterface::SlotColor)data->right;
}

/** Get maximum length of right value.
 * @return length of right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ShelfConfigurationInterface::maxlenof_right() const
{
  return 1;
}

/** Set right value.
 * 
    The color of the right slot.
  
 * @param new_right new right value
 */
void
ShelfConfigurationInterface::set_right(const SlotColor new_right)
{
  data->right = new_right;
  data_changed = true;
}

/** Get ready value.
 * 
    Whether the shelf configuration is done.
  
 * @return ready value
 */
bool
ShelfConfigurationInterface::is_ready() const
{
  return data->ready;
}

/** Get maximum length of ready value.
 * @return length of ready value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ShelfConfigurationInterface::maxlenof_ready() const
{
  return 1;
}

/** Set ready value.
 * 
    Whether the shelf configuration is done.
  
 * @param new_ready new ready value
 */
void
ShelfConfigurationInterface::set_ready(const bool new_ready)
{
  data->ready = new_ready;
  data_changed = true;
}

/* =========== message create =========== */
Message *
ShelfConfigurationInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
ShelfConfigurationInterface::copy_values(const Interface *other)
{
  const ShelfConfigurationInterface *oi = dynamic_cast<const ShelfConfigurationInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(ShelfConfigurationInterface_data_t));
}

const char *
ShelfConfigurationInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "SlotColor") == 0) {
    return tostring_SlotColor((SlotColor)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
ShelfConfigurationInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ShelfConfigurationInterface)
/// @endcond


} // end namespace fawkes
