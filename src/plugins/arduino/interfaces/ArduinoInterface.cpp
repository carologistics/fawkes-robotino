
/***************************************************************************
 *  ArduinoInterface.cpp - Fawkes BlackBoard Interface - ArduinoInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Tim Niemueller, Nicolas Limpert
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

#include <interfaces/ArduinoInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ArduinoInterface <interfaces/ArduinoInterface.h>
 * ArduinoInterface Fawkes BlackBoard Interface.
 * 
      Interface to access Arduino functionalities
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
ArduinoInterface::ArduinoInterface() : Interface()
{
  data_size = sizeof(ArduinoInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ArduinoInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "z_position", 1, &data->z_position);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_messageinfo("MoveUpwardsMessage");
  add_messageinfo("MoveDownwardsMessage");
  unsigned char tmp_hash[] = {0xb8, 0xd3, 0x66, 0x9a, 0x16, 0xc3, 0x20, 0xa7, 0x41, 0xda, 0x21, 0x2f, 0x89, 0x5c, 0x2b, 0x69};
  set_hash(tmp_hash);
}

/** Destructor */
ArduinoInterface::~ArduinoInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get z_position value.
 * Current z-position
 * @return z_position value
 */
uint32_t
ArduinoInterface::z_position() const
{
  return data->z_position;
}

/** Get maximum length of z_position value.
 * @return length of z_position value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ArduinoInterface::maxlenof_z_position() const
{
  return 1;
}

/** Set z_position value.
 * Current z-position
 * @param new_z_position new z_position value
 */
void
ArduinoInterface::set_z_position(const uint32_t new_z_position)
{
  data->z_position = new_z_position;
  data_changed = true;
}

/** Get final value.
 * True, if the last move command has been finished
 * @return final value
 */
bool
ArduinoInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ArduinoInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last move command has been finished
 * @param new_final new final value
 */
void
ArduinoInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/* =========== message create =========== */
Message *
ArduinoInterface::create_message(const char *type) const
{
  if ( strncmp("MoveUpwardsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveUpwardsMessage();
  } else if ( strncmp("MoveDownwardsMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new MoveDownwardsMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
ArduinoInterface::copy_values(const Interface *other)
{
  const ArduinoInterface *oi = dynamic_cast<const ArduinoInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(ArduinoInterface_data_t));
}

const char *
ArduinoInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class ArduinoInterface::MoveUpwardsMessage <interfaces/ArduinoInterface.h>
 * MoveUpwardsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_num_mm initial value for num_mm
 */
ArduinoInterface::MoveUpwardsMessage::MoveUpwardsMessage(const uint32_t ini_num_mm) : Message("MoveUpwardsMessage")
{
  data_size = sizeof(MoveUpwardsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveUpwardsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->num_mm = ini_num_mm;
  add_fieldinfo(IFT_UINT32, "num_mm", 1, &data->num_mm);
}
/** Constructor */
ArduinoInterface::MoveUpwardsMessage::MoveUpwardsMessage() : Message("MoveUpwardsMessage")
{
  data_size = sizeof(MoveUpwardsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveUpwardsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "num_mm", 1, &data->num_mm);
}

/** Destructor */
ArduinoInterface::MoveUpwardsMessage::~MoveUpwardsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ArduinoInterface::MoveUpwardsMessage::MoveUpwardsMessage(const MoveUpwardsMessage *m) : Message("MoveUpwardsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MoveUpwardsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get num_mm value.
 * Number of mm to drive up
 * @return num_mm value
 */
uint32_t
ArduinoInterface::MoveUpwardsMessage::num_mm() const
{
  return data->num_mm;
}

/** Get maximum length of num_mm value.
 * @return length of num_mm value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ArduinoInterface::MoveUpwardsMessage::maxlenof_num_mm() const
{
  return 1;
}

/** Set num_mm value.
 * Number of mm to drive up
 * @param new_num_mm new num_mm value
 */
void
ArduinoInterface::MoveUpwardsMessage::set_num_mm(const uint32_t new_num_mm)
{
  data->num_mm = new_num_mm;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ArduinoInterface::MoveUpwardsMessage::clone() const
{
  return new ArduinoInterface::MoveUpwardsMessage(this);
}
/** @class ArduinoInterface::MoveDownwardsMessage <interfaces/ArduinoInterface.h>
 * MoveDownwardsMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_num_mm initial value for num_mm
 */
ArduinoInterface::MoveDownwardsMessage::MoveDownwardsMessage(const uint32_t ini_num_mm) : Message("MoveDownwardsMessage")
{
  data_size = sizeof(MoveDownwardsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveDownwardsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->num_mm = ini_num_mm;
  add_fieldinfo(IFT_UINT32, "num_mm", 1, &data->num_mm);
}
/** Constructor */
ArduinoInterface::MoveDownwardsMessage::MoveDownwardsMessage() : Message("MoveDownwardsMessage")
{
  data_size = sizeof(MoveDownwardsMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (MoveDownwardsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "num_mm", 1, &data->num_mm);
}

/** Destructor */
ArduinoInterface::MoveDownwardsMessage::~MoveDownwardsMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ArduinoInterface::MoveDownwardsMessage::MoveDownwardsMessage(const MoveDownwardsMessage *m) : Message("MoveDownwardsMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (MoveDownwardsMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get num_mm value.
 * Number of mm to drive down
 * @return num_mm value
 */
uint32_t
ArduinoInterface::MoveDownwardsMessage::num_mm() const
{
  return data->num_mm;
}

/** Get maximum length of num_mm value.
 * @return length of num_mm value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ArduinoInterface::MoveDownwardsMessage::maxlenof_num_mm() const
{
  return 1;
}

/** Set num_mm value.
 * Number of mm to drive down
 * @param new_num_mm new num_mm value
 */
void
ArduinoInterface::MoveDownwardsMessage::set_num_mm(const uint32_t new_num_mm)
{
  data->num_mm = new_num_mm;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ArduinoInterface::MoveDownwardsMessage::clone() const
{
  return new ArduinoInterface::MoveDownwardsMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
ArduinoInterface::message_valid(const Message *message) const
{
  const MoveUpwardsMessage *m0 = dynamic_cast<const MoveUpwardsMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const MoveDownwardsMessage *m1 = dynamic_cast<const MoveDownwardsMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ArduinoInterface)
/// @endcond


} // end namespace fawkes
