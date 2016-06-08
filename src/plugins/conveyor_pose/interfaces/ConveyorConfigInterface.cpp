
/***************************************************************************
 *  ConveyorConfigInterface.cpp - Fawkes BlackBoard Interface - ConveyorConfigInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Tobias Neumann
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

#include <interfaces/ConveyorConfigInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ConveyorConfigInterface <interfaces/ConveyorConfigInterface.h>
 * ConveyorConfigInterface Fawkes BlackBoard Interface.
 * 
      The interface where external transient config values can be set for the conveyor_pose plugin.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
ConveyorConfigInterface::ConveyorConfigInterface() : Interface()
{
  data_size = sizeof(ConveyorConfigInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ConveyorConfigInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "product_removal", 1, &data->product_removal);
  add_messageinfo("enable_product_removalMessage");
  add_messageinfo("disable_product_removalMessage");
  unsigned char tmp_hash[] = {0xd5, 0xbb, 0xaf, 0x8b, 0xca, 0xed, 00, 0x4b, 0xa6, 0x96, 0xfc, 0x70, 0x92, 0xea, 0xd7, 0xaf};
  set_hash(tmp_hash);
}

/** Destructor */
ConveyorConfigInterface::~ConveyorConfigInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get product_removal value.
 * 
      True if a product detection should be run and remove detected cycles.
      This should only be activated if there is a product to be expected, otherwise the performance decreases.
    
 * @return product_removal value
 */
bool
ConveyorConfigInterface::is_product_removal() const
{
  return data->product_removal;
}

/** Get maximum length of product_removal value.
 * @return length of product_removal value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ConveyorConfigInterface::maxlenof_product_removal() const
{
  return 1;
}

/** Set product_removal value.
 * 
      True if a product detection should be run and remove detected cycles.
      This should only be activated if there is a product to be expected, otherwise the performance decreases.
    
 * @param new_product_removal new product_removal value
 */
void
ConveyorConfigInterface::set_product_removal(const bool new_product_removal)
{
  data->product_removal = new_product_removal;
  data_changed = true;
}

/* =========== message create =========== */
Message *
ConveyorConfigInterface::create_message(const char *type) const
{
  if ( strncmp("enable_product_removalMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new enable_product_removalMessage();
  } else if ( strncmp("disable_product_removalMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new disable_product_removalMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
ConveyorConfigInterface::copy_values(const Interface *other)
{
  const ConveyorConfigInterface *oi = dynamic_cast<const ConveyorConfigInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(ConveyorConfigInterface_data_t));
}

const char *
ConveyorConfigInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class ConveyorConfigInterface::enable_product_removalMessage <interfaces/ConveyorConfigInterface.h>
 * enable_product_removalMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
ConveyorConfigInterface::enable_product_removalMessage::enable_product_removalMessage() : Message("enable_product_removalMessage")
{
  data_size = sizeof(enable_product_removalMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (enable_product_removalMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
ConveyorConfigInterface::enable_product_removalMessage::~enable_product_removalMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ConveyorConfigInterface::enable_product_removalMessage::enable_product_removalMessage(const enable_product_removalMessage *m) : Message("enable_product_removalMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (enable_product_removalMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ConveyorConfigInterface::enable_product_removalMessage::clone() const
{
  return new ConveyorConfigInterface::enable_product_removalMessage(this);
}
/** @class ConveyorConfigInterface::disable_product_removalMessage <interfaces/ConveyorConfigInterface.h>
 * disable_product_removalMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
ConveyorConfigInterface::disable_product_removalMessage::disable_product_removalMessage() : Message("disable_product_removalMessage")
{
  data_size = sizeof(disable_product_removalMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (disable_product_removalMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
ConveyorConfigInterface::disable_product_removalMessage::~disable_product_removalMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ConveyorConfigInterface::disable_product_removalMessage::disable_product_removalMessage(const disable_product_removalMessage *m) : Message("disable_product_removalMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (disable_product_removalMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ConveyorConfigInterface::disable_product_removalMessage::clone() const
{
  return new ConveyorConfigInterface::disable_product_removalMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
ConveyorConfigInterface::message_valid(const Message *message) const
{
  const enable_product_removalMessage *m0 = dynamic_cast<const enable_product_removalMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const disable_product_removalMessage *m1 = dynamic_cast<const disable_product_removalMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ConveyorConfigInterface)
/// @endcond


} // end namespace fawkes
