
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
  add_messageinfo("EnableProductRemovalMessage");
  add_messageinfo("DisableProductRemovalMessage");
  unsigned char tmp_hash[] = {0x5c, 0x6c, 0x75, 0xaf, 0xbf, 0x1d, 0xc4, 0x13, 0x4d, 0x59, 0x6a, 0x66, 0x3e, 0x64, 0x26, 0xce};
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
  if ( strncmp("EnableProductRemovalMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new EnableProductRemovalMessage();
  } else if ( strncmp("DisableProductRemovalMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new DisableProductRemovalMessage();
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
/** @class ConveyorConfigInterface::EnableProductRemovalMessage <interfaces/ConveyorConfigInterface.h>
 * EnableProductRemovalMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
ConveyorConfigInterface::EnableProductRemovalMessage::EnableProductRemovalMessage() : Message("EnableProductRemovalMessage")
{
  data_size = sizeof(EnableProductRemovalMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (EnableProductRemovalMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
ConveyorConfigInterface::EnableProductRemovalMessage::~EnableProductRemovalMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ConveyorConfigInterface::EnableProductRemovalMessage::EnableProductRemovalMessage(const EnableProductRemovalMessage *m) : Message("EnableProductRemovalMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (EnableProductRemovalMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ConveyorConfigInterface::EnableProductRemovalMessage::clone() const
{
  return new ConveyorConfigInterface::EnableProductRemovalMessage(this);
}
/** @class ConveyorConfigInterface::DisableProductRemovalMessage <interfaces/ConveyorConfigInterface.h>
 * DisableProductRemovalMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
ConveyorConfigInterface::DisableProductRemovalMessage::DisableProductRemovalMessage() : Message("DisableProductRemovalMessage")
{
  data_size = sizeof(DisableProductRemovalMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (DisableProductRemovalMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
ConveyorConfigInterface::DisableProductRemovalMessage::~DisableProductRemovalMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ConveyorConfigInterface::DisableProductRemovalMessage::DisableProductRemovalMessage(const DisableProductRemovalMessage *m) : Message("DisableProductRemovalMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (DisableProductRemovalMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ConveyorConfigInterface::DisableProductRemovalMessage::clone() const
{
  return new ConveyorConfigInterface::DisableProductRemovalMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
ConveyorConfigInterface::message_valid(const Message *message) const
{
  const EnableProductRemovalMessage *m0 = dynamic_cast<const EnableProductRemovalMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const DisableProductRemovalMessage *m1 = dynamic_cast<const DisableProductRemovalMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ConveyorConfigInterface)
/// @endcond


} // end namespace fawkes
