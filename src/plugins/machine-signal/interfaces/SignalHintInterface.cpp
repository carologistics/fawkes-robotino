
/***************************************************************************
 *  SignalHintInterface.cpp - Fawkes BlackBoard Interface - SignalHintInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Victor Mataré
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

#include <interfaces/SignalHintInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class SignalHintInterface <interfaces/SignalHintInterface.h>
 * SignalHintInterface Fawkes BlackBoard Interface.
 * 
			This field is only here to satisfy ffifacegen. This interface is
			only used to send messages to the machine-signal plugin
		
 * @ingroup FawkesInterfaces
 */



/** Constructor */
SignalHintInterface::SignalHintInterface() : Interface()
{
  data_size = sizeof(SignalHintInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (SignalHintInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_DOUBLE, "translation", 3, &data->translation);
  add_messageinfo("SignalPositionMessage");
  unsigned char tmp_hash[] = {0xfd, 0xe6, 0x54, 0xbf, 0xdf, 0x60, 0xc3, 0xc4, 0x72, 0x36, 0x62, 0xda, 0xf7, 0xbf, 0x1b, 0xb1};
  set_hash(tmp_hash);
}

/** Destructor */
SignalHintInterface::~SignalHintInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get translation value.
 * 
			Last set position estimate, i.e. the one currently used (data fields
			are mandatory?).
		
 * @return translation value
 */
double *
SignalHintInterface::translation() const
{
  return data->translation;
}

/** Get translation value at given index.
 * 
			Last set position estimate, i.e. the one currently used (data fields
			are mandatory?).
		
 * @param index index of value
 * @return translation value
 * @exception Exception thrown if index is out of bounds
 */
double
SignalHintInterface::translation(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->translation[index];
}

/** Get maximum length of translation value.
 * @return length of translation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SignalHintInterface::maxlenof_translation() const
{
  return 3;
}

/** Set translation value.
 * 
			Last set position estimate, i.e. the one currently used (data fields
			are mandatory?).
		
 * @param new_translation new translation value
 */
void
SignalHintInterface::set_translation(const double * new_translation)
{
  memcpy(data->translation, new_translation, sizeof(double) * 3);
  data_changed = true;
}

/** Set translation value at given index.
 * 
			Last set position estimate, i.e. the one currently used (data fields
			are mandatory?).
		
 * @param new_translation new translation value
 * @param index index for of the value
 */
void
SignalHintInterface::set_translation(unsigned int index, const double new_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->translation[index] = new_translation;
  data_changed = true;
}
/* =========== message create =========== */
Message *
SignalHintInterface::create_message(const char *type) const
{
  if ( strncmp("SignalPositionMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SignalPositionMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
SignalHintInterface::copy_values(const Interface *other)
{
  const SignalHintInterface *oi = dynamic_cast<const SignalHintInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(SignalHintInterface_data_t));
}

const char *
SignalHintInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class SignalHintInterface::SignalPositionMessage <interfaces/SignalHintInterface.h>
 * SignalPositionMessage Fawkes BlackBoard Interface Message.
 * 
		
 */


/** Constructor with initial values.
 * @param ini_translation initial value for translation
 */
SignalHintInterface::SignalPositionMessage::SignalPositionMessage(const double * ini_translation) : Message("SignalPositionMessage")
{
  data_size = sizeof(SignalPositionMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SignalPositionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  memcpy(data->translation, ini_translation, sizeof(double) * 3);
  add_fieldinfo(IFT_DOUBLE, "translation", 3, &data->translation);
}
/** Constructor */
SignalHintInterface::SignalPositionMessage::SignalPositionMessage() : Message("SignalPositionMessage")
{
  data_size = sizeof(SignalPositionMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SignalPositionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_DOUBLE, "translation", 3, &data->translation);
}

/** Destructor */
SignalHintInterface::SignalPositionMessage::~SignalPositionMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
SignalHintInterface::SignalPositionMessage::SignalPositionMessage(const SignalPositionMessage *m) : Message("SignalPositionMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SignalPositionMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get translation value.
 * 
			Expected position of the signal, relative to left endpoint of the
			laser-line we see when standing in front of the MPS table
		
 * @return translation value
 */
double *
SignalHintInterface::SignalPositionMessage::translation() const
{
  return data->translation;
}

/** Get translation value at given index.
 * 
			Expected position of the signal, relative to left endpoint of the
			laser-line we see when standing in front of the MPS table
		
 * @param index index of value
 * @return translation value
 * @exception Exception thrown if index is out of bounds
 */
double
SignalHintInterface::SignalPositionMessage::translation(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->translation[index];
}

/** Get maximum length of translation value.
 * @return length of translation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
SignalHintInterface::SignalPositionMessage::maxlenof_translation() const
{
  return 3;
}

/** Set translation value.
 * 
			Expected position of the signal, relative to left endpoint of the
			laser-line we see when standing in front of the MPS table
		
 * @param new_translation new translation value
 */
void
SignalHintInterface::SignalPositionMessage::set_translation(const double * new_translation)
{
  memcpy(data->translation, new_translation, sizeof(double) * 3);
}

/** Set translation value at given index.
 * 
			Expected position of the signal, relative to left endpoint of the
			laser-line we see when standing in front of the MPS table
		
 * @param new_translation new translation value
 * @param index index for of the value
 */
void
SignalHintInterface::SignalPositionMessage::set_translation(unsigned int index, const double new_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->translation[index] = new_translation;
}
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
SignalHintInterface::SignalPositionMessage::clone() const
{
  return new SignalHintInterface::SignalPositionMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
SignalHintInterface::message_valid(const Message *message) const
{
  const SignalPositionMessage *m0 = dynamic_cast<const SignalPositionMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(SignalHintInterface)
/// @endcond


} // end namespace fawkes
