
/***************************************************************************
 *  MPSRecognitionInterface.cpp - Fawkes BlackBoard Interface - MPSRecognitionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  David Schmidt
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

#include <interfaces/MPSRecognitionInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class MPSRecognitionInterface <interfaces/MPSRecognitionInterface.h>
 * MPSRecognitionInterface Fawkes BlackBoard Interface.
 * 
      This interface allows to control mps recognition plugin, e.g. take data, compute machine type, clear taken data.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
MPSRecognitionInterface::MPSRecognitionInterface() : Interface()
{
  data_size = sizeof(MPSRecognitionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (MPSRecognitionInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_MPSType[(int)NoStationDetected] = "NoStationDetected";
  enum_map_MPSType[(int)BS] = "BS";
  enum_map_MPSType[(int)CS] = "CS";
  enum_map_MPSType[(int)DS] = "DS";
  enum_map_MPSType[(int)RS] = "RS";
  enum_map_MPSType[(int)SS] = "SS";
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_ENUM, "mpstype", 2 , &data->mpstype, "MPSType", &enum_map_MPSType);
  add_fieldinfo(IFT_FLOAT, "p_correct", 1, &data->p_correct);
  add_fieldinfo(IFT_FLOAT, "ptot_mpstype", 5, &data->ptot_mpstype);
  add_messageinfo("ClearMessage");
  add_messageinfo("ComputeMessage");
  add_messageinfo("TakeDataMessage");
  unsigned char tmp_hash[] = {0xed, 0xfc, 0xf7, 0x8d, 0x87, 0xc8, 0x69, 0xf2, 0x4f, 0xd0, 0xb1, 0x1f, 0x57, 0xf3, 0xb7, 0xc0};
  set_hash(tmp_hash);
}

/** Destructor */
MPSRecognitionInterface::~MPSRecognitionInterface()
{
  free(data_ptr);
}
/** Convert MPSType constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
MPSRecognitionInterface::tostring_MPSType(MPSType value) const
{
  switch (value) {
  case NoStationDetected: return "NoStationDetected";
  case BS: return "BS";
  case CS: return "CS";
  case DS: return "DS";
  case RS: return "RS";
  case SS: return "SS";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get msgid value.
 * 
      The ID of the message that is currently being processed or
      was processed last.
    
 * @return msgid value
 */
uint32_t
MPSRecognitionInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MPSRecognitionInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * 
      The ID of the message that is currently being processed or
      was processed last.
    
 * @param new_msgid new msgid value
 */
void
MPSRecognitionInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * 
      True, if the last recognition triggered by a ComputeMessage has
      been completed, false if it is still running. Also check the
      msgid field if this field applies to the correct message.
    
 * @return final value
 */
bool
MPSRecognitionInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MPSRecognitionInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * 
      True, if the last recognition triggered by a ComputeMessage has
      been completed, false if it is still running. Also check the
      msgid field if this field applies to the correct message.
    
 * @param new_final new final value
 */
void
MPSRecognitionInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get mpstype value.
 * 
      The result of the recognized mps.
    
 * @return mpstype value
 */
MPSRecognitionInterface::MPSType
MPSRecognitionInterface::mpstype() const
{
  return (MPSRecognitionInterface::MPSType)data->mpstype;
}

/** Get maximum length of mpstype value.
 * @return length of mpstype value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MPSRecognitionInterface::maxlenof_mpstype() const
{
  return 1;
}

/** Set mpstype value.
 * 
      The result of the recognized mps.
    
 * @param new_mpstype new mpstype value
 */
void
MPSRecognitionInterface::set_mpstype(const MPSType new_mpstype)
{
  data->mpstype = new_mpstype;
  data_changed = true;
}

/** Get p_correct value.
 * 
      The probability, that the determined type is correct.
    
 * @return p_correct value
 */
float
MPSRecognitionInterface::p_correct() const
{
  return data->p_correct;
}

/** Get maximum length of p_correct value.
 * @return length of p_correct value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MPSRecognitionInterface::maxlenof_p_correct() const
{
  return 1;
}

/** Set p_correct value.
 * 
      The probability, that the determined type is correct.
    
 * @param new_p_correct new p_correct value
 */
void
MPSRecognitionInterface::set_p_correct(const float new_p_correct)
{
  data->p_correct = new_p_correct;
  data_changed = true;
}

/** Get ptot_mpstype value.
 * 
      The total probabilities for the different mps types
    
 * @return ptot_mpstype value
 */
float *
MPSRecognitionInterface::ptot_mpstype() const
{
  return data->ptot_mpstype;
}

/** Get ptot_mpstype value at given index.
 * 
      The total probabilities for the different mps types
    
 * @param index index of value
 * @return ptot_mpstype value
 * @exception Exception thrown if index is out of bounds
 */
float
MPSRecognitionInterface::ptot_mpstype(unsigned int index) const
{
  if (index > 5) {
    throw Exception("Index value %u out of bounds (0..5)", index);
  }
  return data->ptot_mpstype[index];
}

/** Get maximum length of ptot_mpstype value.
 * @return length of ptot_mpstype value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
MPSRecognitionInterface::maxlenof_ptot_mpstype() const
{
  return 5;
}

/** Set ptot_mpstype value.
 * 
      The total probabilities for the different mps types
    
 * @param new_ptot_mpstype new ptot_mpstype value
 */
void
MPSRecognitionInterface::set_ptot_mpstype(const float * new_ptot_mpstype)
{
  memcpy(data->ptot_mpstype, new_ptot_mpstype, sizeof(float) * 5);
  data_changed = true;
}

/** Set ptot_mpstype value at given index.
 * 
      The total probabilities for the different mps types
    
 * @param new_ptot_mpstype new ptot_mpstype value
 * @param index index for of the value
 */
void
MPSRecognitionInterface::set_ptot_mpstype(unsigned int index, const float new_ptot_mpstype)
{
  if (index > 5) {
    throw Exception("Index value %u out of bounds (0..5)", index);
  }
  data->ptot_mpstype[index] = new_ptot_mpstype;
  data_changed = true;
}
/* =========== message create =========== */
Message *
MPSRecognitionInterface::create_message(const char *type) const
{
  if ( strncmp("ClearMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ClearMessage();
  } else if ( strncmp("ComputeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ComputeMessage();
  } else if ( strncmp("TakeDataMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TakeDataMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
MPSRecognitionInterface::copy_values(const Interface *other)
{
  const MPSRecognitionInterface *oi = dynamic_cast<const MPSRecognitionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(MPSRecognitionInterface_data_t));
}

const char *
MPSRecognitionInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "MPSType") == 0) {
    return tostring_MPSType((MPSType)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class MPSRecognitionInterface::ClearMessage <interfaces/MPSRecognitionInterface.h>
 * ClearMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
MPSRecognitionInterface::ClearMessage::ClearMessage() : Message("ClearMessage")
{
  data_size = sizeof(ClearMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ClearMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_MPSType[(int)NoStationDetected] = "NoStationDetected";
  enum_map_MPSType[(int)BS] = "BS";
  enum_map_MPSType[(int)CS] = "CS";
  enum_map_MPSType[(int)DS] = "DS";
  enum_map_MPSType[(int)RS] = "RS";
  enum_map_MPSType[(int)SS] = "SS";
}

/** Destructor */
MPSRecognitionInterface::ClearMessage::~ClearMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MPSRecognitionInterface::ClearMessage::ClearMessage(const ClearMessage *m) : Message("ClearMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ClearMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MPSRecognitionInterface::ClearMessage::clone() const
{
  return new MPSRecognitionInterface::ClearMessage(this);
}
/** @class MPSRecognitionInterface::ComputeMessage <interfaces/MPSRecognitionInterface.h>
 * ComputeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
MPSRecognitionInterface::ComputeMessage::ComputeMessage() : Message("ComputeMessage")
{
  data_size = sizeof(ComputeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ComputeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_MPSType[(int)NoStationDetected] = "NoStationDetected";
  enum_map_MPSType[(int)BS] = "BS";
  enum_map_MPSType[(int)CS] = "CS";
  enum_map_MPSType[(int)DS] = "DS";
  enum_map_MPSType[(int)RS] = "RS";
  enum_map_MPSType[(int)SS] = "SS";
}

/** Destructor */
MPSRecognitionInterface::ComputeMessage::~ComputeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MPSRecognitionInterface::ComputeMessage::ComputeMessage(const ComputeMessage *m) : Message("ComputeMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ComputeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MPSRecognitionInterface::ComputeMessage::clone() const
{
  return new MPSRecognitionInterface::ComputeMessage(this);
}
/** @class MPSRecognitionInterface::TakeDataMessage <interfaces/MPSRecognitionInterface.h>
 * TakeDataMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
MPSRecognitionInterface::TakeDataMessage::TakeDataMessage() : Message("TakeDataMessage")
{
  data_size = sizeof(TakeDataMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TakeDataMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_MPSType[(int)NoStationDetected] = "NoStationDetected";
  enum_map_MPSType[(int)BS] = "BS";
  enum_map_MPSType[(int)CS] = "CS";
  enum_map_MPSType[(int)DS] = "DS";
  enum_map_MPSType[(int)RS] = "RS";
  enum_map_MPSType[(int)SS] = "SS";
}

/** Destructor */
MPSRecognitionInterface::TakeDataMessage::~TakeDataMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
MPSRecognitionInterface::TakeDataMessage::TakeDataMessage(const TakeDataMessage *m) : Message("TakeDataMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TakeDataMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
MPSRecognitionInterface::TakeDataMessage::clone() const
{
  return new MPSRecognitionInterface::TakeDataMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
MPSRecognitionInterface::message_valid(const Message *message) const
{
  const ClearMessage *m0 = dynamic_cast<const ClearMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const ComputeMessage *m1 = dynamic_cast<const ComputeMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const TakeDataMessage *m2 = dynamic_cast<const TakeDataMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(MPSRecognitionInterface)
/// @endcond


} // end namespace fawkes
