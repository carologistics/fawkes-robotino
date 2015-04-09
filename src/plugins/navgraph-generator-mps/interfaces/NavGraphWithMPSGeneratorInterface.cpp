
/***************************************************************************
 *  NavGraphWithMPSGeneratorInterface.cpp - Fawkes BlackBoard Interface - NavGraphWithMPSGeneratorInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Tim Niemueller
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

#include <interfaces/NavGraphWithMPSGeneratorInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class NavGraphWithMPSGeneratorInterface <interfaces/NavGraphWithMPSGeneratorInterface.h>
 * NavGraphWithMPSGeneratorInterface Fawkes BlackBoard Interface.
 * 
      This interface allows to announce MPS stations to navgraph-generator-mps.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
NavGraphWithMPSGeneratorInterface::NavGraphWithMPSGeneratorInterface() : Interface()
{
  data_size = sizeof(NavGraphWithMPSGeneratorInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (NavGraphWithMPSGeneratorInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_Side[(int)INPUT] = "INPUT";
  enum_map_Side[(int)OUTPUT] = "OUTPUT";
  add_fieldinfo(IFT_STRING, "last_id", 64, data->last_id);
  add_messageinfo("ClearMessage");
  add_messageinfo("ComputeMessage");
  add_messageinfo("UpdateStationByTagMessage");
  unsigned char tmp_hash[] = {0xc6, 0x9c, 0xa9, 0xf1, 0x44, 0x8a, 0xae, 0x36, 0x80, 0x4c, 0xb6, 0x7, 0x48, 0xac, 0x20, 0xe8};
  set_hash(tmp_hash);
}

/** Destructor */
NavGraphWithMPSGeneratorInterface::~NavGraphWithMPSGeneratorInterface()
{
  free(data_ptr);
}
/** Convert Side constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
NavGraphWithMPSGeneratorInterface::tostring_Side(Side value) const
{
  switch (value) {
  case INPUT: return "INPUT";
  case OUTPUT: return "OUTPUT";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get last_id value.
 * 
      Name of last added station.
    
 * @return last_id value
 */
char *
NavGraphWithMPSGeneratorInterface::last_id() const
{
  return data->last_id;
}

/** Get maximum length of last_id value.
 * @return length of last_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphWithMPSGeneratorInterface::maxlenof_last_id() const
{
  return 64;
}

/** Set last_id value.
 * 
      Name of last added station.
    
 * @param new_last_id new last_id value
 */
void
NavGraphWithMPSGeneratorInterface::set_last_id(const char * new_last_id)
{
  strncpy(data->last_id, new_last_id, sizeof(data->last_id));
  data_changed = true;
}

/* =========== message create =========== */
Message *
NavGraphWithMPSGeneratorInterface::create_message(const char *type) const
{
  if ( strncmp("ClearMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ClearMessage();
  } else if ( strncmp("ComputeMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ComputeMessage();
  } else if ( strncmp("UpdateStationByTagMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new UpdateStationByTagMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
NavGraphWithMPSGeneratorInterface::copy_values(const Interface *other)
{
  const NavGraphWithMPSGeneratorInterface *oi = dynamic_cast<const NavGraphWithMPSGeneratorInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(NavGraphWithMPSGeneratorInterface_data_t));
}

const char *
NavGraphWithMPSGeneratorInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "Side") == 0) {
    return tostring_Side((Side)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class NavGraphWithMPSGeneratorInterface::ClearMessage <interfaces/NavGraphWithMPSGeneratorInterface.h>
 * ClearMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NavGraphWithMPSGeneratorInterface::ClearMessage::ClearMessage() : Message("ClearMessage")
{
  data_size = sizeof(ClearMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ClearMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Side[(int)INPUT] = "INPUT";
  enum_map_Side[(int)OUTPUT] = "OUTPUT";
}

/** Destructor */
NavGraphWithMPSGeneratorInterface::ClearMessage::~ClearMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphWithMPSGeneratorInterface::ClearMessage::ClearMessage(const ClearMessage *m) : Message("ClearMessage")
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
NavGraphWithMPSGeneratorInterface::ClearMessage::clone() const
{
  return new NavGraphWithMPSGeneratorInterface::ClearMessage(this);
}
/** @class NavGraphWithMPSGeneratorInterface::ComputeMessage <interfaces/NavGraphWithMPSGeneratorInterface.h>
 * ComputeMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
NavGraphWithMPSGeneratorInterface::ComputeMessage::ComputeMessage() : Message("ComputeMessage")
{
  data_size = sizeof(ComputeMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ComputeMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Side[(int)INPUT] = "INPUT";
  enum_map_Side[(int)OUTPUT] = "OUTPUT";
}

/** Destructor */
NavGraphWithMPSGeneratorInterface::ComputeMessage::~ComputeMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphWithMPSGeneratorInterface::ComputeMessage::ComputeMessage(const ComputeMessage *m) : Message("ComputeMessage")
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
NavGraphWithMPSGeneratorInterface::ComputeMessage::clone() const
{
  return new NavGraphWithMPSGeneratorInterface::ComputeMessage(this);
}
/** @class NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage <interfaces/NavGraphWithMPSGeneratorInterface.h>
 * UpdateStationByTagMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_id initial value for id
 * @param ini_side initial value for side
 * @param ini_frame initial value for frame
 * @param ini_tag_translation initial value for tag_translation
 * @param ini_tag_rotation initial value for tag_rotation
 */
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::UpdateStationByTagMessage(const char * ini_id, const Side ini_side, const char * ini_frame, const double * ini_tag_translation, const double * ini_tag_rotation) : Message("UpdateStationByTagMessage")
{
  data_size = sizeof(UpdateStationByTagMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (UpdateStationByTagMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->id, ini_id, 64);
  data->side = ini_side;
  strncpy(data->frame, ini_frame, 32);
  memcpy(data->tag_translation, ini_tag_translation, sizeof(double) * 3);
  memcpy(data->tag_rotation, ini_tag_rotation, sizeof(double) * 4);
  enum_map_Side[(int)INPUT] = "INPUT";
  enum_map_Side[(int)OUTPUT] = "OUTPUT";
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_ENUM, "side", 1, &data->side, "Side", &enum_map_Side);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_DOUBLE, "tag_translation", 3, &data->tag_translation);
  add_fieldinfo(IFT_DOUBLE, "tag_rotation", 4, &data->tag_rotation);
}
/** Constructor */
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::UpdateStationByTagMessage() : Message("UpdateStationByTagMessage")
{
  data_size = sizeof(UpdateStationByTagMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (UpdateStationByTagMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  enum_map_Side[(int)INPUT] = "INPUT";
  enum_map_Side[(int)OUTPUT] = "OUTPUT";
  add_fieldinfo(IFT_STRING, "id", 64, data->id);
  add_fieldinfo(IFT_ENUM, "side", 1, &data->side, "Side", &enum_map_Side);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_DOUBLE, "tag_translation", 3, &data->tag_translation);
  add_fieldinfo(IFT_DOUBLE, "tag_rotation", 4, &data->tag_rotation);
}

/** Destructor */
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::~UpdateStationByTagMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::UpdateStationByTagMessage(const UpdateStationByTagMessage *m) : Message("UpdateStationByTagMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (UpdateStationByTagMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get id value.
 * 
      ID of the obstacle. Can later be used to remove it again.
    
 * @return id value
 */
char *
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::id() const
{
  return data->id;
}

/** Get maximum length of id value.
 * @return length of id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::maxlenof_id() const
{
  return 64;
}

/** Set id value.
 * 
      ID of the obstacle. Can later be used to remove it again.
    
 * @param new_id new id value
 */
void
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::set_id(const char * new_id)
{
  strncpy(data->id, new_id, sizeof(data->id));
}

/** Get side value.
 * Which side are we reporting.
 * @return side value
 */
NavGraphWithMPSGeneratorInterface::Side
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::side() const
{
  return (NavGraphWithMPSGeneratorInterface::Side)data->side;
}

/** Get maximum length of side value.
 * @return length of side value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::maxlenof_side() const
{
  return 1;
}

/** Set side value.
 * Which side are we reporting.
 * @param new_side new side value
 */
void
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::set_side(const Side new_side)
{
  data->side = new_side;
}

/** Get frame value.
 * 
      Coordinate reference frame of tag.
    
 * @return frame value
 */
char *
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Coordinate reference frame of tag.
    
 * @param new_frame new frame value
 */
void
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame));
}

/** Get tag_translation value.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @return tag_translation value
 */
double *
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::tag_translation() const
{
  return data->tag_translation;
}

/** Get tag_translation value at given index.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @param index index of value
 * @return tag_translation value
 * @exception Exception thrown if index is out of bounds
 */
double
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::tag_translation(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->tag_translation[index];
}

/** Get maximum length of tag_translation value.
 * @return length of tag_translation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::maxlenof_tag_translation() const
{
  return 3;
}

/** Set tag_translation value.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @param new_tag_translation new tag_translation value
 */
void
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::set_tag_translation(const double * new_tag_translation)
{
  memcpy(data->tag_translation, new_tag_translation, sizeof(double) * 3);
}

/** Set tag_translation value at given index.
 * 
      Translation vector from the reference frame's origin, ordered as (x, y, z).
    
 * @param new_tag_translation new tag_translation value
 * @param index index for of the value
 */
void
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::set_tag_translation(unsigned int index, const double new_tag_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->tag_translation[index] = new_tag_translation;
}
/** Get tag_rotation value.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @return tag_rotation value
 */
double *
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::tag_rotation() const
{
  return data->tag_rotation;
}

/** Get tag_rotation value at given index.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @param index index of value
 * @return tag_rotation value
 * @exception Exception thrown if index is out of bounds
 */
double
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::tag_rotation(unsigned int index) const
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  return data->tag_rotation[index];
}

/** Get maximum length of tag_rotation value.
 * @return length of tag_rotation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::maxlenof_tag_rotation() const
{
  return 4;
}

/** Set tag_rotation value.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @param new_tag_rotation new tag_rotation value
 */
void
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::set_tag_rotation(const double * new_tag_rotation)
{
  memcpy(data->tag_rotation, new_tag_rotation, sizeof(double) * 4);
}

/** Set tag_rotation value at given index.
 * 
      Rotation quaternion relative to reference frame, ordered as (x, y, z, w).
    
 * @param new_tag_rotation new tag_rotation value
 * @param index index for of the value
 */
void
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::set_tag_rotation(unsigned int index, const double new_tag_rotation)
{
  if (index > 4) {
    throw Exception("Index value %u out of bounds (0..4)", index);
  }
  data->tag_rotation[index] = new_tag_rotation;
}
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage::clone() const
{
  return new NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
NavGraphWithMPSGeneratorInterface::message_valid(const Message *message) const
{
  const ClearMessage *m0 = dynamic_cast<const ClearMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const ComputeMessage *m1 = dynamic_cast<const ComputeMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const UpdateStationByTagMessage *m2 = dynamic_cast<const UpdateStationByTagMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(NavGraphWithMPSGeneratorInterface)
/// @endcond


} // end namespace fawkes
