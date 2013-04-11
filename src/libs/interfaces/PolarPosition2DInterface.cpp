
/***************************************************************************
 *  PolarPosition2DInterface.cpp - Fawkes BlackBoard Interface - PolarPosition2DInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Daniel Ewert
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

#include <interfaces/PolarPosition2DInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class PolarPosition2DInterface <interfaces/PolarPosition2DInterface.h>
 * PolarPosition2DInterface Fawkes BlackBoard Interface.
 * 
      Storage for a polar coordinate in 2d. Intended to hold the angle and distance to an object
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
PolarPosition2DInterface::PolarPosition2DInterface() : Interface()
{
  data_size = sizeof(PolarPosition2DInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (PolarPosition2DInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_INT32, "visibility_history", 1, &data->visibility_history);
  add_fieldinfo(IFT_INT32, "angle", 1, &data->angle);
  add_fieldinfo(IFT_DOUBLE, "distance", 1, &data->distance);
  unsigned char tmp_hash[] = {0x25, 0x5b, 0x1b, 0x8f, 0x11, 0x8b, 0x15, 0x40, 0xa2, 0x4d, 0x84, 0xb3, 0xed, 0x6, 0x51, 0x99};
  set_hash(tmp_hash);
}

/** Destructor */
PolarPosition2DInterface::~PolarPosition2DInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get frame value.
 * 
      Reference coordinate frame for the data.
    
 * @return frame value
 */
char *
PolarPosition2DInterface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PolarPosition2DInterface::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Reference coordinate frame for the data.
    
 * @param new_frame new frame value
 */
void
PolarPosition2DInterface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame));
  data_changed = true;
}

/** Get visibility_history value.
 * 
      The visibilitiy history indicates the number of consecutive positive or negative
      sightings. If the history is negative, there have been as many negative sightings
      (object not visible) as the absolute value of the history. A positive value denotes
      as many positive sightings. 0 shall only be used during the initialization of the
      interface or if the visibility history is not updated.
    
 * @return visibility_history value
 */
int32_t
PolarPosition2DInterface::visibility_history() const
{
  return data->visibility_history;
}

/** Get maximum length of visibility_history value.
 * @return length of visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PolarPosition2DInterface::maxlenof_visibility_history() const
{
  return 1;
}

/** Set visibility_history value.
 * 
      The visibilitiy history indicates the number of consecutive positive or negative
      sightings. If the history is negative, there have been as many negative sightings
      (object not visible) as the absolute value of the history. A positive value denotes
      as many positive sightings. 0 shall only be used during the initialization of the
      interface or if the visibility history is not updated.
    
 * @param new_visibility_history new visibility_history value
 */
void
PolarPosition2DInterface::set_visibility_history(const int32_t new_visibility_history)
{
  data->visibility_history = new_visibility_history;
  data_changed = true;
}

/** Get angle value.
 * 
     Angle between the x-axis of frame (e.g. the pov of the robot) and the center of 
     the detected object in degree
    
 * @return angle value
 */
int32_t
PolarPosition2DInterface::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PolarPosition2DInterface::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * 
     Angle between the x-axis of frame (e.g. the pov of the robot) and the center of 
     the detected object in degree
    
 * @param new_angle new angle value
 */
void
PolarPosition2DInterface::set_angle(const int32_t new_angle)
{
  data->angle = new_angle;
  data_changed = true;
}

/** Get distance value.
 * 
     distance between origin of frame and center of the detected object
    
 * @return distance value
 */
double
PolarPosition2DInterface::distance() const
{
  return data->distance;
}

/** Get maximum length of distance value.
 * @return length of distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PolarPosition2DInterface::maxlenof_distance() const
{
  return 1;
}

/** Set distance value.
 * 
     distance between origin of frame and center of the detected object
    
 * @param new_distance new distance value
 */
void
PolarPosition2DInterface::set_distance(const double new_distance)
{
  data->distance = new_distance;
  data_changed = true;
}

/* =========== message create =========== */
Message *
PolarPosition2DInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
PolarPosition2DInterface::copy_values(const Interface *other)
{
  const PolarPosition2DInterface *oi = dynamic_cast<const PolarPosition2DInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(PolarPosition2DInterface_data_t));
}

const char *
PolarPosition2DInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
PolarPosition2DInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(PolarPosition2DInterface)
/// @endcond


} // end namespace fawkes
