
/***************************************************************************
 *  PuckVisionInterface.cpp - Fawkes BlackBoard Interface - PuckVisionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Florian Nolden
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

#include <interfaces/PuckVisionInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class PuckVisionInterface <interfaces/PuckVisionInterface.h>
 * PuckVisionInterface Fawkes BlackBoard Interface.
 * 
     Puck detection interface
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
PuckVisionInterface::PuckVisionInterface() : Interface()
{
  data_size = sizeof(PuckVisionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (PuckVisionInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_ENUM, "puck1_color", 1, &data->puck1_color, "PuckColor");
  add_fieldinfo(IFT_ENUM, "puck2_color", 1, &data->puck2_color, "PuckColor");
  add_fieldinfo(IFT_ENUM, "puck3_color", 1, &data->puck3_color, "PuckColor");
  add_fieldinfo(IFT_INT32, "puck1_visibility_history", 1, &data->puck1_visibility_history);
  add_fieldinfo(IFT_INT32, "puck2_visibility_history", 1, &data->puck2_visibility_history);
  add_fieldinfo(IFT_INT32, "puck3_visibility_history", 1, &data->puck3_visibility_history);
  add_fieldinfo(IFT_DOUBLE, "puck1_translation", 3, &data->puck1_translation);
  add_fieldinfo(IFT_DOUBLE, "puck2_translation", 3, &data->puck2_translation);
  add_fieldinfo(IFT_DOUBLE, "puck3_translation", 3, &data->puck3_translation);
  add_fieldinfo(IFT_DOUBLE, "puck1_polar", 2, &data->puck1_polar);
  add_fieldinfo(IFT_DOUBLE, "puck2_polar", 2, &data->puck2_polar);
  add_fieldinfo(IFT_DOUBLE, "puck3_polar", 2, &data->puck3_polar);
  unsigned char tmp_hash[] = {0x73, 0x39, 0x58, 0x47, 00, 0xd2, 0x94, 0x8c, 0x21, 0x97, 0x50, 0x58, 0x32, 0xe6, 0x6d, 0xd1};
  set_hash(tmp_hash);
}

/** Destructor */
PuckVisionInterface::~PuckVisionInterface()
{
  free(data_ptr);
}
/** Convert PuckColor constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
PuckVisionInterface::tostring_PuckColor(PuckColor value) const
{
  switch (value) {
  case C_RED: return "C_RED";
  case C_GREEN: return "C_GREEN";
  case C_BLUE: return "C_BLUE";
  case C_BLACK: return "C_BLACK";
  case C_YELLOW: return "C_YELLOW";
  case C_WHITE: return "C_WHITE";
  case C_UNKNOWN: return "C_UNKNOWN";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get frame value.
 * 
	      Reference coordinate frame for the data.
    	
 * @return frame value
 */
char *
PuckVisionInterface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
	      Reference coordinate frame for the data.
    	
 * @param new_frame new frame value
 */
void
PuckVisionInterface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame));
  data_changed = true;
}

/** Get puck1_color value.
 * Detected Puck Color
 * @return puck1_color value
 */
PuckVisionInterface::PuckColor
PuckVisionInterface::puck1_color() const
{
  return (PuckVisionInterface::PuckColor)data->puck1_color;
}

/** Get maximum length of puck1_color value.
 * @return length of puck1_color value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck1_color() const
{
  return 1;
}

/** Set puck1_color value.
 * Detected Puck Color
 * @param new_puck1_color new puck1_color value
 */
void
PuckVisionInterface::set_puck1_color(const PuckColor new_puck1_color)
{
  data->puck1_color = new_puck1_color;
  data_changed = true;
}

/** Get puck2_color value.
 * Detected Puck Color
 * @return puck2_color value
 */
PuckVisionInterface::PuckColor
PuckVisionInterface::puck2_color() const
{
  return (PuckVisionInterface::PuckColor)data->puck2_color;
}

/** Get maximum length of puck2_color value.
 * @return length of puck2_color value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck2_color() const
{
  return 1;
}

/** Set puck2_color value.
 * Detected Puck Color
 * @param new_puck2_color new puck2_color value
 */
void
PuckVisionInterface::set_puck2_color(const PuckColor new_puck2_color)
{
  data->puck2_color = new_puck2_color;
  data_changed = true;
}

/** Get puck3_color value.
 * Detected Puck Color
 * @return puck3_color value
 */
PuckVisionInterface::PuckColor
PuckVisionInterface::puck3_color() const
{
  return (PuckVisionInterface::PuckColor)data->puck3_color;
}

/** Get maximum length of puck3_color value.
 * @return length of puck3_color value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck3_color() const
{
  return 1;
}

/** Set puck3_color value.
 * Detected Puck Color
 * @param new_puck3_color new puck3_color value
 */
void
PuckVisionInterface::set_puck3_color(const PuckColor new_puck3_color)
{
  data->puck3_color = new_puck3_color;
  data_changed = true;
}

/** Get puck1_visibility_history value.
 * visibility history
 * @return puck1_visibility_history value
 */
int32_t
PuckVisionInterface::puck1_visibility_history() const
{
  return data->puck1_visibility_history;
}

/** Get maximum length of puck1_visibility_history value.
 * @return length of puck1_visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck1_visibility_history() const
{
  return 1;
}

/** Set puck1_visibility_history value.
 * visibility history
 * @param new_puck1_visibility_history new puck1_visibility_history value
 */
void
PuckVisionInterface::set_puck1_visibility_history(const int32_t new_puck1_visibility_history)
{
  data->puck1_visibility_history = new_puck1_visibility_history;
  data_changed = true;
}

/** Get puck2_visibility_history value.
 * visibility history
 * @return puck2_visibility_history value
 */
int32_t
PuckVisionInterface::puck2_visibility_history() const
{
  return data->puck2_visibility_history;
}

/** Get maximum length of puck2_visibility_history value.
 * @return length of puck2_visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck2_visibility_history() const
{
  return 1;
}

/** Set puck2_visibility_history value.
 * visibility history
 * @param new_puck2_visibility_history new puck2_visibility_history value
 */
void
PuckVisionInterface::set_puck2_visibility_history(const int32_t new_puck2_visibility_history)
{
  data->puck2_visibility_history = new_puck2_visibility_history;
  data_changed = true;
}

/** Get puck3_visibility_history value.
 * visibility history
 * @return puck3_visibility_history value
 */
int32_t
PuckVisionInterface::puck3_visibility_history() const
{
  return data->puck3_visibility_history;
}

/** Get maximum length of puck3_visibility_history value.
 * @return length of puck3_visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck3_visibility_history() const
{
  return 1;
}

/** Set puck3_visibility_history value.
 * visibility history
 * @param new_puck3_visibility_history new puck3_visibility_history value
 */
void
PuckVisionInterface::set_puck3_visibility_history(const int32_t new_puck3_visibility_history)
{
  data->puck3_visibility_history = new_puck3_visibility_history;
  data_changed = true;
}

/** Get puck1_translation value.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
        
 * @return puck1_translation value
 */
double *
PuckVisionInterface::puck1_translation() const
{
  return data->puck1_translation;
}

/** Get puck1_translation value at given index.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
        
 * @param index index of value
 * @return puck1_translation value
 * @exception Exception thrown if index is out of bounds
 */
double
PuckVisionInterface::puck1_translation(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->puck1_translation[index];
}

/** Get maximum length of puck1_translation value.
 * @return length of puck1_translation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck1_translation() const
{
  return 3;
}

/** Set puck1_translation value.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
        
 * @param new_puck1_translation new puck1_translation value
 */
void
PuckVisionInterface::set_puck1_translation(const double * new_puck1_translation)
{
  memcpy(data->puck1_translation, new_puck1_translation, sizeof(double) * 3);
  data_changed = true;
}

/** Set puck1_translation value at given index.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
        
 * @param new_puck1_translation new puck1_translation value
 * @param index index for of the value
 */
void
PuckVisionInterface::set_puck1_translation(unsigned int index, const double new_puck1_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->puck1_translation[index] = new_puck1_translation;
  data_changed = true;
}
/** Get puck2_translation value.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
        
 * @return puck2_translation value
 */
double *
PuckVisionInterface::puck2_translation() const
{
  return data->puck2_translation;
}

/** Get puck2_translation value at given index.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
        
 * @param index index of value
 * @return puck2_translation value
 * @exception Exception thrown if index is out of bounds
 */
double
PuckVisionInterface::puck2_translation(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->puck2_translation[index];
}

/** Get maximum length of puck2_translation value.
 * @return length of puck2_translation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck2_translation() const
{
  return 3;
}

/** Set puck2_translation value.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
        
 * @param new_puck2_translation new puck2_translation value
 */
void
PuckVisionInterface::set_puck2_translation(const double * new_puck2_translation)
{
  memcpy(data->puck2_translation, new_puck2_translation, sizeof(double) * 3);
  data_changed = true;
}

/** Set puck2_translation value at given index.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
        
 * @param new_puck2_translation new puck2_translation value
 * @param index index for of the value
 */
void
PuckVisionInterface::set_puck2_translation(unsigned int index, const double new_puck2_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->puck2_translation[index] = new_puck2_translation;
  data_changed = true;
}
/** Get puck3_translation value.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
	
 * @return puck3_translation value
 */
double *
PuckVisionInterface::puck3_translation() const
{
  return data->puck3_translation;
}

/** Get puck3_translation value at given index.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
	
 * @param index index of value
 * @return puck3_translation value
 * @exception Exception thrown if index is out of bounds
 */
double
PuckVisionInterface::puck3_translation(unsigned int index) const
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  return data->puck3_translation[index];
}

/** Get maximum length of puck3_translation value.
 * @return length of puck3_translation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck3_translation() const
{
  return 3;
}

/** Set puck3_translation value.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
	
 * @param new_puck3_translation new puck3_translation value
 */
void
PuckVisionInterface::set_puck3_translation(const double * new_puck3_translation)
{
  memcpy(data->puck3_translation, new_puck3_translation, sizeof(double) * 3);
  data_changed = true;
}

/** Set puck3_translation value at given index.
 * Translation vector from the reference frame's origin, ordered as (x, y, z).
	
 * @param new_puck3_translation new puck3_translation value
 * @param index index for of the value
 */
void
PuckVisionInterface::set_puck3_translation(unsigned int index, const double new_puck3_translation)
{
  if (index > 3) {
    throw Exception("Index value %u out of bounds (0..3)", index);
  }
  data->puck3_translation[index] = new_puck3_translation;
  data_changed = true;
}
/** Get puck1_polar value.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @return puck1_polar value
 */
double *
PuckVisionInterface::puck1_polar() const
{
  return data->puck1_polar;
}

/** Get puck1_polar value at given index.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param index index of value
 * @return puck1_polar value
 * @exception Exception thrown if index is out of bounds
 */
double
PuckVisionInterface::puck1_polar(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->puck1_polar[index];
}

/** Get maximum length of puck1_polar value.
 * @return length of puck1_polar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck1_polar() const
{
  return 2;
}

/** Set puck1_polar value.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param new_puck1_polar new puck1_polar value
 */
void
PuckVisionInterface::set_puck1_polar(const double * new_puck1_polar)
{
  memcpy(data->puck1_polar, new_puck1_polar, sizeof(double) * 2);
  data_changed = true;
}

/** Set puck1_polar value at given index.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param new_puck1_polar new puck1_polar value
 * @param index index for of the value
 */
void
PuckVisionInterface::set_puck1_polar(unsigned int index, const double new_puck1_polar)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->puck1_polar[index] = new_puck1_polar;
  data_changed = true;
}
/** Get puck2_polar value.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @return puck2_polar value
 */
double *
PuckVisionInterface::puck2_polar() const
{
  return data->puck2_polar;
}

/** Get puck2_polar value at given index.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param index index of value
 * @return puck2_polar value
 * @exception Exception thrown if index is out of bounds
 */
double
PuckVisionInterface::puck2_polar(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->puck2_polar[index];
}

/** Get maximum length of puck2_polar value.
 * @return length of puck2_polar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck2_polar() const
{
  return 2;
}

/** Set puck2_polar value.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param new_puck2_polar new puck2_polar value
 */
void
PuckVisionInterface::set_puck2_polar(const double * new_puck2_polar)
{
  memcpy(data->puck2_polar, new_puck2_polar, sizeof(double) * 2);
  data_changed = true;
}

/** Set puck2_polar value at given index.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param new_puck2_polar new puck2_polar value
 * @param index index for of the value
 */
void
PuckVisionInterface::set_puck2_polar(unsigned int index, const double new_puck2_polar)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->puck2_polar[index] = new_puck2_polar;
  data_changed = true;
}
/** Get puck3_polar value.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @return puck3_polar value
 */
double *
PuckVisionInterface::puck3_polar() const
{
  return data->puck3_polar;
}

/** Get puck3_polar value at given index.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param index index of value
 * @return puck3_polar value
 * @exception Exception thrown if index is out of bounds
 */
double
PuckVisionInterface::puck3_polar(unsigned int index) const
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  return data->puck3_polar[index];
}

/** Get maximum length of puck3_polar value.
 * @return length of puck3_polar value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
PuckVisionInterface::maxlenof_puck3_polar() const
{
  return 2;
}

/** Set puck3_polar value.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param new_puck3_polar new puck3_polar value
 */
void
PuckVisionInterface::set_puck3_polar(const double * new_puck3_polar)
{
  memcpy(data->puck3_polar, new_puck3_polar, sizeof(double) * 2);
  data_changed = true;
}

/** Set puck3_polar value at given index.
 * Polar koordinates (Phi, R) = (x, y).
        
 * @param new_puck3_polar new puck3_polar value
 * @param index index for of the value
 */
void
PuckVisionInterface::set_puck3_polar(unsigned int index, const double new_puck3_polar)
{
  if (index > 2) {
    throw Exception("Index value %u out of bounds (0..2)", index);
  }
  data->puck3_polar[index] = new_puck3_polar;
  data_changed = true;
}
/* =========== message create =========== */
Message *
PuckVisionInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
PuckVisionInterface::copy_values(const Interface *other)
{
  const PuckVisionInterface *oi = dynamic_cast<const PuckVisionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(PuckVisionInterface_data_t));
}

const char *
PuckVisionInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "PuckColor") == 0) {
    return tostring_PuckColor((PuckColor)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
PuckVisionInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(PuckVisionInterface)
/// @endcond


} // end namespace fawkes
