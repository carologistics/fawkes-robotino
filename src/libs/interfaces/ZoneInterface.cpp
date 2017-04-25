
/***************************************************************************
 *  ZoneInterface.cpp - Fawkes BlackBoard Interface - ZoneInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Tobias Neumann
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

#include <interfaces/ZoneInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ZoneInterface <interfaces/ZoneInterface.h>
 * ZoneInterface Fawkes BlackBoard Interface.
 * 
      The interface about the zone that is to be explored
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
ZoneInterface::ZoneInterface() : Interface()
{
  data_size = sizeof(ZoneInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ZoneInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  enum_map_MPS_IN_ZONE[(int)UNKNOWN] = "UNKNOWN";
  enum_map_MPS_IN_ZONE[(int)NO] = "NO";
  enum_map_MPS_IN_ZONE[(int)YES] = "YES";
  enum_map_MPS_IN_ZONE[(int)MAYBE] = "MAYBE";
  add_fieldinfo(IFT_ENUM, "search_state", 1, &data->search_state, "MPS_IN_ZONE", &enum_map_MPS_IN_ZONE);
  add_fieldinfo(IFT_INT32, "tag_id", 1, &data->tag_id);
  add_fieldinfo(IFT_INT32, "orientation", 1, &data->orientation);
  add_fieldinfo(IFT_STRING, "zone", 16, data->zone);
  unsigned char tmp_hash[] = {0x84, 0xeb, 0x2d, 0xe5, 0xd3, 0x17, 0xe1, 0x8c, 0x75, 0xb3, 0xe0, 0x94, 0x8e, 0x5a, 0xbb, 0xc6};
  set_hash(tmp_hash);
}

/** Destructor */
ZoneInterface::~ZoneInterface()
{
  free(data_ptr);
}
/** Convert MPS_IN_ZONE constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
ZoneInterface::tostring_MPS_IN_ZONE(MPS_IN_ZONE value) const
{
  switch (value) {
  case UNKNOWN: return "UNKNOWN";
  case NO: return "NO";
  case YES: return "YES";
  case MAYBE: return "MAYBE";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get search_state value.
 * 
      The state of the search of the actual zone.
    
 * @return search_state value
 */
ZoneInterface::MPS_IN_ZONE
ZoneInterface::search_state() const
{
  return (ZoneInterface::MPS_IN_ZONE)data->search_state;
}

/** Get maximum length of search_state value.
 * @return length of search_state value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ZoneInterface::maxlenof_search_state() const
{
  return 1;
}

/** Set search_state value.
 * 
      The state of the search of the actual zone.
    
 * @param new_search_state new search_state value
 */
void
ZoneInterface::set_search_state(const MPS_IN_ZONE new_search_state)
{
  data->search_state = new_search_state;
  data_changed = true;
}

/** Get tag_id value.
 * 
      The ID of the tag, if set to -1 this means no TAG is known, e.g. MPS just found maybe (with laser-lines).
    
 * @return tag_id value
 */
int32_t
ZoneInterface::tag_id() const
{
  return data->tag_id;
}

/** Get maximum length of tag_id value.
 * @return length of tag_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ZoneInterface::maxlenof_tag_id() const
{
  return 1;
}

/** Set tag_id value.
 * 
      The ID of the tag, if set to -1 this means no TAG is known, e.g. MPS just found maybe (with laser-lines).
    
 * @param new_tag_id new tag_id value
 */
void
ZoneInterface::set_tag_id(const int32_t new_tag_id)
{
  data->tag_id = new_tag_id;
  data_changed = true;
}

/** Get orientation value.
 * 
      The discretized orientation of the MPS
    
 * @return orientation value
 */
int32_t
ZoneInterface::orientation() const
{
  return data->orientation;
}

/** Get maximum length of orientation value.
 * @return length of orientation value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ZoneInterface::maxlenof_orientation() const
{
  return 1;
}

/** Set orientation value.
 * 
      The discretized orientation of the MPS
    
 * @param new_orientation new orientation value
 */
void
ZoneInterface::set_orientation(const int32_t new_orientation)
{
  data->orientation = new_orientation;
  data_changed = true;
}

/** Get zone value.
 * 
      The name of the zone we're investigating
    
 * @return zone value
 */
char *
ZoneInterface::zone() const
{
  return data->zone;
}

/** Get maximum length of zone value.
 * @return length of zone value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ZoneInterface::maxlenof_zone() const
{
  return 16;
}

/** Set zone value.
 * 
      The name of the zone we're investigating
    
 * @param new_zone new zone value
 */
void
ZoneInterface::set_zone(const char * new_zone)
{
  strncpy(data->zone, new_zone, sizeof(data->zone));
  data_changed = true;
}

/* =========== message create =========== */
Message *
ZoneInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
ZoneInterface::copy_values(const Interface *other)
{
  const ZoneInterface *oi = dynamic_cast<const ZoneInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(ZoneInterface_data_t));
}

const char *
ZoneInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "MPS_IN_ZONE") == 0) {
    return tostring_MPS_IN_ZONE((MPS_IN_ZONE)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
ZoneInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ZoneInterface)
/// @endcond


} // end namespace fawkes
