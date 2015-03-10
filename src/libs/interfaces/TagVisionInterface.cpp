
/***************************************************************************
 *  TagVisionInterface.cpp - Fawkes BlackBoard Interface - TagVisionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Randolph Maaßen
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

#include <interfaces/TagVisionInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class TagVisionInterface <interfaces/TagVisionInterface.h>
 * TagVisionInterface Fawkes BlackBoard Interface.
 * 
      Storage for information about the TagVision
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
TagVisionInterface::TagVisionInterface() : Interface()
{
  data_size = sizeof(TagVisionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (TagVisionInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_INT32, "tags_visible", 1, &data->tags_visible);
  add_fieldinfo(IFT_INT32, "tag_id", 16, &data->tag_id);
  unsigned char tmp_hash[] = {0xa5, 0xfa, 0xe5, 0xd3, 0xb9, 0x8e, 0xcc, 0x8c, 0xcc, 0xa0, 0x85, 0xc4, 0x3e, 0xd8, 0x94, 0xe9};
  set_hash(tmp_hash);
}

/** Destructor */
TagVisionInterface::~TagVisionInterface()
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
TagVisionInterface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TagVisionInterface::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Reference coordinate frame for the data.
    
 * @param new_frame new frame value
 */
void
TagVisionInterface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame));
  data_changed = true;
}

/** Get tags_visible value.
 * 
      The number of currently visible tags.
    
 * @return tags_visible value
 */
int32_t
TagVisionInterface::tags_visible() const
{
  return data->tags_visible;
}

/** Get maximum length of tags_visible value.
 * @return length of tags_visible value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TagVisionInterface::maxlenof_tags_visible() const
{
  return 1;
}

/** Set tags_visible value.
 * 
      The number of currently visible tags.
    
 * @param new_tags_visible new tags_visible value
 */
void
TagVisionInterface::set_tags_visible(const int32_t new_tags_visible)
{
  data->tags_visible = new_tags_visible;
  data_changed = true;
}

/** Get tag_id value.
 * 
      The IDs of the tags. The IDs are in the same order the Tag_nr interfaces are in
    
 * @return tag_id value
 */
int32_t *
TagVisionInterface::tag_id() const
{
  return data->tag_id;
}

/** Get tag_id value at given index.
 * 
      The IDs of the tags. The IDs are in the same order the Tag_nr interfaces are in
    
 * @param index index of value
 * @return tag_id value
 * @exception Exception thrown if index is out of bounds
 */
int32_t
TagVisionInterface::tag_id(unsigned int index) const
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  return data->tag_id[index];
}

/** Get maximum length of tag_id value.
 * @return length of tag_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TagVisionInterface::maxlenof_tag_id() const
{
  return 16;
}

/** Set tag_id value.
 * 
      The IDs of the tags. The IDs are in the same order the Tag_nr interfaces are in
    
 * @param new_tag_id new tag_id value
 */
void
TagVisionInterface::set_tag_id(const int32_t * new_tag_id)
{
  memcpy(data->tag_id, new_tag_id, sizeof(int32_t) * 16);
  data_changed = true;
}

/** Set tag_id value at given index.
 * 
      The IDs of the tags. The IDs are in the same order the Tag_nr interfaces are in
    
 * @param new_tag_id new tag_id value
 * @param index index for of the value
 */
void
TagVisionInterface::set_tag_id(unsigned int index, const int32_t new_tag_id)
{
  if (index > 16) {
    throw Exception("Index value %u out of bounds (0..16)", index);
  }
  data->tag_id[index] = new_tag_id;
  data_changed = true;
}
/* =========== message create =========== */
Message *
TagVisionInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
TagVisionInterface::copy_values(const Interface *other)
{
  const TagVisionInterface *oi = dynamic_cast<const TagVisionInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(TagVisionInterface_data_t));
}

const char *
TagVisionInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
TagVisionInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(TagVisionInterface)
/// @endcond


} // end namespace fawkes
