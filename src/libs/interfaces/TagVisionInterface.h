
/***************************************************************************
 *  TagVisionInterface.h - Fawkes BlackBoard Interface - TagVisionInterface
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

#ifndef __INTERFACES_TAGVISIONINTERFACE_H_
#define __INTERFACES_TAGVISIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class TagVisionInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(TagVisionInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char frame[32]; /**< 
      Reference coordinate frame for the data.
     */
    int32_t tags_visible; /**< 
      The number of currently visible tags.
     */
    int32_t tag_id[16]; /**< 
      The IDs of the tags. The IDs are in the same order the Tag_nr interfaces are in
     */
  } TagVisionInterface_data_t;
#pragma pack(pop)

  TagVisionInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  TagVisionInterface();
  ~TagVisionInterface();

 public:
  /* Methods */
  char * frame() const;
  void set_frame(const char * new_frame);
  size_t maxlenof_frame() const;
  int32_t tags_visible() const;
  void set_tags_visible(const int32_t new_tags_visible);
  size_t maxlenof_tags_visible() const;
  int32_t * tag_id() const;
  int32_t tag_id(unsigned int index) const;
  void set_tag_id(unsigned int index, const int32_t new_tag_id);
  void set_tag_id(const int32_t * new_tag_id);
  size_t maxlenof_tag_id() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
