
/***************************************************************************
 *  ZoneInterface.h - Fawkes BlackBoard Interface - ZoneInterface
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

#ifndef __INTERFACES_ZONEINTERFACE_H_
#define __INTERFACES_ZONEINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class ZoneInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(ZoneInterface)
 /// @endcond
 public:
  /* constants */

  /** The information about the MPS in a zone */
  typedef enum {
    UNKNOWN /**< We did not detect it yet, but can't say no */,
    NO /**< We search everthing, there might be no MPS in the zone (or the skill can't find it) */,
    YES /**< We found the MPS and in the corresponding Position 3D interface are the possition */,
    MAYBE /**< We found the MPS at a far possition, but not from close */
  } MPS_IN_ZONE;
  const char * tostring_MPS_IN_ZONE(MPS_IN_ZONE value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t search_state; /**< 
      The state of the search of the actual zone.
     */
    int32_t tag_id; /**< 
      The ID of the tag, if set to -1 this means no TAG is known, e.g. MPS just found maybe (with laser-lines).
     */
  } ZoneInterface_data_t;
#pragma pack(pop)

  ZoneInterface_data_t *data;

  interface_enum_map_t enum_map_MPS_IN_ZONE;
 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  ZoneInterface();
  ~ZoneInterface();

 public:
  /* Methods */
  MPS_IN_ZONE search_state() const;
  void set_search_state(const MPS_IN_ZONE new_search_state);
  size_t maxlenof_search_state() const;
  int32_t tag_id() const;
  void set_tag_id(const int32_t new_tag_id);
  size_t maxlenof_tag_id() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
