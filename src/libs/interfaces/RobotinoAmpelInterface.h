
/***************************************************************************
 *  RobotinoAmpelInterface.h - Fawkes BlackBoard Interface - RobotinoAmpelInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012  Jens Wittmeyer
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

#ifndef __INTERFACES_ROBOTINOAMPELINTERFACE_H_
#define __INTERFACES_ROBOTINOAMPELINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class RobotinoAmpelInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(RobotinoAmpelInterface)
 /// @endcond
 public:
  /* constants */

  /** 
	This determines the current status of skill execution.
       */
  typedef enum {
    YELLOW /**<  true if ONLY yellow is activ. */,
    GREEN /**< true if ONLY green is active . */,
    RED /**< true if ONLY red is active . */,
    YELLOW_FLASHING /**< true if yellow is falshing. */,
    NO_CHANGE /**<  true if ampel didnt change   */
  } AmpelState;
  const char * tostring_AmpelState(AmpelState value) const;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t state; /**< 
 */
  } RobotinoAmpelInterface_data_t;

  RobotinoAmpelInterface_data_t *data;

  interface_enum_map_t enum_map_AmpelState;
 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  RobotinoAmpelInterface();
  ~RobotinoAmpelInterface();

 public:
  /* Methods */
  AmpelState state() const;
  void set_state(const AmpelState new_state);
  size_t maxlenof_state() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
