
/***************************************************************************
 *  RobotinoLightInterface.h - Fawkes BlackBoard Interface - RobotinoLightInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Florian Nolden, Victor Mataré, Tobias Neumann
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

#ifndef __INTERFACES_ROBOTINOLIGHTINTERFACE_H_
#define __INTERFACES_ROBOTINOLIGHTINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class RobotinoLightInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(RobotinoLightInterface)
 /// @endcond
 public:
  /* constants */

  /** 
	This determines the current status of skill execution.
       */
  typedef enum {
    ON /**< the signal is on */,
    OFF /**< the signal is on */,
    BLINKING /**< the signal is blinking. */,
    UNKNOWN /**< the signal state is unknown */
  } LightState;
  const char * tostring_LightState(LightState value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t red; /**< State of red light */
    int32_t yellow; /**< State of yellow light */
    int32_t green; /**< State of green light */
    int32_t visibility_history; /**< visibility history */
    bool ready; /**< Data valid and ready */
  } RobotinoLightInterface_data_t;
#pragma pack(pop)

  RobotinoLightInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  RobotinoLightInterface();
  ~RobotinoLightInterface();

 public:
  /* Methods */
  LightState red() const;
  void set_red(const LightState new_red);
  size_t maxlenof_red() const;
  LightState yellow() const;
  void set_yellow(const LightState new_yellow);
  size_t maxlenof_yellow() const;
  LightState green() const;
  void set_green(const LightState new_green);
  size_t maxlenof_green() const;
  int32_t visibility_history() const;
  void set_visibility_history(const int32_t new_visibility_history);
  size_t maxlenof_visibility_history() const;
  bool is_ready() const;
  void set_ready(const bool new_ready);
  size_t maxlenof_ready() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
