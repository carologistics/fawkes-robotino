
/***************************************************************************
 *  ShelfConfigurationInterface.h - Fawkes BlackBoard Interface - ShelfConfigurationInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2017  Till Hofmann
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

#ifndef __INTERFACES_SHELFCONFIGURATIONINTERFACE_H_
#define __INTERFACES_SHELFCONFIGURATIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class ShelfConfigurationInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(ShelfConfigurationInterface)
 /// @endcond
 public:
  /* constants */

  /** 
        Possible colors of a slot on the shelf.
       */
  typedef enum {
    RED /**< red */,
    BLACK /**< black */,
    EMPTY /**< empty */
  } SlotColor;
  const char * tostring_SlotColor(SlotColor value) const;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t left; /**< 
    The color of the left slot.
   */
    int32_t center; /**< 
    The color of the center slot.
   */
    int32_t right; /**< 
    The color of the right slot.
   */
    bool ready; /**< 
    Whether the shelf configuration is done.
   */
  } ShelfConfigurationInterface_data_t;

  ShelfConfigurationInterface_data_t *data;

  interface_enum_map_t enum_map_SlotColor;
 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  ShelfConfigurationInterface();
  ~ShelfConfigurationInterface();

 public:
  /* Methods */
  SlotColor left() const;
  void set_left(const SlotColor new_left);
  size_t maxlenof_left() const;
  SlotColor center() const;
  void set_center(const SlotColor new_center);
  size_t maxlenof_center() const;
  SlotColor right() const;
  void set_right(const SlotColor new_right);
  size_t maxlenof_right() const;
  bool is_ready() const;
  void set_ready(const bool new_ready);
  size_t maxlenof_ready() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
