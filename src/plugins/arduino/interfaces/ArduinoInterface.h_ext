
/***************************************************************************
 *  ArduinoInterface.h - Fawkes BlackBoard Interface - ArduinoInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Tim Niemueller, Nicolas Limpert
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

#ifndef __INTERFACES_ARDUINOINTERFACE_H_
#define __INTERFACES_ARDUINOINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class ArduinoInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(ArduinoInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t z_position; /**< Current z-position */
    bool final; /**< True, if the last move command has been finished */
  } ArduinoInterface_data_t;

  ArduinoInterface_data_t *data;

 public:
  /* messages */
  class MoveUpwardsMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t num_mm; /**< Number of mm to drive up */
    } MoveUpwardsMessage_data_t;

    MoveUpwardsMessage_data_t *data;

   public:
    MoveUpwardsMessage(const uint32_t ini_num_mm);
    MoveUpwardsMessage();
    ~MoveUpwardsMessage();

    MoveUpwardsMessage(const MoveUpwardsMessage *m);
    /* Methods */
    uint32_t num_mm() const;
    void set_num_mm(const uint32_t new_num_mm);
    size_t maxlenof_num_mm() const;
    virtual Message * clone() const;
  };

  class MoveDownwardsMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t num_mm; /**< Number of mm to drive down */
    } MoveDownwardsMessage_data_t;

    MoveDownwardsMessage_data_t *data;

   public:
    MoveDownwardsMessage(const uint32_t ini_num_mm);
    MoveDownwardsMessage();
    ~MoveDownwardsMessage();

    MoveDownwardsMessage(const MoveDownwardsMessage *m);
    /* Methods */
    uint32_t num_mm() const;
    void set_num_mm(const uint32_t new_num_mm);
    size_t maxlenof_num_mm() const;
    virtual Message * clone() const;
  };

  class MoveToZ0Message : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } MoveToZ0Message_data_t;

    MoveToZ0Message_data_t *data;

   public:
    MoveToZ0Message();
    ~MoveToZ0Message();

    MoveToZ0Message(const MoveToZ0Message *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  ArduinoInterface();
  ~ArduinoInterface();

 public:
  /* Methods */
  uint32_t z_position() const;
  void set_z_position(const uint32_t new_z_position);
  size_t maxlenof_z_position() const;
  bool is_final() const;
  void set_final(const bool new_final);
  size_t maxlenof_final() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
