
/***************************************************************************
 *  ConveyorConfigInterface.h - Fawkes BlackBoard Interface - ConveyorConfigInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  Tobias Neumann
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

#ifndef __INTERFACES_CONVEYORCONFIGINTERFACE_H_
#define __INTERFACES_CONVEYORCONFIGINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class ConveyorConfigInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(ConveyorConfigInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    bool product_removal; /**< 
      True if a product detection should be run and remove detected cycles.
      This should only be activated if there is a product to be expected, otherwise the performance decreases.
     */
  } ConveyorConfigInterface_data_t;
#pragma pack(pop)

  ConveyorConfigInterface_data_t *data;

 public:
  /* messages */
  class enable_product_removalMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } enable_product_removalMessage_data_t;
#pragma pack(pop)

    enable_product_removalMessage_data_t *data;

   public:
    enable_product_removalMessage();
    ~enable_product_removalMessage();

    enable_product_removalMessage(const enable_product_removalMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class disable_product_removalMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } disable_product_removalMessage_data_t;
#pragma pack(pop)

    disable_product_removalMessage_data_t *data;

   public:
    disable_product_removalMessage();
    ~disable_product_removalMessage();

    disable_product_removalMessage(const disable_product_removalMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  ConveyorConfigInterface();
  ~ConveyorConfigInterface();

 public:
  /* Methods */
  bool is_product_removal() const;
  void set_product_removal(const bool new_product_removal);
  size_t maxlenof_product_removal() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
