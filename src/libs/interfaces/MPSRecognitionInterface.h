
/***************************************************************************
 *  MPSRecognitionInterface.h - Fawkes BlackBoard Interface - MPSRecognitionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2016  David Schmidt
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

#ifndef __INTERFACES_MPSRECOGNITIONINTERFACE_H_
#define __INTERFACES_MPSRECOGNITIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class MPSRecognitionInterface : public Interface {

       	/// @cond INTERNALS
 
	INTERFACE_MGMT_FRIENDS(MPSRecognitionInterface)
 
	/// @endcond

       	public:
 
	       	/* constants */

  
		/** The different types of machines. */
  
		typedef enum {
    
			NoStationDetected /**< No station detected. */,
		       	BS /**< Base station. */,
    			CS /**< Cap station. */,
    			DS /**< Delivery station. */,
    			RS /**< Ring station. */,
    			SS /**< Storage Station. */
  			} MPSType;
  const char * tostring_MPSType(MPSType value) const;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t msgid; /**< 
      The ID of the message that is currently being processed or
      was processed last.
     */
    bool final; /**< 
      True, if the last recognition triggered by a ComputeMessage has
      been completed, false if it is still running. Also check the
      msgid field if this field applies to the correct message.
     */
    int32_t mpstype; 

  } MPSRecognitionInterface_data_t;

  MPSRecognitionInterface_data_t *data;

  interface_enum_map_t enum_map_MPSType;
 public:
  /* messages */
  class ClearMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ClearMessage_data_t;

    ClearMessage_data_t *data;

  interface_enum_map_t enum_map_MPSType;
   public:
    ClearMessage();
    ~ClearMessage();

    ClearMessage(const ClearMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };

  class ComputeMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    } ComputeMessage_data_t;

    ComputeMessage_data_t *data;

  interface_enum_map_t enum_map_MPSType;
   public:
    ComputeMessage();
    ~ComputeMessage();

    ComputeMessage(const ComputeMessage *m);
    /* Methods */
    virtual Message * clone() const;
  };




  virtual bool message_valid(const Message *message) const;
 private:
  MPSRecognitionInterface();
  ~MPSRecognitionInterface();

 public:
  /* Methods */
  uint32_t msgid() const;
  void set_msgid(const uint32_t new_msgid);
  size_t maxlenof_msgid() const;
  bool is_final() const;
  void set_final(const bool new_final);
  size_t maxlenof_final() const;
  MPSType mpstype() const;
  void set_mpstype(const MPSType new_mpstype);
  size_t maxlenof_mpstype() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
