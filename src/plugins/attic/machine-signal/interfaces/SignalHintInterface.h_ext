
/***************************************************************************
 *  SignalHintInterface.h - Fawkes BlackBoard Interface - SignalHintInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Victor Mataré
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

#ifndef __INTERFACES_SIGNALHINTINTERFACE_H_
#define __INTERFACES_SIGNALHINTINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class SignalHintInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(SignalHintInterface)
 /// @endcond
 public:
  /* constants */

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct __attribute__((packed)) {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    double translation[3]; /**< 
			Last set position estimate, i.e. the one currently used (data fields
			are mandatory?).
		 */
  } SignalHintInterface_data_t;

  SignalHintInterface_data_t *data;

 public:
  /* messages */
  class SignalPositionMessage : public Message
  {
   private:
    /** Internal data storage, do NOT modify! */
    typedef struct __attribute__((packed)) {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      double translation[3]; /**< 
			Expected position of the signal, relative to left endpoint of the
			laser-line we see when standing in front of the MPS table
		 */
    } SignalPositionMessage_data_t;

    SignalPositionMessage_data_t *data;

   public:
    SignalPositionMessage(const double * ini_translation);
    SignalPositionMessage();
    ~SignalPositionMessage();

    SignalPositionMessage(const SignalPositionMessage *m);
    /* Methods */
    double * translation() const;
    double translation(unsigned int index) const;
    void set_translation(unsigned int index, const double new_translation);
    void set_translation(const double * new_translation);
    size_t maxlenof_translation() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  SignalHintInterface();
  ~SignalHintInterface();

 public:
  /* Methods */
  double * translation() const;
  double translation(unsigned int index) const;
  void set_translation(unsigned int index, const double new_translation);
  void set_translation(const double * new_translation);
  size_t maxlenof_translation() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
