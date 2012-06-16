
/***************************************************************************
 *  RobotinoWorldModelInterface.h - Fawkes BlackBoard Interface - RobotinoWorldModelInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2012  Daniel Ewert
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

#ifndef __INTERFACES_ROBOTINOWORLDMODELINTERFACE_H_
#define __INTERFACES_ROBOTINOWORLDMODELINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class RobotinoWorldModelInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(RobotinoWorldModelInterface)
 /// @endcond
 public:
  /* constants */

  /** 
				Enumeration for the type of a machine
			 */
  typedef enum {
    EXPRESS_MACHINE /**< Express Machine */,
    M1 /**< M1 */,
    M2 /**< M1 */,
    M3 /**< M1 */,
    M1_2 /**< M1 or M2 */,
    M2_3 /**< M2 or M3 */,
    UNKNOWN /**< Unknown machine type */,
    RECYCLING /**< Recycling machine */,
    TEST /**< Test machine */
  } machine_name_t;
  const char * tostring_machine_name_t(machine_name_t value) const;

  /** 
				States describe which pucks are currently located at the
				machine and the
				state of the machine itsself.
			 */
  typedef enum {
    S0_ONLY /**< S0 at machine */,
    S1_ONLY /**< S1 at machine */,
    S2_ONLY /**< S2 at machine */,
    P /**< P at machine */,
    S1_S2 /**< S1 and S2 at machine */
  } machine_state_t;
  const char * tostring_machine_state_t(machine_state_t value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    int32_t machine_names[13]; /**< Names
			of the machines on the field */
    int32_t machine_states[13]; /**< States
			of all machines on the field */
    char express_machine; /**< Name of the machine reserved for the
			express route */
  } RobotinoWorldModelInterface_data_t;
#pragma pack(pop)

  RobotinoWorldModelInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  RobotinoWorldModelInterface();
  ~RobotinoWorldModelInterface();

 public:
  /* Methods */
  machine_name_t * machine_names() const;
  machine_name_t machine_names(unsigned int index) const;
  void set_machine_names(unsigned int index, const machine_name_t new_machine_names);
  void set_machine_names(const machine_name_t * new_machine_names);
  size_t maxlenof_machine_names() const;
  machine_state_t * machine_states() const;
  machine_state_t machine_states(unsigned int index) const;
  void set_machine_states(unsigned int index, const machine_state_t new_machine_states);
  void set_machine_states(const machine_state_t * new_machine_states);
  size_t maxlenof_machine_states() const;
  char * express_machine() const;
  void set_express_machine(const char * new_express_machine);
  size_t maxlenof_express_machine() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
