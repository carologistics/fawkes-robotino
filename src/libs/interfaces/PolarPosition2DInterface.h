
/***************************************************************************
 *  PolarPosition2DInterface.h - Fawkes BlackBoard Interface - PolarPosition2DInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Daniel Ewert
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

#ifndef __INTERFACES_POLARPOSITION2DINTERFACE_H_
#define __INTERFACES_POLARPOSITION2DINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class PolarPosition2DInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(PolarPosition2DInterface)
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
    int32_t visibility_history; /**< 
      The visibilitiy history indicates the number of consecutive positive or negative
      sightings. If the history is negative, there have been as many negative sightings
      (object not visible) as the absolute value of the history. A positive value denotes
      as many positive sightings. 0 shall only be used during the initialization of the
      interface or if the visibility history is not updated.
     */
    int32_t angle; /**< 
     Angle between the x-axis of frame (e.g. the pov of the robot) and the center of 
     the detected object in degree
     */
    double distance; /**< 
     distance between origin of frame and center of the detected object
     */
  } PolarPosition2DInterface_data_t;
#pragma pack(pop)

  PolarPosition2DInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  PolarPosition2DInterface();
  ~PolarPosition2DInterface();

 public:
  /* Methods */
  char * frame() const;
  void set_frame(const char * new_frame);
  size_t maxlenof_frame() const;
  int32_t visibility_history() const;
  void set_visibility_history(const int32_t new_visibility_history);
  size_t maxlenof_visibility_history() const;
  int32_t angle() const;
  void set_angle(const int32_t new_angle);
  size_t maxlenof_angle() const;
  double distance() const;
  void set_distance(const double new_distance);
  size_t maxlenof_distance() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
