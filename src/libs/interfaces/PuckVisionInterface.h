
/***************************************************************************
 *  PuckVisionInterface.h - Fawkes BlackBoard Interface - PuckVisionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2013  Florian Nolden
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

#ifndef __INTERFACES_PUCKVISIONINTERFACE_H_
#define __INTERFACES_PUCKVISIONINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class PuckVisionInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(PuckVisionInterface)
 /// @endcond
 public:
  /* constants */

  /** 
	Possible puck colors
       */
  typedef enum {
    C_RED /**< red */,
    C_GREEN /**< green */,
    C_BLUE /**< blue */,
    C_BLACK /**< black */,
    C_YELLOW /**< yellow */,
    C_WHITE /**< white */,
    C_UNKNOWN /**< unknown */
  } PuckColor;
  const char * tostring_PuckColor(PuckColor value) const;

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char frame[32]; /**< 
	      Reference coordinate frame for the data.
    	 */
    int32_t puck1_color; /**< Detected Puck Color */
    int32_t puck2_color; /**< Detected Puck Color */
    int32_t puck3_color; /**< Detected Puck Color */
    int32_t puck1_visibility_history; /**< visibility history */
    int32_t puck2_visibility_history; /**< visibility history */
    int32_t puck3_visibility_history; /**< visibility history */
    double puck1_translation[3]; /**< Translation vector from the reference frame's origin, ordered as (x, y, z).
         */
    double puck2_translation[3]; /**< Translation vector from the reference frame's origin, ordered as (x, y, z).
         */
    double puck3_translation[3]; /**< Translation vector from the reference frame's origin, ordered as (x, y, z).
	 */
    double puck1_polar[2]; /**< Polar koordinates (Phi, R) = (x, y).
         */
    double puck2_polar[2]; /**< Polar koordinates (Phi, R) = (x, y).
         */
    double puck3_polar[2]; /**< Polar koordinates (Phi, R) = (x, y).
         */
  } PuckVisionInterface_data_t;
#pragma pack(pop)

  PuckVisionInterface_data_t *data;

  interface_enum_map_t enum_map_PuckColor;
 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  PuckVisionInterface();
  ~PuckVisionInterface();

 public:
  /* Methods */
  char * frame() const;
  void set_frame(const char * new_frame);
  size_t maxlenof_frame() const;
  PuckColor puck1_color() const;
  void set_puck1_color(const PuckColor new_puck1_color);
  size_t maxlenof_puck1_color() const;
  PuckColor puck2_color() const;
  void set_puck2_color(const PuckColor new_puck2_color);
  size_t maxlenof_puck2_color() const;
  PuckColor puck3_color() const;
  void set_puck3_color(const PuckColor new_puck3_color);
  size_t maxlenof_puck3_color() const;
  int32_t puck1_visibility_history() const;
  void set_puck1_visibility_history(const int32_t new_puck1_visibility_history);
  size_t maxlenof_puck1_visibility_history() const;
  int32_t puck2_visibility_history() const;
  void set_puck2_visibility_history(const int32_t new_puck2_visibility_history);
  size_t maxlenof_puck2_visibility_history() const;
  int32_t puck3_visibility_history() const;
  void set_puck3_visibility_history(const int32_t new_puck3_visibility_history);
  size_t maxlenof_puck3_visibility_history() const;
  double * puck1_translation() const;
  double puck1_translation(unsigned int index) const;
  void set_puck1_translation(unsigned int index, const double new_puck1_translation);
  void set_puck1_translation(const double * new_puck1_translation);
  size_t maxlenof_puck1_translation() const;
  double * puck2_translation() const;
  double puck2_translation(unsigned int index) const;
  void set_puck2_translation(unsigned int index, const double new_puck2_translation);
  void set_puck2_translation(const double * new_puck2_translation);
  size_t maxlenof_puck2_translation() const;
  double * puck3_translation() const;
  double puck3_translation(unsigned int index) const;
  void set_puck3_translation(unsigned int index, const double new_puck3_translation);
  void set_puck3_translation(const double * new_puck3_translation);
  size_t maxlenof_puck3_translation() const;
  double * puck1_polar() const;
  double puck1_polar(unsigned int index) const;
  void set_puck1_polar(unsigned int index, const double new_puck1_polar);
  void set_puck1_polar(const double * new_puck1_polar);
  size_t maxlenof_puck1_polar() const;
  double * puck2_polar() const;
  double puck2_polar(unsigned int index) const;
  void set_puck2_polar(unsigned int index, const double new_puck2_polar);
  void set_puck2_polar(const double * new_puck2_polar);
  size_t maxlenof_puck2_polar() const;
  double * puck3_polar() const;
  double puck3_polar(unsigned int index) const;
  void set_puck3_polar(unsigned int index, const double new_puck3_polar);
  void set_puck3_polar(const double * new_puck3_polar);
  size_t maxlenof_puck3_polar() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
