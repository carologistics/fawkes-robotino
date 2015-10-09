
/***************************************************************************
 *  AX12GripperInterface.cpp - Fawkes BlackBoard Interface - AX12GripperInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2015  Tim Niemueller, Nicolas Limpert
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

#include <interfaces/AX12GripperInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class AX12GripperInterface <interfaces/AX12GripperInterface.h>
 * AX12GripperInterface Fawkes BlackBoard Interface.
 * 
      Interface to access left/right units.
    
 * @ingroup FawkesInterfaces
 */


/** FLAG_SUPPORTS_LEFT constant */
const uint32_t AX12GripperInterface::FLAG_SUPPORTS_LEFT = 1u;
/** FLAG_SUPPORTS_RIGHT constant */
const uint32_t AX12GripperInterface::FLAG_SUPPORTS_RIGHT = 2u;
/** ERROR_NONE constant */
const uint32_t AX12GripperInterface::ERROR_NONE = 0u;
/** ERROR_UNSPECIFIC constant */
const uint32_t AX12GripperInterface::ERROR_UNSPECIFIC = 1u;
/** ERROR_COMMUNICATION constant */
const uint32_t AX12GripperInterface::ERROR_COMMUNICATION = 2u;
/** ERROR_LEFT_OUTOFRANGE constant */
const uint32_t AX12GripperInterface::ERROR_LEFT_OUTOFRANGE = 4u;
/** ERROR_RIGHT_OUTOFRANGE constant */
const uint32_t AX12GripperInterface::ERROR_RIGHT_OUTOFRANGE = 8u;

/** Constructor */
AX12GripperInterface::AX12GripperInterface() : Interface()
{
  data_size = sizeof(AX12GripperInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (AX12GripperInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "flags", 1, &data->flags);
  add_fieldinfo(IFT_INT32, "z_position", 1, &data->z_position);
  add_fieldinfo(IFT_INT32, "z_upper_bound", 1, &data->z_upper_bound);
  add_fieldinfo(IFT_INT32, "z_lower_bound", 1, &data->z_lower_bound);
  add_fieldinfo(IFT_FLOAT, "left", 1, &data->left);
  add_fieldinfo(IFT_FLOAT, "right", 1, &data->right);
  add_fieldinfo(IFT_UINT32, "left_load", 1, &data->left_load);
  add_fieldinfo(IFT_UINT32, "right_load", 1, &data->right_load);
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
  add_fieldinfo(IFT_FLOAT, "offset", 1, &data->offset);
  add_fieldinfo(IFT_UINT32, "msgid", 1, &data->msgid);
  add_fieldinfo(IFT_BOOL, "final", 1, &data->final);
  add_fieldinfo(IFT_UINT32, "error_code", 1, &data->error_code);
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
  add_fieldinfo(IFT_BOOL, "calibrated", 1, &data->calibrated);
  add_fieldinfo(IFT_FLOAT, "min_left", 1, &data->min_left);
  add_fieldinfo(IFT_FLOAT, "max_left", 1, &data->max_left);
  add_fieldinfo(IFT_FLOAT, "min_right", 1, &data->min_right);
  add_fieldinfo(IFT_FLOAT, "max_right", 1, &data->max_right);
  add_fieldinfo(IFT_FLOAT, "max_left_velocity", 1, &data->max_left_velocity);
  add_fieldinfo(IFT_FLOAT, "max_right_velocity", 1, &data->max_right_velocity);
  add_fieldinfo(IFT_FLOAT, "left_velocity", 1, &data->left_velocity);
  add_fieldinfo(IFT_FLOAT, "right_velocity", 1, &data->right_velocity);
  add_fieldinfo(IFT_FLOAT, "left_margin", 1, &data->left_margin);
  add_fieldinfo(IFT_FLOAT, "right_margin", 1, &data->right_margin);
  add_fieldinfo(IFT_BOOL, "holds_puck", 1, &data->holds_puck);
  add_messageinfo("Open_AngleMessage");
  add_messageinfo("CloseLoadMessage");
  add_messageinfo("CenterMessage");
  add_messageinfo("CloseMessage");
  add_messageinfo("OpenMessage");
  add_messageinfo("RelGotoZMessage");
  add_messageinfo("StopLeftMessage");
  add_messageinfo("StopRightMessage");
  add_messageinfo("StopMessage");
  add_messageinfo("FlushMessage");
  add_messageinfo("CalibrateMessage");
  add_messageinfo("ParkMessage");
  add_messageinfo("GotoMessage");
  add_messageinfo("TimedGotoMessage");
  add_messageinfo("SetServoMessage");
  add_messageinfo("SetEnabledMessage");
  add_messageinfo("SetVelocityMessage");
  add_messageinfo("SetMarginMessage");
  unsigned char tmp_hash[] = {0xff, 0x83, 0x2a, 0x67, 0x10, 0x4a, 0x21, 0xb8, 0x88, 0xf, 0xa6, 0xa6, 0x17, 0x20, 0x8f, 0x56};
  set_hash(tmp_hash);
}

/** Destructor */
AX12GripperInterface::~AX12GripperInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get flags value.
 * Flags.
 * @return flags value
 */
uint32_t
AX12GripperInterface::flags() const
{
  return data->flags;
}

/** Get maximum length of flags value.
 * @return length of flags value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_flags() const
{
  return 1;
}

/** Set flags value.
 * Flags.
 * @param new_flags new flags value
 */
void
AX12GripperInterface::set_flags(const uint32_t new_flags)
{
  data->flags = new_flags;
  data_changed = true;
}

/** Get z_position value.
 * Z-Position.
 * @return z_position value
 */
int32_t
AX12GripperInterface::z_position() const
{
  return data->z_position;
}

/** Get maximum length of z_position value.
 * @return length of z_position value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_z_position() const
{
  return 1;
}

/** Set z_position value.
 * Z-Position.
 * @param new_z_position new z_position value
 */
void
AX12GripperInterface::set_z_position(const int32_t new_z_position)
{
  data->z_position = new_z_position;
  data_changed = true;
}

/** Get z_upper_bound value.
 * Z-upper bound.
 * @return z_upper_bound value
 */
int32_t
AX12GripperInterface::z_upper_bound() const
{
  return data->z_upper_bound;
}

/** Get maximum length of z_upper_bound value.
 * @return length of z_upper_bound value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_z_upper_bound() const
{
  return 1;
}

/** Set z_upper_bound value.
 * Z-upper bound.
 * @param new_z_upper_bound new z_upper_bound value
 */
void
AX12GripperInterface::set_z_upper_bound(const int32_t new_z_upper_bound)
{
  data->z_upper_bound = new_z_upper_bound;
  data_changed = true;
}

/** Get z_lower_bound value.
 * Z-lower bound.
 * @return z_lower_bound value
 */
int32_t
AX12GripperInterface::z_lower_bound() const
{
  return data->z_lower_bound;
}

/** Get maximum length of z_lower_bound value.
 * @return length of z_lower_bound value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_z_lower_bound() const
{
  return 1;
}

/** Set z_lower_bound value.
 * Z-lower bound.
 * @param new_z_lower_bound new z_lower_bound value
 */
void
AX12GripperInterface::set_z_lower_bound(const int32_t new_z_lower_bound)
{
  data->z_lower_bound = new_z_lower_bound;
  data_changed = true;
}

/** Get left value.
 * Current left.
 * @return left value
 */
float
AX12GripperInterface::left() const
{
  return data->left;
}

/** Get maximum length of left value.
 * @return length of left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_left() const
{
  return 1;
}

/** Set left value.
 * Current left.
 * @param new_left new left value
 */
void
AX12GripperInterface::set_left(const float new_left)
{
  data->left = new_left;
  data_changed = true;
}

/** Get right value.
 * Current right.
 * @return right value
 */
float
AX12GripperInterface::right() const
{
  return data->right;
}

/** Get maximum length of right value.
 * @return length of right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_right() const
{
  return 1;
}

/** Set right value.
 * Current right.
 * @param new_right new right value
 */
void
AX12GripperInterface::set_right(const float new_right)
{
  data->right = new_right;
  data_changed = true;
}

/** Get left_load value.
 * Current left load.
 * @return left_load value
 */
uint32_t
AX12GripperInterface::left_load() const
{
  return data->left_load;
}

/** Get maximum length of left_load value.
 * @return length of left_load value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_left_load() const
{
  return 1;
}

/** Set left_load value.
 * Current left load.
 * @param new_left_load new left_load value
 */
void
AX12GripperInterface::set_left_load(const uint32_t new_left_load)
{
  data->left_load = new_left_load;
  data_changed = true;
}

/** Get right_load value.
 * Current right load.
 * @return right_load value
 */
uint32_t
AX12GripperInterface::right_load() const
{
  return data->right_load;
}

/** Get maximum length of right_load value.
 * @return length of right_load value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_right_load() const
{
  return 1;
}

/** Set right_load value.
 * Current right load.
 * @param new_right_load new right_load value
 */
void
AX12GripperInterface::set_right_load(const uint32_t new_right_load)
{
  data->right_load = new_right_load;
  data_changed = true;
}

/** Get angle value.
 * Current angle.
 * @return angle value
 */
float
AX12GripperInterface::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Current angle.
 * @param new_angle new angle value
 */
void
AX12GripperInterface::set_angle(const float new_angle)
{
  data->angle = new_angle;
  data_changed = true;
}

/** Get offset value.
 * Current offset.
 * @return offset value
 */
float
AX12GripperInterface::offset() const
{
  return data->offset;
}

/** Get maximum length of offset value.
 * @return length of offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_offset() const
{
  return 1;
}

/** Set offset value.
 * Current offset.
 * @param new_offset new offset value
 */
void
AX12GripperInterface::set_offset(const float new_offset)
{
  data->offset = new_offset;
  data_changed = true;
}

/** Get msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @return msgid value
 */
uint32_t
AX12GripperInterface::msgid() const
{
  return data->msgid;
}

/** Get maximum length of msgid value.
 * @return length of msgid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_msgid() const
{
  return 1;
}

/** Set msgid value.
 * The ID of the message that is currently being
      processed, or 0 if no message is being processed.
 * @param new_msgid new msgid value
 */
void
AX12GripperInterface::set_msgid(const uint32_t new_msgid)
{
  data->msgid = new_msgid;
  data_changed = true;
}

/** Get final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @return final value
 */
bool
AX12GripperInterface::is_final() const
{
  return data->final;
}

/** Get maximum length of final value.
 * @return length of final value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_final() const
{
  return 1;
}

/** Set final value.
 * True, if the last goto command has been finished,
      false if it is still running
 * @param new_final new final value
 */
void
AX12GripperInterface::set_final(const bool new_final)
{
  data->final = new_final;
  data_changed = true;
}

/** Get error_code value.
 * Failure code set if
    final is true. 0 if no error occured, an error code from ERROR_*
    constants otherwise (or a bit-wise combination).
 * @return error_code value
 */
uint32_t
AX12GripperInterface::error_code() const
{
  return data->error_code;
}

/** Get maximum length of error_code value.
 * @return length of error_code value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_error_code() const
{
  return 1;
}

/** Set error_code value.
 * Failure code set if
    final is true. 0 if no error occured, an error code from ERROR_*
    constants otherwise (or a bit-wise combination).
 * @param new_error_code new error_code value
 */
void
AX12GripperInterface::set_error_code(const uint32_t new_error_code)
{
  data->error_code = new_error_code;
  data_changed = true;
}

/** Get enabled value.
 * Is the left/right unit enabled?
 * @return enabled value
 */
bool
AX12GripperInterface::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * Is the left/right unit enabled?
 * @param new_enabled new enabled value
 */
void
AX12GripperInterface::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
  data_changed = true;
}

/** Get calibrated value.
 * Is the left/right unit calibrated?
 * @return calibrated value
 */
bool
AX12GripperInterface::is_calibrated() const
{
  return data->calibrated;
}

/** Get maximum length of calibrated value.
 * @return length of calibrated value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_calibrated() const
{
  return 1;
}

/** Set calibrated value.
 * Is the left/right unit calibrated?
 * @param new_calibrated new calibrated value
 */
void
AX12GripperInterface::set_calibrated(const bool new_calibrated)
{
  data->calibrated = new_calibrated;
  data_changed = true;
}

/** Get min_left value.
 * Minimum left possible.
 * @return min_left value
 */
float
AX12GripperInterface::min_left() const
{
  return data->min_left;
}

/** Get maximum length of min_left value.
 * @return length of min_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_min_left() const
{
  return 1;
}

/** Set min_left value.
 * Minimum left possible.
 * @param new_min_left new min_left value
 */
void
AX12GripperInterface::set_min_left(const float new_min_left)
{
  data->min_left = new_min_left;
  data_changed = true;
}

/** Get max_left value.
 * Maximum left possible.
 * @return max_left value
 */
float
AX12GripperInterface::max_left() const
{
  return data->max_left;
}

/** Get maximum length of max_left value.
 * @return length of max_left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_max_left() const
{
  return 1;
}

/** Set max_left value.
 * Maximum left possible.
 * @param new_max_left new max_left value
 */
void
AX12GripperInterface::set_max_left(const float new_max_left)
{
  data->max_left = new_max_left;
  data_changed = true;
}

/** Get min_right value.
 * Minimum right possible.
 * @return min_right value
 */
float
AX12GripperInterface::min_right() const
{
  return data->min_right;
}

/** Get maximum length of min_right value.
 * @return length of min_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_min_right() const
{
  return 1;
}

/** Set min_right value.
 * Minimum right possible.
 * @param new_min_right new min_right value
 */
void
AX12GripperInterface::set_min_right(const float new_min_right)
{
  data->min_right = new_min_right;
  data_changed = true;
}

/** Get max_right value.
 * Maximum right possible.
 * @return max_right value
 */
float
AX12GripperInterface::max_right() const
{
  return data->max_right;
}

/** Get maximum length of max_right value.
 * @return length of max_right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_max_right() const
{
  return 1;
}

/** Set max_right value.
 * Maximum right possible.
 * @param new_max_right new max_right value
 */
void
AX12GripperInterface::set_max_right(const float new_max_right)
{
  data->max_right = new_max_right;
  data_changed = true;
}

/** Get max_left_velocity value.
 * Maximum supported left velocity.
 * @return max_left_velocity value
 */
float
AX12GripperInterface::max_left_velocity() const
{
  return data->max_left_velocity;
}

/** Get maximum length of max_left_velocity value.
 * @return length of max_left_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_max_left_velocity() const
{
  return 1;
}

/** Set max_left_velocity value.
 * Maximum supported left velocity.
 * @param new_max_left_velocity new max_left_velocity value
 */
void
AX12GripperInterface::set_max_left_velocity(const float new_max_left_velocity)
{
  data->max_left_velocity = new_max_left_velocity;
  data_changed = true;
}

/** Get max_right_velocity value.
 * Maximum supported right velocity.
 * @return max_right_velocity value
 */
float
AX12GripperInterface::max_right_velocity() const
{
  return data->max_right_velocity;
}

/** Get maximum length of max_right_velocity value.
 * @return length of max_right_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_max_right_velocity() const
{
  return 1;
}

/** Set max_right_velocity value.
 * Maximum supported right velocity.
 * @param new_max_right_velocity new max_right_velocity value
 */
void
AX12GripperInterface::set_max_right_velocity(const float new_max_right_velocity)
{
  data->max_right_velocity = new_max_right_velocity;
  data_changed = true;
}

/** Get left_velocity value.
 * Maximum left velocity currently reached.
 * @return left_velocity value
 */
float
AX12GripperInterface::left_velocity() const
{
  return data->left_velocity;
}

/** Get maximum length of left_velocity value.
 * @return length of left_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_left_velocity() const
{
  return 1;
}

/** Set left_velocity value.
 * Maximum left velocity currently reached.
 * @param new_left_velocity new left_velocity value
 */
void
AX12GripperInterface::set_left_velocity(const float new_left_velocity)
{
  data->left_velocity = new_left_velocity;
  data_changed = true;
}

/** Get right_velocity value.
 * Maximum right velocity currently reached.
 * @return right_velocity value
 */
float
AX12GripperInterface::right_velocity() const
{
  return data->right_velocity;
}

/** Get maximum length of right_velocity value.
 * @return length of right_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_right_velocity() const
{
  return 1;
}

/** Set right_velocity value.
 * Maximum right velocity currently reached.
 * @param new_right_velocity new right_velocity value
 */
void
AX12GripperInterface::set_right_velocity(const float new_right_velocity)
{
  data->right_velocity = new_right_velocity;
  data_changed = true;
}

/** Get left_margin value.
 * Margin in radians around a
    target left value to consider the motion as final.
 * @return left_margin value
 */
float
AX12GripperInterface::left_margin() const
{
  return data->left_margin;
}

/** Get maximum length of left_margin value.
 * @return length of left_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_left_margin() const
{
  return 1;
}

/** Set left_margin value.
 * Margin in radians around a
    target left value to consider the motion as final.
 * @param new_left_margin new left_margin value
 */
void
AX12GripperInterface::set_left_margin(const float new_left_margin)
{
  data->left_margin = new_left_margin;
  data_changed = true;
}

/** Get right_margin value.
 * Margin in radians around a
    target right value to consider the motion as final.
 * @return right_margin value
 */
float
AX12GripperInterface::right_margin() const
{
  return data->right_margin;
}

/** Get maximum length of right_margin value.
 * @return length of right_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_right_margin() const
{
  return 1;
}

/** Set right_margin value.
 * Margin in radians around a
    target right value to consider the motion as final.
 * @param new_right_margin new right_margin value
 */
void
AX12GripperInterface::set_right_margin(const float new_right_margin)
{
  data->right_margin = new_right_margin;
  data_changed = true;
}

/** Get holds_puck value.
 * True if the gripper holds a puck
 * @return holds_puck value
 */
bool
AX12GripperInterface::is_holds_puck() const
{
  return data->holds_puck;
}

/** Get maximum length of holds_puck value.
 * @return length of holds_puck value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::maxlenof_holds_puck() const
{
  return 1;
}

/** Set holds_puck value.
 * True if the gripper holds a puck
 * @param new_holds_puck new holds_puck value
 */
void
AX12GripperInterface::set_holds_puck(const bool new_holds_puck)
{
  data->holds_puck = new_holds_puck;
  data_changed = true;
}

/* =========== message create =========== */
Message *
AX12GripperInterface::create_message(const char *type) const
{
  if ( strncmp("Open_AngleMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new Open_AngleMessage();
  } else if ( strncmp("CloseLoadMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CloseLoadMessage();
  } else if ( strncmp("CenterMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CenterMessage();
  } else if ( strncmp("CloseMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CloseMessage();
  } else if ( strncmp("OpenMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new OpenMessage();
  } else if ( strncmp("RelGotoZMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new RelGotoZMessage();
  } else if ( strncmp("StopLeftMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopLeftMessage();
  } else if ( strncmp("StopRightMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopRightMessage();
  } else if ( strncmp("StopMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new StopMessage();
  } else if ( strncmp("FlushMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new FlushMessage();
  } else if ( strncmp("CalibrateMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new CalibrateMessage();
  } else if ( strncmp("ParkMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new ParkMessage();
  } else if ( strncmp("GotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new GotoMessage();
  } else if ( strncmp("TimedGotoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new TimedGotoMessage();
  } else if ( strncmp("SetServoMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetServoMessage();
  } else if ( strncmp("SetEnabledMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetEnabledMessage();
  } else if ( strncmp("SetVelocityMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetVelocityMessage();
  } else if ( strncmp("SetMarginMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetMarginMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
AX12GripperInterface::copy_values(const Interface *other)
{
  const AX12GripperInterface *oi = dynamic_cast<const AX12GripperInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(AX12GripperInterface_data_t));
}

const char *
AX12GripperInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class AX12GripperInterface::Open_AngleMessage <interfaces/AX12GripperInterface.h>
 * Open_AngleMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_angle initial value for angle
 */
AX12GripperInterface::Open_AngleMessage::Open_AngleMessage(const float ini_angle) : Message("Open_AngleMessage")
{
  data_size = sizeof(Open_AngleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (Open_AngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->angle = ini_angle;
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}
/** Constructor */
AX12GripperInterface::Open_AngleMessage::Open_AngleMessage() : Message("Open_AngleMessage")
{
  data_size = sizeof(Open_AngleMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (Open_AngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}

/** Destructor */
AX12GripperInterface::Open_AngleMessage::~Open_AngleMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::Open_AngleMessage::Open_AngleMessage(const Open_AngleMessage *m) : Message("Open_AngleMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (Open_AngleMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get angle value.
 * Current angle.
 * @return angle value
 */
float
AX12GripperInterface::Open_AngleMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::Open_AngleMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Current angle.
 * @param new_angle new angle value
 */
void
AX12GripperInterface::Open_AngleMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::Open_AngleMessage::clone() const
{
  return new AX12GripperInterface::Open_AngleMessage(this);
}
/** @class AX12GripperInterface::CloseLoadMessage <interfaces/AX12GripperInterface.h>
 * CloseLoadMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_offset initial value for offset
 */
AX12GripperInterface::CloseLoadMessage::CloseLoadMessage(const float ini_offset) : Message("CloseLoadMessage")
{
  data_size = sizeof(CloseLoadMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CloseLoadMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->offset = ini_offset;
  add_fieldinfo(IFT_FLOAT, "offset", 1, &data->offset);
}
/** Constructor */
AX12GripperInterface::CloseLoadMessage::CloseLoadMessage() : Message("CloseLoadMessage")
{
  data_size = sizeof(CloseLoadMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CloseLoadMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "offset", 1, &data->offset);
}

/** Destructor */
AX12GripperInterface::CloseLoadMessage::~CloseLoadMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::CloseLoadMessage::CloseLoadMessage(const CloseLoadMessage *m) : Message("CloseLoadMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CloseLoadMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get offset value.
 * Current offset.
 * @return offset value
 */
float
AX12GripperInterface::CloseLoadMessage::offset() const
{
  return data->offset;
}

/** Get maximum length of offset value.
 * @return length of offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::CloseLoadMessage::maxlenof_offset() const
{
  return 1;
}

/** Set offset value.
 * Current offset.
 * @param new_offset new offset value
 */
void
AX12GripperInterface::CloseLoadMessage::set_offset(const float new_offset)
{
  data->offset = new_offset;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::CloseLoadMessage::clone() const
{
  return new AX12GripperInterface::CloseLoadMessage(this);
}
/** @class AX12GripperInterface::CenterMessage <interfaces/AX12GripperInterface.h>
 * CenterMessage Fawkes BlackBoard Interface Message.
 * 
   
 */


/** Constructor */
AX12GripperInterface::CenterMessage::CenterMessage() : Message("CenterMessage")
{
  data_size = sizeof(CenterMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CenterMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
AX12GripperInterface::CenterMessage::~CenterMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::CenterMessage::CenterMessage(const CenterMessage *m) : Message("CenterMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CenterMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::CenterMessage::clone() const
{
  return new AX12GripperInterface::CenterMessage(this);
}
/** @class AX12GripperInterface::CloseMessage <interfaces/AX12GripperInterface.h>
 * CloseMessage Fawkes BlackBoard Interface Message.
 * 
   
 */


/** Constructor with initial values.
 * @param ini_offset initial value for offset
 */
AX12GripperInterface::CloseMessage::CloseMessage(const float ini_offset) : Message("CloseMessage")
{
  data_size = sizeof(CloseMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CloseMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->offset = ini_offset;
  add_fieldinfo(IFT_FLOAT, "offset", 1, &data->offset);
}
/** Constructor */
AX12GripperInterface::CloseMessage::CloseMessage() : Message("CloseMessage")
{
  data_size = sizeof(CloseMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CloseMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "offset", 1, &data->offset);
}

/** Destructor */
AX12GripperInterface::CloseMessage::~CloseMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::CloseMessage::CloseMessage(const CloseMessage *m) : Message("CloseMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CloseMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get offset value.
 * Current offset.
 * @return offset value
 */
float
AX12GripperInterface::CloseMessage::offset() const
{
  return data->offset;
}

/** Get maximum length of offset value.
 * @return length of offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::CloseMessage::maxlenof_offset() const
{
  return 1;
}

/** Set offset value.
 * Current offset.
 * @param new_offset new offset value
 */
void
AX12GripperInterface::CloseMessage::set_offset(const float new_offset)
{
  data->offset = new_offset;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::CloseMessage::clone() const
{
  return new AX12GripperInterface::CloseMessage(this);
}
/** @class AX12GripperInterface::OpenMessage <interfaces/AX12GripperInterface.h>
 * OpenMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_offset initial value for offset
 */
AX12GripperInterface::OpenMessage::OpenMessage(const float ini_offset) : Message("OpenMessage")
{
  data_size = sizeof(OpenMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OpenMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->offset = ini_offset;
  add_fieldinfo(IFT_FLOAT, "offset", 1, &data->offset);
}
/** Constructor */
AX12GripperInterface::OpenMessage::OpenMessage() : Message("OpenMessage")
{
  data_size = sizeof(OpenMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (OpenMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "offset", 1, &data->offset);
}

/** Destructor */
AX12GripperInterface::OpenMessage::~OpenMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::OpenMessage::OpenMessage(const OpenMessage *m) : Message("OpenMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (OpenMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get offset value.
 * Current offset.
 * @return offset value
 */
float
AX12GripperInterface::OpenMessage::offset() const
{
  return data->offset;
}

/** Get maximum length of offset value.
 * @return length of offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::OpenMessage::maxlenof_offset() const
{
  return 1;
}

/** Set offset value.
 * Current offset.
 * @param new_offset new offset value
 */
void
AX12GripperInterface::OpenMessage::set_offset(const float new_offset)
{
  data->offset = new_offset;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::OpenMessage::clone() const
{
  return new AX12GripperInterface::OpenMessage(this);
}
/** @class AX12GripperInterface::RelGotoZMessage <interfaces/AX12GripperInterface.h>
 * RelGotoZMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_rel_z initial value for rel_z
 */
AX12GripperInterface::RelGotoZMessage::RelGotoZMessage(const int32_t ini_rel_z) : Message("RelGotoZMessage")
{
  data_size = sizeof(RelGotoZMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RelGotoZMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->rel_z = ini_rel_z;
  add_fieldinfo(IFT_INT32, "rel_z", 1, &data->rel_z);
}
/** Constructor */
AX12GripperInterface::RelGotoZMessage::RelGotoZMessage() : Message("RelGotoZMessage")
{
  data_size = sizeof(RelGotoZMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (RelGotoZMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_INT32, "rel_z", 1, &data->rel_z);
}

/** Destructor */
AX12GripperInterface::RelGotoZMessage::~RelGotoZMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::RelGotoZMessage::RelGotoZMessage(const RelGotoZMessage *m) : Message("RelGotoZMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (RelGotoZMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get rel_z value.
 * New z relative to the current z, in mm.
 * @return rel_z value
 */
int32_t
AX12GripperInterface::RelGotoZMessage::rel_z() const
{
  return data->rel_z;
}

/** Get maximum length of rel_z value.
 * @return length of rel_z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::RelGotoZMessage::maxlenof_rel_z() const
{
  return 1;
}

/** Set rel_z value.
 * New z relative to the current z, in mm.
 * @param new_rel_z new rel_z value
 */
void
AX12GripperInterface::RelGotoZMessage::set_rel_z(const int32_t new_rel_z)
{
  data->rel_z = new_rel_z;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::RelGotoZMessage::clone() const
{
  return new AX12GripperInterface::RelGotoZMessage(this);
}
/** @class AX12GripperInterface::StopLeftMessage <interfaces/AX12GripperInterface.h>
 * StopLeftMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
AX12GripperInterface::StopLeftMessage::StopLeftMessage() : Message("StopLeftMessage")
{
  data_size = sizeof(StopLeftMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopLeftMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
AX12GripperInterface::StopLeftMessage::~StopLeftMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::StopLeftMessage::StopLeftMessage(const StopLeftMessage *m) : Message("StopLeftMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopLeftMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::StopLeftMessage::clone() const
{
  return new AX12GripperInterface::StopLeftMessage(this);
}
/** @class AX12GripperInterface::StopRightMessage <interfaces/AX12GripperInterface.h>
 * StopRightMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
AX12GripperInterface::StopRightMessage::StopRightMessage() : Message("StopRightMessage")
{
  data_size = sizeof(StopRightMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopRightMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
AX12GripperInterface::StopRightMessage::~StopRightMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::StopRightMessage::StopRightMessage(const StopRightMessage *m) : Message("StopRightMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopRightMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::StopRightMessage::clone() const
{
  return new AX12GripperInterface::StopRightMessage(this);
}
/** @class AX12GripperInterface::StopMessage <interfaces/AX12GripperInterface.h>
 * StopMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
AX12GripperInterface::StopMessage::StopMessage() : Message("StopMessage")
{
  data_size = sizeof(StopMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
AX12GripperInterface::StopMessage::~StopMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::StopMessage::StopMessage(const StopMessage *m) : Message("StopMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (StopMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::StopMessage::clone() const
{
  return new AX12GripperInterface::StopMessage(this);
}
/** @class AX12GripperInterface::FlushMessage <interfaces/AX12GripperInterface.h>
 * FlushMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
AX12GripperInterface::FlushMessage::FlushMessage() : Message("FlushMessage")
{
  data_size = sizeof(FlushMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (FlushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
AX12GripperInterface::FlushMessage::~FlushMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::FlushMessage::FlushMessage(const FlushMessage *m) : Message("FlushMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (FlushMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::FlushMessage::clone() const
{
  return new AX12GripperInterface::FlushMessage(this);
}
/** @class AX12GripperInterface::CalibrateMessage <interfaces/AX12GripperInterface.h>
 * CalibrateMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
AX12GripperInterface::CalibrateMessage::CalibrateMessage() : Message("CalibrateMessage")
{
  data_size = sizeof(CalibrateMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (CalibrateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
AX12GripperInterface::CalibrateMessage::~CalibrateMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::CalibrateMessage::CalibrateMessage(const CalibrateMessage *m) : Message("CalibrateMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (CalibrateMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::CalibrateMessage::clone() const
{
  return new AX12GripperInterface::CalibrateMessage(this);
}
/** @class AX12GripperInterface::ParkMessage <interfaces/AX12GripperInterface.h>
 * ParkMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor */
AX12GripperInterface::ParkMessage::ParkMessage() : Message("ParkMessage")
{
  data_size = sizeof(ParkMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/** Destructor */
AX12GripperInterface::ParkMessage::~ParkMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::ParkMessage::ParkMessage(const ParkMessage *m) : Message("ParkMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (ParkMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::ParkMessage::clone() const
{
  return new AX12GripperInterface::ParkMessage(this);
}
/** @class AX12GripperInterface::GotoMessage <interfaces/AX12GripperInterface.h>
 * GotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_left initial value for left
 * @param ini_right initial value for right
 */
AX12GripperInterface::GotoMessage::GotoMessage(const float ini_left, const float ini_right) : Message("GotoMessage")
{
  data_size = sizeof(GotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->left = ini_left;
  data->right = ini_right;
  add_fieldinfo(IFT_FLOAT, "left", 1, &data->left);
  add_fieldinfo(IFT_FLOAT, "right", 1, &data->right);
}
/** Constructor */
AX12GripperInterface::GotoMessage::GotoMessage() : Message("GotoMessage")
{
  data_size = sizeof(GotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "left", 1, &data->left);
  add_fieldinfo(IFT_FLOAT, "right", 1, &data->right);
}

/** Destructor */
AX12GripperInterface::GotoMessage::~GotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::GotoMessage::GotoMessage(const GotoMessage *m) : Message("GotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (GotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get left value.
 * Current left.
 * @return left value
 */
float
AX12GripperInterface::GotoMessage::left() const
{
  return data->left;
}

/** Get maximum length of left value.
 * @return length of left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::GotoMessage::maxlenof_left() const
{
  return 1;
}

/** Set left value.
 * Current left.
 * @param new_left new left value
 */
void
AX12GripperInterface::GotoMessage::set_left(const float new_left)
{
  data->left = new_left;
}

/** Get right value.
 * Current right.
 * @return right value
 */
float
AX12GripperInterface::GotoMessage::right() const
{
  return data->right;
}

/** Get maximum length of right value.
 * @return length of right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::GotoMessage::maxlenof_right() const
{
  return 1;
}

/** Set right value.
 * Current right.
 * @param new_right new right value
 */
void
AX12GripperInterface::GotoMessage::set_right(const float new_right)
{
  data->right = new_right;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::GotoMessage::clone() const
{
  return new AX12GripperInterface::GotoMessage(this);
}
/** @class AX12GripperInterface::TimedGotoMessage <interfaces/AX12GripperInterface.h>
 * TimedGotoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_time_sec initial value for time_sec
 * @param ini_left initial value for left
 * @param ini_right initial value for right
 */
AX12GripperInterface::TimedGotoMessage::TimedGotoMessage(const float ini_time_sec, const float ini_left, const float ini_right) : Message("TimedGotoMessage")
{
  data_size = sizeof(TimedGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->time_sec = ini_time_sec;
  data->left = ini_left;
  data->right = ini_right;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "left", 1, &data->left);
  add_fieldinfo(IFT_FLOAT, "right", 1, &data->right);
}
/** Constructor */
AX12GripperInterface::TimedGotoMessage::TimedGotoMessage() : Message("TimedGotoMessage")
{
  data_size = sizeof(TimedGotoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "time_sec", 1, &data->time_sec);
  add_fieldinfo(IFT_FLOAT, "left", 1, &data->left);
  add_fieldinfo(IFT_FLOAT, "right", 1, &data->right);
}

/** Destructor */
AX12GripperInterface::TimedGotoMessage::~TimedGotoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::TimedGotoMessage::TimedGotoMessage(const TimedGotoMessage *m) : Message("TimedGotoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (TimedGotoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get time_sec value.
 * Time in seconds when to reach
    the final position.
 * @return time_sec value
 */
float
AX12GripperInterface::TimedGotoMessage::time_sec() const
{
  return data->time_sec;
}

/** Get maximum length of time_sec value.
 * @return length of time_sec value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::TimedGotoMessage::maxlenof_time_sec() const
{
  return 1;
}

/** Set time_sec value.
 * Time in seconds when to reach
    the final position.
 * @param new_time_sec new time_sec value
 */
void
AX12GripperInterface::TimedGotoMessage::set_time_sec(const float new_time_sec)
{
  data->time_sec = new_time_sec;
}

/** Get left value.
 * Current left.
 * @return left value
 */
float
AX12GripperInterface::TimedGotoMessage::left() const
{
  return data->left;
}

/** Get maximum length of left value.
 * @return length of left value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::TimedGotoMessage::maxlenof_left() const
{
  return 1;
}

/** Set left value.
 * Current left.
 * @param new_left new left value
 */
void
AX12GripperInterface::TimedGotoMessage::set_left(const float new_left)
{
  data->left = new_left;
}

/** Get right value.
 * Current right.
 * @return right value
 */
float
AX12GripperInterface::TimedGotoMessage::right() const
{
  return data->right;
}

/** Get maximum length of right value.
 * @return length of right value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::TimedGotoMessage::maxlenof_right() const
{
  return 1;
}

/** Set right value.
 * Current right.
 * @param new_right new right value
 */
void
AX12GripperInterface::TimedGotoMessage::set_right(const float new_right)
{
  data->right = new_right;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::TimedGotoMessage::clone() const
{
  return new AX12GripperInterface::TimedGotoMessage(this);
}
/** @class AX12GripperInterface::SetServoMessage <interfaces/AX12GripperInterface.h>
 * SetServoMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_servoID initial value for servoID
 * @param ini_angle initial value for angle
 */
AX12GripperInterface::SetServoMessage::SetServoMessage(const uint32_t ini_servoID, const float ini_angle) : Message("SetServoMessage")
{
  data_size = sizeof(SetServoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->servoID = ini_servoID;
  data->angle = ini_angle;
  add_fieldinfo(IFT_UINT32, "servoID", 1, &data->servoID);
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}
/** Constructor */
AX12GripperInterface::SetServoMessage::SetServoMessage() : Message("SetServoMessage")
{
  data_size = sizeof(SetServoMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "servoID", 1, &data->servoID);
  add_fieldinfo(IFT_FLOAT, "angle", 1, &data->angle);
}

/** Destructor */
AX12GripperInterface::SetServoMessage::~SetServoMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::SetServoMessage::SetServoMessage(const SetServoMessage *m) : Message("SetServoMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetServoMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get servoID value.
 * Servo ID.
 * @return servoID value
 */
uint32_t
AX12GripperInterface::SetServoMessage::servoID() const
{
  return data->servoID;
}

/** Get maximum length of servoID value.
 * @return length of servoID value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::SetServoMessage::maxlenof_servoID() const
{
  return 1;
}

/** Set servoID value.
 * Servo ID.
 * @param new_servoID new servoID value
 */
void
AX12GripperInterface::SetServoMessage::set_servoID(const uint32_t new_servoID)
{
  data->servoID = new_servoID;
}

/** Get angle value.
 * Target Servo angle.
 * @return angle value
 */
float
AX12GripperInterface::SetServoMessage::angle() const
{
  return data->angle;
}

/** Get maximum length of angle value.
 * @return length of angle value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::SetServoMessage::maxlenof_angle() const
{
  return 1;
}

/** Set angle value.
 * Target Servo angle.
 * @param new_angle new angle value
 */
void
AX12GripperInterface::SetServoMessage::set_angle(const float new_angle)
{
  data->angle = new_angle;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::SetServoMessage::clone() const
{
  return new AX12GripperInterface::SetServoMessage(this);
}
/** @class AX12GripperInterface::SetEnabledMessage <interfaces/AX12GripperInterface.h>
 * SetEnabledMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_enabled initial value for enabled
 */
AX12GripperInterface::SetEnabledMessage::SetEnabledMessage(const bool ini_enabled) : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->enabled = ini_enabled;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}
/** Constructor */
AX12GripperInterface::SetEnabledMessage::SetEnabledMessage() : Message("SetEnabledMessage")
{
  data_size = sizeof(SetEnabledMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_BOOL, "enabled", 1, &data->enabled);
}

/** Destructor */
AX12GripperInterface::SetEnabledMessage::~SetEnabledMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::SetEnabledMessage::SetEnabledMessage(const SetEnabledMessage *m) : Message("SetEnabledMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetEnabledMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get enabled value.
 * Is the left/right unit enabled?
 * @return enabled value
 */
bool
AX12GripperInterface::SetEnabledMessage::is_enabled() const
{
  return data->enabled;
}

/** Get maximum length of enabled value.
 * @return length of enabled value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::SetEnabledMessage::maxlenof_enabled() const
{
  return 1;
}

/** Set enabled value.
 * Is the left/right unit enabled?
 * @param new_enabled new enabled value
 */
void
AX12GripperInterface::SetEnabledMessage::set_enabled(const bool new_enabled)
{
  data->enabled = new_enabled;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::SetEnabledMessage::clone() const
{
  return new AX12GripperInterface::SetEnabledMessage(this);
}
/** @class AX12GripperInterface::SetVelocityMessage <interfaces/AX12GripperInterface.h>
 * SetVelocityMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_left_velocity initial value for left_velocity
 * @param ini_right_velocity initial value for right_velocity
 */
AX12GripperInterface::SetVelocityMessage::SetVelocityMessage(const float ini_left_velocity, const float ini_right_velocity) : Message("SetVelocityMessage")
{
  data_size = sizeof(SetVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->left_velocity = ini_left_velocity;
  data->right_velocity = ini_right_velocity;
  add_fieldinfo(IFT_FLOAT, "left_velocity", 1, &data->left_velocity);
  add_fieldinfo(IFT_FLOAT, "right_velocity", 1, &data->right_velocity);
}
/** Constructor */
AX12GripperInterface::SetVelocityMessage::SetVelocityMessage() : Message("SetVelocityMessage")
{
  data_size = sizeof(SetVelocityMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "left_velocity", 1, &data->left_velocity);
  add_fieldinfo(IFT_FLOAT, "right_velocity", 1, &data->right_velocity);
}

/** Destructor */
AX12GripperInterface::SetVelocityMessage::~SetVelocityMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::SetVelocityMessage::SetVelocityMessage(const SetVelocityMessage *m) : Message("SetVelocityMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetVelocityMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get left_velocity value.
 * Maximum left velocity currently reached.
 * @return left_velocity value
 */
float
AX12GripperInterface::SetVelocityMessage::left_velocity() const
{
  return data->left_velocity;
}

/** Get maximum length of left_velocity value.
 * @return length of left_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::SetVelocityMessage::maxlenof_left_velocity() const
{
  return 1;
}

/** Set left_velocity value.
 * Maximum left velocity currently reached.
 * @param new_left_velocity new left_velocity value
 */
void
AX12GripperInterface::SetVelocityMessage::set_left_velocity(const float new_left_velocity)
{
  data->left_velocity = new_left_velocity;
}

/** Get right_velocity value.
 * Maximum right velocity currently reached.
 * @return right_velocity value
 */
float
AX12GripperInterface::SetVelocityMessage::right_velocity() const
{
  return data->right_velocity;
}

/** Get maximum length of right_velocity value.
 * @return length of right_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::SetVelocityMessage::maxlenof_right_velocity() const
{
  return 1;
}

/** Set right_velocity value.
 * Maximum right velocity currently reached.
 * @param new_right_velocity new right_velocity value
 */
void
AX12GripperInterface::SetVelocityMessage::set_right_velocity(const float new_right_velocity)
{
  data->right_velocity = new_right_velocity;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::SetVelocityMessage::clone() const
{
  return new AX12GripperInterface::SetVelocityMessage(this);
}
/** @class AX12GripperInterface::SetMarginMessage <interfaces/AX12GripperInterface.h>
 * SetMarginMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_left_margin initial value for left_margin
 * @param ini_right_margin initial value for right_margin
 */
AX12GripperInterface::SetMarginMessage::SetMarginMessage(const float ini_left_margin, const float ini_right_margin) : Message("SetMarginMessage")
{
  data_size = sizeof(SetMarginMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->left_margin = ini_left_margin;
  data->right_margin = ini_right_margin;
  add_fieldinfo(IFT_FLOAT, "left_margin", 1, &data->left_margin);
  add_fieldinfo(IFT_FLOAT, "right_margin", 1, &data->right_margin);
}
/** Constructor */
AX12GripperInterface::SetMarginMessage::SetMarginMessage() : Message("SetMarginMessage")
{
  data_size = sizeof(SetMarginMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_FLOAT, "left_margin", 1, &data->left_margin);
  add_fieldinfo(IFT_FLOAT, "right_margin", 1, &data->right_margin);
}

/** Destructor */
AX12GripperInterface::SetMarginMessage::~SetMarginMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
AX12GripperInterface::SetMarginMessage::SetMarginMessage(const SetMarginMessage *m) : Message("SetMarginMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetMarginMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get left_margin value.
 * Margin in radians around a
    target left value to consider the motion as final.
 * @return left_margin value
 */
float
AX12GripperInterface::SetMarginMessage::left_margin() const
{
  return data->left_margin;
}

/** Get maximum length of left_margin value.
 * @return length of left_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::SetMarginMessage::maxlenof_left_margin() const
{
  return 1;
}

/** Set left_margin value.
 * Margin in radians around a
    target left value to consider the motion as final.
 * @param new_left_margin new left_margin value
 */
void
AX12GripperInterface::SetMarginMessage::set_left_margin(const float new_left_margin)
{
  data->left_margin = new_left_margin;
}

/** Get right_margin value.
 * Margin in radians around a
    target right value to consider the motion as final.
 * @return right_margin value
 */
float
AX12GripperInterface::SetMarginMessage::right_margin() const
{
  return data->right_margin;
}

/** Get maximum length of right_margin value.
 * @return length of right_margin value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AX12GripperInterface::SetMarginMessage::maxlenof_right_margin() const
{
  return 1;
}

/** Set right_margin value.
 * Margin in radians around a
    target right value to consider the motion as final.
 * @param new_right_margin new right_margin value
 */
void
AX12GripperInterface::SetMarginMessage::set_right_margin(const float new_right_margin)
{
  data->right_margin = new_right_margin;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
AX12GripperInterface::SetMarginMessage::clone() const
{
  return new AX12GripperInterface::SetMarginMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
AX12GripperInterface::message_valid(const Message *message) const
{
  const Open_AngleMessage *m0 = dynamic_cast<const Open_AngleMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const CloseLoadMessage *m1 = dynamic_cast<const CloseLoadMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const CenterMessage *m2 = dynamic_cast<const CenterMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  const CloseMessage *m3 = dynamic_cast<const CloseMessage *>(message);
  if ( m3 != NULL ) {
    return true;
  }
  const OpenMessage *m4 = dynamic_cast<const OpenMessage *>(message);
  if ( m4 != NULL ) {
    return true;
  }
  const RelGotoZMessage *m5 = dynamic_cast<const RelGotoZMessage *>(message);
  if ( m5 != NULL ) {
    return true;
  }
  const StopLeftMessage *m6 = dynamic_cast<const StopLeftMessage *>(message);
  if ( m6 != NULL ) {
    return true;
  }
  const StopRightMessage *m7 = dynamic_cast<const StopRightMessage *>(message);
  if ( m7 != NULL ) {
    return true;
  }
  const StopMessage *m8 = dynamic_cast<const StopMessage *>(message);
  if ( m8 != NULL ) {
    return true;
  }
  const FlushMessage *m9 = dynamic_cast<const FlushMessage *>(message);
  if ( m9 != NULL ) {
    return true;
  }
  const CalibrateMessage *m10 = dynamic_cast<const CalibrateMessage *>(message);
  if ( m10 != NULL ) {
    return true;
  }
  const ParkMessage *m11 = dynamic_cast<const ParkMessage *>(message);
  if ( m11 != NULL ) {
    return true;
  }
  const GotoMessage *m12 = dynamic_cast<const GotoMessage *>(message);
  if ( m12 != NULL ) {
    return true;
  }
  const TimedGotoMessage *m13 = dynamic_cast<const TimedGotoMessage *>(message);
  if ( m13 != NULL ) {
    return true;
  }
  const SetServoMessage *m14 = dynamic_cast<const SetServoMessage *>(message);
  if ( m14 != NULL ) {
    return true;
  }
  const SetEnabledMessage *m15 = dynamic_cast<const SetEnabledMessage *>(message);
  if ( m15 != NULL ) {
    return true;
  }
  const SetVelocityMessage *m16 = dynamic_cast<const SetVelocityMessage *>(message);
  if ( m16 != NULL ) {
    return true;
  }
  const SetMarginMessage *m17 = dynamic_cast<const SetMarginMessage *>(message);
  if ( m17 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(AX12GripperInterface)
/// @endcond


} // end namespace fawkes
