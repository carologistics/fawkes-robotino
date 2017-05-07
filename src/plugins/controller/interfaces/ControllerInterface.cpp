
/***************************************************************************
 *  ControllerInterface.cpp - Fawkes BlackBoard Interface - ControllerInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2017  Christoph Henke
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

#include <interfaces/ControllerInterface.h>

#include <core/exceptions/software.h>

#include <map>
#include <string>
#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ControllerInterface <interfaces/ControllerInterface.h>
 * ControllerInterface Fawkes BlackBoard Interface.
 * 
      The interface is used for calling and parameterization of the controller plugin
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
ControllerInterface::ControllerInterface() : Interface()
{
  data_size = sizeof(ControllerInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ControllerInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_BOOL, "controller_running", 1, &data->controller_running);
  add_fieldinfo(IFT_STRING, "robot_frame", 64, data->robot_frame);
  add_fieldinfo(IFT_STRING, "target_frame", 64, data->target_frame);
  add_fieldinfo(IFT_FLOAT, "x_error", 1, &data->x_error);
  add_fieldinfo(IFT_FLOAT, "y_error", 1, &data->y_error);
  add_fieldinfo(IFT_FLOAT, "ori_error", 1, &data->ori_error);
  add_fieldinfo(IFT_FLOAT, "x_offset", 1, &data->x_offset);
  add_fieldinfo(IFT_FLOAT, "y_offset", 1, &data->y_offset);
  add_fieldinfo(IFT_FLOAT, "ori_offset", 1, &data->ori_offset);
  add_fieldinfo(IFT_FLOAT, "kp", 1, &data->kp);
  add_fieldinfo(IFT_FLOAT, "ki", 1, &data->ki);
  add_fieldinfo(IFT_FLOAT, "kd", 1, &data->kd);
  add_messageinfo("PidControlMessage");
  unsigned char tmp_hash[] = {0x62, 0x5b, 0xbf, 0xae, 0x40, 0x73, 0xb8, 0x14, 0x9a, 0xe0, 0x75, 0x97, 0x2, 0x3f, 0x40, 0xbd};
  set_hash(tmp_hash);
}

/** Destructor */
ControllerInterface::~ControllerInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get controller_running value.
 * 
      True if the controller is currently performing a control task.
    
 * @return controller_running value
 */
bool
ControllerInterface::is_controller_running() const
{
  return data->controller_running;
}

/** Get maximum length of controller_running value.
 * @return length of controller_running value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_controller_running() const
{
  return 1;
}

/** Set controller_running value.
 * 
      True if the controller is currently performing a control task.
    
 * @param new_controller_running new controller_running value
 */
void
ControllerInterface::set_controller_running(const bool new_controller_running)
{
  data->controller_running = new_controller_running;
  data_changed = true;
}

/** Get robot_frame value.
 * 
      The current frame to control onto the target frame (an offset in x, y, ori is addable).
 * @return robot_frame value
 */
char *
ControllerInterface::robot_frame() const
{
  return data->robot_frame;
}

/** Get maximum length of robot_frame value.
 * @return length of robot_frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_robot_frame() const
{
  return 64;
}

/** Set robot_frame value.
 * 
      The current frame to control onto the target frame (an offset in x, y, ori is addable).
 * @param new_robot_frame new robot_frame value
 */
void
ControllerInterface::set_robot_frame(const char * new_robot_frame)
{
  strncpy(data->robot_frame, new_robot_frame, sizeof(data->robot_frame));
  data_changed = true;
}

/** Get target_frame value.
 * 
      The current target frame.
 * @return target_frame value
 */
char *
ControllerInterface::target_frame() const
{
  return data->target_frame;
}

/** Get maximum length of target_frame value.
 * @return length of target_frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_target_frame() const
{
  return 64;
}

/** Set target_frame value.
 * 
      The current target frame.
 * @param new_target_frame new target_frame value
 */
void
ControllerInterface::set_target_frame(const char * new_target_frame)
{
  strncpy(data->target_frame, new_target_frame, sizeof(data->target_frame));
  data_changed = true;
}

/** Get x_error value.
 * 
      The current x error between robot and target frame to be achieved.
 * @return x_error value
 */
float
ControllerInterface::x_error() const
{
  return data->x_error;
}

/** Get maximum length of x_error value.
 * @return length of x_error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_x_error() const
{
  return 1;
}

/** Set x_error value.
 * 
      The current x error between robot and target frame to be achieved.
 * @param new_x_error new x_error value
 */
void
ControllerInterface::set_x_error(const float new_x_error)
{
  data->x_error = new_x_error;
  data_changed = true;
}

/** Get y_error value.
 * 
      The current y error between robot and target frame to be achieved.
 * @return y_error value
 */
float
ControllerInterface::y_error() const
{
  return data->y_error;
}

/** Get maximum length of y_error value.
 * @return length of y_error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_y_error() const
{
  return 1;
}

/** Set y_error value.
 * 
      The current y error between robot and target frame to be achieved.
 * @param new_y_error new y_error value
 */
void
ControllerInterface::set_y_error(const float new_y_error)
{
  data->y_error = new_y_error;
  data_changed = true;
}

/** Get ori_error value.
 * 
      The current orientational error between robot and target frame to be achieved.
 * @return ori_error value
 */
float
ControllerInterface::ori_error() const
{
  return data->ori_error;
}

/** Get maximum length of ori_error value.
 * @return length of ori_error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_ori_error() const
{
  return 1;
}

/** Set ori_error value.
 * 
      The current orientational error between robot and target frame to be achieved.
 * @param new_ori_error new ori_error value
 */
void
ControllerInterface::set_ori_error(const float new_ori_error)
{
  data->ori_error = new_ori_error;
  data_changed = true;
}

/** Get x_offset value.
 * 
      The current x offset between robot and target frame.
 * @return x_offset value
 */
float
ControllerInterface::x_offset() const
{
  return data->x_offset;
}

/** Get maximum length of x_offset value.
 * @return length of x_offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_x_offset() const
{
  return 1;
}

/** Set x_offset value.
 * 
      The current x offset between robot and target frame.
 * @param new_x_offset new x_offset value
 */
void
ControllerInterface::set_x_offset(const float new_x_offset)
{
  data->x_offset = new_x_offset;
  data_changed = true;
}

/** Get y_offset value.
 * 
      The current y offset between robot and target frame.
 * @return y_offset value
 */
float
ControllerInterface::y_offset() const
{
  return data->y_offset;
}

/** Get maximum length of y_offset value.
 * @return length of y_offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_y_offset() const
{
  return 1;
}

/** Set y_offset value.
 * 
      The current y offset between robot and target frame.
 * @param new_y_offset new y_offset value
 */
void
ControllerInterface::set_y_offset(const float new_y_offset)
{
  data->y_offset = new_y_offset;
  data_changed = true;
}

/** Get ori_offset value.
 * 
      The current orientational offset between robot and target frame.
 * @return ori_offset value
 */
float
ControllerInterface::ori_offset() const
{
  return data->ori_offset;
}

/** Get maximum length of ori_offset value.
 * @return length of ori_offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_ori_offset() const
{
  return 1;
}

/** Set ori_offset value.
 * 
      The current orientational offset between robot and target frame.
 * @param new_ori_offset new ori_offset value
 */
void
ControllerInterface::set_ori_offset(const float new_ori_offset)
{
  data->ori_offset = new_ori_offset;
  data_changed = true;
}

/** Get kp value.
 * 
      The current proportional controller parameter.
 * @return kp value
 */
float
ControllerInterface::kp() const
{
  return data->kp;
}

/** Get maximum length of kp value.
 * @return length of kp value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_kp() const
{
  return 1;
}

/** Set kp value.
 * 
      The current proportional controller parameter.
 * @param new_kp new kp value
 */
void
ControllerInterface::set_kp(const float new_kp)
{
  data->kp = new_kp;
  data_changed = true;
}

/** Get ki value.
 * 
      The current integral controller parameter.
 * @return ki value
 */
float
ControllerInterface::ki() const
{
  return data->ki;
}

/** Get maximum length of ki value.
 * @return length of ki value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_ki() const
{
  return 1;
}

/** Set ki value.
 * 
      The current integral controller parameter.
 * @param new_ki new ki value
 */
void
ControllerInterface::set_ki(const float new_ki)
{
  data->ki = new_ki;
  data_changed = true;
}

/** Get kd value.
 * 
      The current differential controller parameter.
 * @return kd value
 */
float
ControllerInterface::kd() const
{
  return data->kd;
}

/** Get maximum length of kd value.
 * @return length of kd value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::maxlenof_kd() const
{
  return 1;
}

/** Set kd value.
 * 
      The current differential controller parameter.
 * @param new_kd new kd value
 */
void
ControllerInterface::set_kd(const float new_kd)
{
  data->kd = new_kd;
  data_changed = true;
}

/* =========== message create =========== */
Message *
ControllerInterface::create_message(const char *type) const
{
  if ( strncmp("PidControlMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new PidControlMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
ControllerInterface::copy_values(const Interface *other)
{
  const ControllerInterface *oi = dynamic_cast<const ControllerInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(ControllerInterface_data_t));
}

const char *
ControllerInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class ControllerInterface::PidControlMessage <interfaces/ControllerInterface.h>
 * PidControlMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_robot_frame initial value for robot_frame
 * @param ini_target_frame initial value for target_frame
 * @param ini_x_error initial value for x_error
 * @param ini_y_error initial value for y_error
 * @param ini_ori_error initial value for ori_error
 * @param ini_x_offset initial value for x_offset
 * @param ini_y_offset initial value for y_offset
 * @param ini_ori_offset initial value for ori_offset
 * @param ini_kp initial value for kp
 * @param ini_ki initial value for ki
 * @param ini_kd initial value for kd
 */
ControllerInterface::PidControlMessage::PidControlMessage(const char * ini_robot_frame, const char * ini_target_frame, const float ini_x_error, const float ini_y_error, const float ini_ori_error, const float ini_x_offset, const float ini_y_offset, const float ini_ori_offset, const float ini_kp, const float ini_ki, const float ini_kd) : Message("PidControlMessage")
{
  data_size = sizeof(PidControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PidControlMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  strncpy(data->robot_frame, ini_robot_frame, 64);
  strncpy(data->target_frame, ini_target_frame, 64);
  data->x_error = ini_x_error;
  data->y_error = ini_y_error;
  data->ori_error = ini_ori_error;
  data->x_offset = ini_x_offset;
  data->y_offset = ini_y_offset;
  data->ori_offset = ini_ori_offset;
  data->kp = ini_kp;
  data->ki = ini_ki;
  data->kd = ini_kd;
  add_fieldinfo(IFT_STRING, "robot_frame", 64, data->robot_frame);
  add_fieldinfo(IFT_STRING, "target_frame", 64, data->target_frame);
  add_fieldinfo(IFT_FLOAT, "x_error", 1, &data->x_error);
  add_fieldinfo(IFT_FLOAT, "y_error", 1, &data->y_error);
  add_fieldinfo(IFT_FLOAT, "ori_error", 1, &data->ori_error);
  add_fieldinfo(IFT_FLOAT, "x_offset", 1, &data->x_offset);
  add_fieldinfo(IFT_FLOAT, "y_offset", 1, &data->y_offset);
  add_fieldinfo(IFT_FLOAT, "ori_offset", 1, &data->ori_offset);
  add_fieldinfo(IFT_FLOAT, "kp", 1, &data->kp);
  add_fieldinfo(IFT_FLOAT, "ki", 1, &data->ki);
  add_fieldinfo(IFT_FLOAT, "kd", 1, &data->kd);
}
/** Constructor */
ControllerInterface::PidControlMessage::PidControlMessage() : Message("PidControlMessage")
{
  data_size = sizeof(PidControlMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (PidControlMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_STRING, "robot_frame", 64, data->robot_frame);
  add_fieldinfo(IFT_STRING, "target_frame", 64, data->target_frame);
  add_fieldinfo(IFT_FLOAT, "x_error", 1, &data->x_error);
  add_fieldinfo(IFT_FLOAT, "y_error", 1, &data->y_error);
  add_fieldinfo(IFT_FLOAT, "ori_error", 1, &data->ori_error);
  add_fieldinfo(IFT_FLOAT, "x_offset", 1, &data->x_offset);
  add_fieldinfo(IFT_FLOAT, "y_offset", 1, &data->y_offset);
  add_fieldinfo(IFT_FLOAT, "ori_offset", 1, &data->ori_offset);
  add_fieldinfo(IFT_FLOAT, "kp", 1, &data->kp);
  add_fieldinfo(IFT_FLOAT, "ki", 1, &data->ki);
  add_fieldinfo(IFT_FLOAT, "kd", 1, &data->kd);
}

/** Destructor */
ControllerInterface::PidControlMessage::~PidControlMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ControllerInterface::PidControlMessage::PidControlMessage(const PidControlMessage *m) : Message("PidControlMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (PidControlMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get robot_frame value.
 * The frame to control onto the target frame (an offset in x, y, ori is addable).
 * @return robot_frame value
 */
char *
ControllerInterface::PidControlMessage::robot_frame() const
{
  return data->robot_frame;
}

/** Get maximum length of robot_frame value.
 * @return length of robot_frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_robot_frame() const
{
  return 64;
}

/** Set robot_frame value.
 * The frame to control onto the target frame (an offset in x, y, ori is addable).
 * @param new_robot_frame new robot_frame value
 */
void
ControllerInterface::PidControlMessage::set_robot_frame(const char * new_robot_frame)
{
  strncpy(data->robot_frame, new_robot_frame, sizeof(data->robot_frame));
}

/** Get target_frame value.
 * The target frame.
 * @return target_frame value
 */
char *
ControllerInterface::PidControlMessage::target_frame() const
{
  return data->target_frame;
}

/** Get maximum length of target_frame value.
 * @return length of target_frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_target_frame() const
{
  return 64;
}

/** Set target_frame value.
 * The target frame.
 * @param new_target_frame new target_frame value
 */
void
ControllerInterface::PidControlMessage::set_target_frame(const char * new_target_frame)
{
  strncpy(data->target_frame, new_target_frame, sizeof(data->target_frame));
}

/** Get x_error value.
 * The x error between robot and target frame to be achieved.
 * @return x_error value
 */
float
ControllerInterface::PidControlMessage::x_error() const
{
  return data->x_error;
}

/** Get maximum length of x_error value.
 * @return length of x_error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_x_error() const
{
  return 1;
}

/** Set x_error value.
 * The x error between robot and target frame to be achieved.
 * @param new_x_error new x_error value
 */
void
ControllerInterface::PidControlMessage::set_x_error(const float new_x_error)
{
  data->x_error = new_x_error;
}

/** Get y_error value.
 * The y error between robot and target frame to be achieved.
 * @return y_error value
 */
float
ControllerInterface::PidControlMessage::y_error() const
{
  return data->y_error;
}

/** Get maximum length of y_error value.
 * @return length of y_error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_y_error() const
{
  return 1;
}

/** Set y_error value.
 * The y error between robot and target frame to be achieved.
 * @param new_y_error new y_error value
 */
void
ControllerInterface::PidControlMessage::set_y_error(const float new_y_error)
{
  data->y_error = new_y_error;
}

/** Get ori_error value.
 * The orientational error between robot and target frame to be achieved.
 * @return ori_error value
 */
float
ControllerInterface::PidControlMessage::ori_error() const
{
  return data->ori_error;
}

/** Get maximum length of ori_error value.
 * @return length of ori_error value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_ori_error() const
{
  return 1;
}

/** Set ori_error value.
 * The orientational error between robot and target frame to be achieved.
 * @param new_ori_error new ori_error value
 */
void
ControllerInterface::PidControlMessage::set_ori_error(const float new_ori_error)
{
  data->ori_error = new_ori_error;
}

/** Get x_offset value.
 * The x offset between robot and target frame.
 * @return x_offset value
 */
float
ControllerInterface::PidControlMessage::x_offset() const
{
  return data->x_offset;
}

/** Get maximum length of x_offset value.
 * @return length of x_offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_x_offset() const
{
  return 1;
}

/** Set x_offset value.
 * The x offset between robot and target frame.
 * @param new_x_offset new x_offset value
 */
void
ControllerInterface::PidControlMessage::set_x_offset(const float new_x_offset)
{
  data->x_offset = new_x_offset;
}

/** Get y_offset value.
 * The y offset between robot and target frame.
 * @return y_offset value
 */
float
ControllerInterface::PidControlMessage::y_offset() const
{
  return data->y_offset;
}

/** Get maximum length of y_offset value.
 * @return length of y_offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_y_offset() const
{
  return 1;
}

/** Set y_offset value.
 * The y offset between robot and target frame.
 * @param new_y_offset new y_offset value
 */
void
ControllerInterface::PidControlMessage::set_y_offset(const float new_y_offset)
{
  data->y_offset = new_y_offset;
}

/** Get ori_offset value.
 * The orientational offset between robot and target frame.
 * @return ori_offset value
 */
float
ControllerInterface::PidControlMessage::ori_offset() const
{
  return data->ori_offset;
}

/** Get maximum length of ori_offset value.
 * @return length of ori_offset value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_ori_offset() const
{
  return 1;
}

/** Set ori_offset value.
 * The orientational offset between robot and target frame.
 * @param new_ori_offset new ori_offset value
 */
void
ControllerInterface::PidControlMessage::set_ori_offset(const float new_ori_offset)
{
  data->ori_offset = new_ori_offset;
}

/** Get kp value.
 * The proportional controller parameter.
 * @return kp value
 */
float
ControllerInterface::PidControlMessage::kp() const
{
  return data->kp;
}

/** Get maximum length of kp value.
 * @return length of kp value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_kp() const
{
  return 1;
}

/** Set kp value.
 * The proportional controller parameter.
 * @param new_kp new kp value
 */
void
ControllerInterface::PidControlMessage::set_kp(const float new_kp)
{
  data->kp = new_kp;
}

/** Get ki value.
 * The integral controller parameter.
 * @return ki value
 */
float
ControllerInterface::PidControlMessage::ki() const
{
  return data->ki;
}

/** Get maximum length of ki value.
 * @return length of ki value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_ki() const
{
  return 1;
}

/** Set ki value.
 * The integral controller parameter.
 * @param new_ki new ki value
 */
void
ControllerInterface::PidControlMessage::set_ki(const float new_ki)
{
  data->ki = new_ki;
}

/** Get kd value.
 * The differential controller parameter.
 * @return kd value
 */
float
ControllerInterface::PidControlMessage::kd() const
{
  return data->kd;
}

/** Get maximum length of kd value.
 * @return length of kd value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ControllerInterface::PidControlMessage::maxlenof_kd() const
{
  return 1;
}

/** Set kd value.
 * The differential controller parameter.
 * @param new_kd new kd value
 */
void
ControllerInterface::PidControlMessage::set_kd(const float new_kd)
{
  data->kd = new_kd;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ControllerInterface::PidControlMessage::clone() const
{
  return new ControllerInterface::PidControlMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
ControllerInterface::message_valid(const Message *message) const
{
  const PidControlMessage *m0 = dynamic_cast<const PidControlMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ControllerInterface)
/// @endcond


} // end namespace fawkes
