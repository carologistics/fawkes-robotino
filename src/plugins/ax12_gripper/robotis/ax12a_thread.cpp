
/***************************************************************************
 *  ax12a_thread.cpp - AX12A Gripper unit act thread
 *
 *  Created: Sat Feb 28 10:30:04 2015
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *                  2015  Nicolas Limpert
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "ax12a_thread.h"
#include "ax12a.h"

#include <utils/math/angle.h>
#include <core/threading/mutex_locker.h>
#include <core/threading/read_write_lock.h>
#include <core/threading/scoped_rwlock.h>
#include <core/threading/wait_condition.h>
#include <interfaces/AX12GripperInterface.h>
#include <interfaces/LedInterface.h>
#include <interfaces/JointInterface.h>

#include <cstdarg>
#include <cmath>
#include <unistd.h>

using namespace fawkes;

/** @class GripperAX12AThread "ax12a_thread.h"
 * Gripper act thread for AX12A Gripper.
 * This thread integrates into the Fawkes main loop at the ACT_EXEC hook and
 * interacts with the controller of the AX12 Gripper.
 * @author Tim Niemueller
 */
/** Constructor.
 * @param gripper_cfg_prefix gripper plugin configuration prefix
 * @param ptu_cfg_prefix configuration prefix specific for the Gripper
 * @param ptu_name name of the Gripper configuration
 */
GripperAX12AThread::GripperAX12AThread(std::string &gripper_cfg_prefix)
  : AX12GripperActThread("GripperAX12AThread"),
#ifdef HAVE_TF
       TransformAspect(TransformAspect::ONLY_PUBLISHER,
                       std::string("GRIPPER AX12A").c_str()),
#endif
    BlackBoardInterfaceListener("GripperAX12AThread()")
{
  set_name("GripperAX12AThread()");

  __gripper_cfg_prefix = gripper_cfg_prefix;
  __ptu_cfg_prefix     = gripper_cfg_prefix;
  //  __ptu_name           = ptu_name;

  __ax12a = NULL;
}


void
GripperAX12AThread::init()
{
  __last_left = __last_right = 0.f;
  float init_left_velocity = 0.f;
  float init_right_velocity = 0.f;

  // Note: due to the use of auto_ptr and RefPtr resources are automatically
  // freed on destruction, therefore no special handling is necessary in init()
  // itself!
  __cfg_device           = config->get_string((__gripper_cfg_prefix + "device").c_str());
  __cfg_left_servo_id    = config->get_uint((__gripper_cfg_prefix + "left_servo_id").c_str());
  __cfg_right_servo_id   = config->get_uint((__gripper_cfg_prefix + "right_servo_id").c_str());
  __cfg_read_timeout_ms  = config->get_uint((__gripper_cfg_prefix + "read_timeout_ms").c_str());
  __cfg_disc_timeout_ms  = config->get_uint((__gripper_cfg_prefix + "discover_timeout_ms").c_str());
  __cfg_goto_zero_start  = config->get_bool((__gripper_cfg_prefix + "goto_zero_start").c_str());
  __cfg_turn_off         = config->get_bool((__gripper_cfg_prefix + "turn_off").c_str());
  __cfg_left_min         = config->get_float((__gripper_cfg_prefix + "left_min").c_str());
  __cfg_left_max         = config->get_float((__gripper_cfg_prefix + "left_max").c_str());
  __cfg_right_min        = config->get_float((__gripper_cfg_prefix + "right_min").c_str());
  __cfg_right_max        = config->get_float((__gripper_cfg_prefix + "right_max").c_str());
  __cfg_left_torque      = config->get_float((__gripper_cfg_prefix + "left_torque").c_str());
  __cfg_right_torque     = config->get_float((__gripper_cfg_prefix + "right_torque").c_str());

  __cfg_left_margin       = config->get_float((__ptu_cfg_prefix + "left_margin").c_str());
  __cfg_right_margin      = config->get_float((__ptu_cfg_prefix + "right_margin").c_str());
  __cfg_left_start        = config->get_float((__ptu_cfg_prefix + "left_start").c_str());
  __cfg_right_start       = config->get_float((__ptu_cfg_prefix + "right_start").c_str());
  __cfg_left_open_angle   = config->get_float((__ptu_cfg_prefix + "left_open").c_str());
  __cfg_left_close_angle  = config->get_float((__ptu_cfg_prefix + "left_close").c_str());
  __cfg_left_close_load_angle  = config->get_float((__ptu_cfg_prefix + "left_close_load").c_str());
  __cfg_right_open_angle  = config->get_float((__ptu_cfg_prefix + "right_open").c_str());
  __cfg_right_close_angle = config->get_float((__ptu_cfg_prefix + "right_close").c_str());
  __cfg_right_close_load_angle = config->get_float((__ptu_cfg_prefix + "right_close_load").c_str());
  __cfg_max_speed         = config->get_float((__ptu_cfg_prefix + "max_speed").c_str());
  __cfg_max_load          = config->get_float((__ptu_cfg_prefix + "max_load").c_str());

#ifdef HAVE_TF
  __cfg_publish_transforms=config->get_bool((__ptu_cfg_prefix + "publish_transforms").c_str());
#endif

#ifdef HAVE_TF
  if (__cfg_publish_transforms) {
    // float left_trans_x  =
    //     config->get_float((__ptu_cfg_prefix + "left_trans_x").c_str());
    // float left_trans_y  =
    //     config->get_float((__ptu_cfg_prefix + "left_trans_y").c_str());
    // float left_trans_z  =
    //     config->get_float((__ptu_cfg_prefix + "left_trans_z").c_str());
    // float right_trans_x =
    //     config->get_float((__ptu_cfg_prefix + "right_trans_x").c_str());
    // float right_trans_y =
    //     config->get_float((__ptu_cfg_prefix + "right_trans_y").c_str());
    // float right_trans_z =
    //     config->get_float((__ptu_cfg_prefix + "right_trans_z").c_str());


    std::string frame_id_prefix = std::string("/") + __ptu_name;
    try {
      frame_id_prefix =
          config->get_string((__ptu_cfg_prefix + "frame_id_prefix").c_str());
    } catch (Exception &e) {} // ignore, use default

    __cfg_base_frame = frame_id_prefix + "/base";
    __cfg_left_link   = frame_id_prefix + "/left";
    __cfg_right_link  = frame_id_prefix + "/right";

    // __translation_left.setValue(left_trans_x, left_trans_y, left_trans_z);
    // __translation_right.setValue(right_trans_x, right_trans_y, right_trans_z);
  }
#endif

  bool left_servo_found = false, right_servo_found = false;

  __ax12a = new RobotisAX12A(__cfg_device.c_str(), __cfg_read_timeout_ms);
  RobotisAX12A::DeviceList devl = __ax12a->discover();
  for (RobotisAX12A::DeviceList::iterator i = devl.begin(); i != devl.end(); ++i) {
    if (__cfg_left_servo_id == *i) {
      left_servo_found  = true;
    } else if (__cfg_right_servo_id == *i) {
      right_servo_found = true;
    } else {
      logger->log_warn(name(), "Servo %u in Gripper servo chain, but neither "
		       "configured as left nor as right servo", *i);
    }
  }
  __ax12a->set_max_torque(__cfg_right_servo_id, __cfg_right_torque * 1023);
  __ax12a->set_max_torque(__cfg_left_servo_id, __cfg_left_torque * 1023);
  // __ax12a->set_goal_speeds(2, __cfg_left_servo_id, __cfg_max_speed * 1023, __cfg_right_servo_id, __cfg_max_speed * 1023);
  // __ax12a->set_torque_enabled(__cfg_left_servo_id, true);
  // __ax12a->set_torque_enabled(__cfg_right_servo_id, true);
  printf("left torque: %d\n", __ax12a->get_max_torque(__cfg_left_servo_id, true));
  printf("right torque: %d\n", __ax12a->get_max_torque(__cfg_right_servo_id, true));
  printf("left torque enabled: %d\n", __ax12a->is_torque_enabled(__cfg_left_servo_id, true));
  printf("right torque enaled: %d\n", __ax12a->is_torque_enabled(__cfg_right_servo_id, true));
  printf("left moving speed: %d\n", __ax12a->get_goal_speed(__cfg_left_servo_id, true));
  printf("right moving speed: %d\n", __ax12a->get_goal_speed(__cfg_right_servo_id, true));
  // We only want responses to be sent on explicit READ to speed up communication
  __ax12a->set_status_return_level(RobotisAX12A::BROADCAST_ID, RobotisAX12A::SRL_RESPOND_READ);
  // set compliance values
  __ax12a->set_compliance_values(RobotisAX12A::BROADCAST_ID,
				__cfg_cw_compl_margin, __cfg_cw_compl_slope,
				__cfg_ccw_compl_margin, __cfg_ccw_compl_slope);
  // __ax12a->set_led_enabled(__cfg_right_servo_id, true);
  // __ax12a->set_led_enabled(__cfg_left_servo_id, false);

  // Test of tilt and pitch unit
  __ax12a->set_goal_speed(1,50);
  __ax12a->goto_position(1,512);
  __ax12a->set_goal_speed(2,50);
  __ax12a->goto_position(2,350);

  if (! (left_servo_found && right_servo_found)) {
    throw Exception("Left and/or right servo not found: left: %i  right: %i",
		    left_servo_found, right_servo_found);
  }

  // If you have more than one interface: catch exception and close them!
  std::string bbid = "Gripper AX12";
  __gripper_if = blackboard->open_for_writing<AX12GripperInterface>(bbid.c_str());
  __gripper_if->set_calibrated(true);
  __gripper_if->set_min_left(__cfg_left_min);
  __gripper_if->set_max_left(__cfg_left_max);
  __gripper_if->set_min_right(__cfg_right_min);
  __gripper_if->set_max_right(__cfg_right_max);
  __gripper_if->set_left_margin(__cfg_left_margin);
  __gripper_if->set_right_margin(__cfg_right_margin);
  __gripper_if->set_max_left_velocity(__ax12a->get_max_supported_speed(__cfg_left_servo_id));
  __gripper_if->set_max_right_velocity(__ax12a->get_max_supported_speed(__cfg_right_servo_id));
  __gripper_if->set_left_velocity(init_left_velocity);
  __gripper_if->set_right_velocity(init_right_velocity);
  __gripper_if->write();

  __led_if = blackboard->open_for_writing<LedInterface>(bbid.c_str());

  std::string leftid = __ptu_name + " left";
  __leftjoint_if = blackboard->open_for_writing<JointInterface>(leftid.c_str());
  __leftjoint_if->set_position(__last_left);
  __leftjoint_if->set_velocity(init_left_velocity);
  __leftjoint_if->write();

  std::string rightid = __ptu_name + " right";
  __rightjoint_if = blackboard->open_for_writing<JointInterface>(rightid.c_str());
  __rightjoint_if->set_position(__last_right);
  __rightjoint_if->set_velocity(init_right_velocity);
  __rightjoint_if->write();

  __wt = new WorkerThread(__ptu_name, logger, __ax12a,
  			  __cfg_left_servo_id, __cfg_right_servo_id,
  			  __cfg_left_min, __cfg_left_max, __cfg_right_min, __cfg_right_max,
  			  __cfg_left_offset, __cfg_right_offset,
			  __cfg_max_load);
  __wt->set_margins(__cfg_left_margin, __cfg_right_margin);
  __wt->start();
  __wt->set_enabled(true);
  if ( __cfg_goto_zero_start ) {
    __wt->goto_gripper_timed(__cfg_left_start, __cfg_right_start, 3.0);
  }

  // printf("speeds from init to: %f, %f\n", __cfg_max_speed, __cfg_max_speed);
  __wt->set_velocities(__cfg_max_speed, __cfg_max_speed);

  bbil_add_message_interface(__gripper_if);
  bbil_add_message_interface(__leftjoint_if);
  bbil_add_message_interface(__rightjoint_if);
  blackboard->register_listener(this);

#ifdef USE_TIMETRACKER
  __tt.reset(new TimeTracker());
  __tt_count = 0;
  __ttc_read_sensor = __tt->add_class("Read Sensor");
#endif  

}


bool
GripperAX12AThread::prepare_finalize_user()
{
  if (__cfg_turn_off) {
    logger->log_info(name(), "Moving to park position");
    // __wt->goto_gripper_timed(__cfg_left_start, __cfg_right_start, 2.0);
    __wt->goto_gripper(__cfg_left_start, __cfg_right_start);
    // we need to wait twice, because the first wakeup is likely to happen
    // before the command is actually send
    __wt->wait_for_fresh_data();
    __wt->wait_for_fresh_data();

    while (! __wt->is_final()) {
      //__wt->wakeup();
      __wt->wait_for_fresh_data();
    }
  }
  return true;
}

void
GripperAX12AThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->close(__gripper_if);
  blackboard->close(__led_if);
  blackboard->close(__leftjoint_if);
  blackboard->close(__rightjoint_if);

  __wt->cancel();
  __wt->join();
  delete __wt;

  if (__cfg_turn_off) {
    logger->log_info(name(), "Turning off Gripper");
    try {
      __ax12a->set_led_enabled(__cfg_left_servo_id,  false);
      __ax12a->set_led_enabled(__cfg_right_servo_id, false);
      __ax12a->set_torques_enabled(false, 2, __cfg_left_servo_id, __cfg_right_servo_id);
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to turn of Gripper: %s", e.what());
    }
  }
  
  // Setting to NULL deletes instance (RefPtr)
  // TODO: Ask Tim if delete __ax12a needed
  __ax12a = NULL;
}


/** Update sensor values as necessary.
 * To be called only from GripperSensorThread. Writes the current left/right
 * data into the interface.
 */
void
GripperAX12AThread::update_sensor_values()
{
  if (__wt->has_fresh_data()) {
    float left = 0, right = 0, leftvel=0, rightvel=0;
    unsigned int left_load = 0, right_load = 0;
    fawkes::Time time;
    __wt->get_gripper(left, right, time);
    __wt->get_velocities(leftvel, rightvel);
    __wt->get_loads(left_load, right_load);

    // poor man's filter: only update if we get a change of least half a degree
    if (fabs(__last_left - left) >= 0.009 || fabs(__last_right - right) >= 0.009) {
      __last_left  = left;
      __last_right = right;
    } else {
      left  = __last_left;
      right = __last_right;
    }

    __gripper_if->set_left(left);
    __gripper_if->set_right(right);
    __gripper_if->set_left_velocity(leftvel);
    __gripper_if->set_right_velocity(rightvel);
    __gripper_if->set_enabled(__wt->is_enabled());
    __gripper_if->set_final(__wt->is_final());
    __gripper_if->set_left_load(left_load);
    __gripper_if->set_right_load(right_load);
    __gripper_if->write();

    __leftjoint_if->set_position(left);
    __leftjoint_if->set_velocity(leftvel);
    __leftjoint_if->write();

    __rightjoint_if->set_position(right);
    __rightjoint_if->set_velocity(rightvel);
    __rightjoint_if->write();

#ifdef HAVE_TF
    if (__cfg_publish_transforms) {
      // Always publish updated transforms
      tf::Quaternion pr;  pr.setEulerZYX(left, 0, 0);
      tf::Transform ptr(pr, __translation_left);
      tf_publisher->send_transform(ptr, time, __cfg_base_frame, __cfg_left_link);

      tf::Quaternion tr; tr.setEulerZYX(0, right, 0);
      tf::Transform ttr(tr, __translation_right);
      tf_publisher->send_transform(ttr, time, __cfg_left_link, __cfg_right_link);
    }
#endif
    // if (__move_load_pending)
    //   {
    //   __ax12a->get_load(__left_servo_id) & 0x1FF;
    //   __ax12a->get_load(__right_servo_id) & 0x1FF;
    //   }
  }
}


void
GripperAX12AThread::loop()
{
   __gripper_if->set_final(__wt->is_final());

  while (! __gripper_if->msgq_empty() ) {
    if (__gripper_if->msgq_first_is<AX12GripperInterface::CalibrateMessage>()) {
      // ignored

    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::GotoMessage>()) {
      AX12GripperInterface::GotoMessage *msg = __gripper_if->msgq_first(msg);

      __wt->goto_gripper(msg->left(), msg->right());
      __gripper_if->set_msgid(msg->id());
      __gripper_if->set_final(false);

    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::TimedGotoMessage>()) {
      AX12GripperInterface::TimedGotoMessage *msg = __gripper_if->msgq_first(msg);

      printf("timedgotomessage\n");
      __wt->goto_gripper_timed(msg->left(), msg->right(), msg->time_sec());
      __gripper_if->set_msgid(msg->id());
      __gripper_if->set_final(false);

    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::ParkMessage>()) {
      AX12GripperInterface::ParkMessage *msg = __gripper_if->msgq_first(msg);

      __wt->goto_gripper(0, 0);
      __gripper_if->set_msgid(msg->id());
      __gripper_if->set_final(false);

    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::SetEnabledMessage>()) {
      AX12GripperInterface::SetEnabledMessage *msg = __gripper_if->msgq_first(msg);

      __wt->set_enabled(msg->is_enabled());

    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::SetVelocityMessage>()) {
      AX12GripperInterface::SetVelocityMessage *msg = __gripper_if->msgq_first(msg);

      printf("set velocity message\n");

      if (msg->left_velocity() > __gripper_if->max_left_velocity()) {
  	logger->log_warn(name(), "Desired left velocity %f too high, max is %f",
  			 msg->left_velocity(), __gripper_if->max_left_velocity());
      } else if (msg->right_velocity() > __gripper_if->max_right_velocity()) {
  	logger->log_warn(name(), "Desired right velocity %f too high, max is %f",
  			 msg->right_velocity(), __gripper_if->max_right_velocity());
      } else {
  	__wt->set_velocities(msg->left_velocity(), msg->right_velocity());
      }

    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::OpenMessage>()) {
      AX12GripperInterface::OpenMessage *msg = __gripper_if->msgq_first(msg);
      printf("open left: %f, right: %f\n", __cfg_left_open_angle + msg->offset(), __cfg_right_open_angle + msg->offset());
      __wt->goto_gripper(__cfg_left_open_angle + msg->offset(), __cfg_right_open_angle + msg->offset());
      
    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::CloseMessage>()) {
      AX12GripperInterface::CloseMessage *msg = __gripper_if->msgq_first(msg);

      printf("performing close\n");
      __wt->goto_gripper(__cfg_left_close_angle, __cfg_right_close_angle);
      // __wt->set_move_load_pending(true);

    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::CloseLoadMessage>()) {
      AX12GripperInterface::CloseLoadMessage *msg = __gripper_if->msgq_first(msg);

      printf("performing close with load\n");
      __wt->goto_gripper(__cfg_left_close_load_angle, __cfg_right_close_load_angle);
      // __wt->set_move_load_pending(true);

   } else if (__gripper_if->msgq_first_is<AX12GripperInterface::Open_AngleMessage>()) {
      AX12GripperInterface::Open_AngleMessage *msg = __gripper_if->msgq_first(msg);
      float angle_part = msg->angle() / 2.;
      __wt->goto_gripper(-angle_part, angle_part);
      
    } else if (__gripper_if->msgq_first_is<AX12GripperInterface::SetMarginMessage>()) {
      AX12GripperInterface::SetMarginMessage *msg = __gripper_if->msgq_first(msg);

      __wt->set_margins(msg->left_margin(), msg->right_margin());
      __gripper_if->set_left_margin(msg->left_margin());
      __gripper_if->set_right_margin(msg->right_margin());

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __gripper_if->msgq_pop();
  }

  __gripper_if->write();

  bool write_led_if = false;
  while (! __led_if->msgq_empty() ) {
    write_led_if = true;
    if (__led_if->msgq_first_is<LedInterface::SetIntensityMessage>()) {
      LedInterface::SetIntensityMessage *msg = __led_if->msgq_first(msg);
      __wt->set_led_enabled((msg->intensity() >= 0.5));
      __led_if->set_intensity((msg->intensity() >= 0.5) ? LedInterface::ON : LedInterface::OFF);
    } else if (__led_if->msgq_first_is<LedInterface::TurnOnMessage>()) {
      __wt->set_led_enabled(true);
      __led_if->set_intensity(LedInterface::ON);
    } else if (__led_if->msgq_first_is<LedInterface::TurnOffMessage>()) {
      __wt->set_led_enabled(false);
      __led_if->set_intensity(LedInterface::OFF);
    }

    __led_if->msgq_pop();
  }
  if (write_led_if)  __led_if->write();

  __wt->wakeup();
}


bool
GripperAX12AThread::bb_interface_message_received(Interface *interface,
						 Message *message) throw()
{
    if (message->is_of_type<AX12GripperInterface::StopLeftMessage>()) {
      __wt->stop_left();
      return false; // do not enqueue StopMessage
    } else if (message->is_of_type<AX12GripperInterface::StopRightMessage>()) {
      __wt->stop_right();
      return false; // do not enqueue StopMessage
    } else if (message->is_of_type<AX12GripperInterface::StopMessage>()) {
      __wt->stop_motion();
      return false; // do not enqueue StopMessage
  } else if (message->is_of_type<AX12GripperInterface::FlushMessage>()) {
    __wt->stop_motion();
    logger->log_info(name(), "Flushing message queue");
    __gripper_if->msgq_flush();
    return false;
  } else {
    logger->log_info(name(), "Received message of type %s, enqueueing", message->type());
    return true;
  }
  return true;
}


/** @class GripperAX12AThread::WorkerThread "robotis/rx28_thread.h"
 * Worker thread for the GripperAX12AThread.
 * This continuous thread issues commands to the AX12 chain. In each loop it
 * will first execute pending operations, and then update the sensor data (lengthy
 * operation). Sensor data will only be updated while either a servo in the chain
 * is still moving or torque is disabled (so the motor can be move manually).
 * @author Tim Niemueller
 */


/** Constructor.
 * @param ptu_name name of the left/right unit
 * @param logger logger
 * @param rx28 AX12 chain
 * @param left_servo_id servo ID of the left servo
 * @param right_servo_id servo ID of the right servo
 * @param left_min minimum left in rad
 * @param left_max maximum left in rad
 * @param right_min minimum right in rad
 * @param right_max maximum right in rad
 * @param left_offset left offset from zero in servo ticks
 * @param right_offset right offset from zero in servo ticks
 */
GripperAX12AThread::WorkerThread::WorkerThread(std::string ptu_name,
					      fawkes::Logger *logger,
					      fawkes::RefPtr<RobotisAX12A> rx28,
					      unsigned char left_servo_id,
					      unsigned char right_servo_id,
					      float &left_min, float &left_max,
					      float &right_min, float &right_max,
					      float &left_offset, float &right_offset,
					      float &max_load)
  : Thread("", Thread::OPMODE_WAITFORWAKEUP)
{
  set_name("AX12WorkerThread(%s)", ptu_name.c_str());
  set_coalesce_wakeups(true);

  __logger           = logger;

  __value_rwlock     = new ReadWriteLock();
  __ax12a_rwlock     = new ReadWriteLock();
  __fresh_data_mutex = new Mutex();
  __update_waitcond  = new WaitCondition();
  __time             = new Time();

  __ax12a = rx28;
  __move_pending     = false;
  __target_left       = 0;
  __target_right      = 0;
  __left_servo_id     = left_servo_id;
  __right_servo_id    = right_servo_id;

  __left_min          = left_min;
  __left_max          = left_max;
  __right_min         = right_min;
  __right_max         = right_max;
  __left_offset       = left_offset;
  __right_offset      = right_offset;
  __enable           = false;
  __disable          = false;
  __led_enable       = false;
  __led_disable      = false;

  __max_left_speed    = __ax12a->get_max_supported_speed(__left_servo_id);
  __max_right_speed   = __ax12a->get_max_supported_speed(__right_servo_id);
  __max_load          = max_load;
}


/** Destructor. */
GripperAX12AThread::WorkerThread::~WorkerThread()
{
  delete __value_rwlock;
  delete __ax12a_rwlock;
  delete __fresh_data_mutex;
  delete __update_waitcond;
}


/** Enable or disable servo.
 * @param enabled true to enable servos, false to turn them off
 */
void
GripperAX12AThread::WorkerThread::set_enabled(bool enabled)
{
  ScopedRWLock lock(__value_rwlock);
  if (enabled) {
    __enable  = true;
    __disable = false;
  } else {
    __enable  = false;
    __disable = true;
  }
  wakeup();
}

/** Enable or disable LED.
 * @param enabled true to enable LED, false to turn it off
 */
void
GripperAX12AThread::WorkerThread::set_led_enabled(bool enabled)
{
  ScopedRWLock lock(__value_rwlock);
  if (enabled) {
    __led_enable  = true;
    __led_disable = false;
  } else {
    __led_enable  = false;
    __led_disable = true;
  }
  wakeup();
}


/** Stop currently running left motion. */
void
GripperAX12AThread::WorkerThread::stop_left()
{
  int left_ticks  = ((int)__ax12a->get_position(__left_servo_id)  - (int)RobotisAX12A::CENTER_POSITION);
  
  __target_left   = left_ticks *  RobotisAX12A::RAD_PER_POS_TICK + __left_offset;

  __move_pending = true;
  wakeup();
}

/** Stop currently running right motion. */
void
GripperAX12AThread::WorkerThread::stop_right()
{
  int right_ticks  = ((int)__ax12a->get_position(__right_servo_id)  - (int)RobotisAX12A::CENTER_POSITION);
  
  __target_right   = right_ticks *  RobotisAX12A::RAD_PER_POS_TICK + __right_offset;

  __move_pending = true;
  wakeup();
}

/** Stop currently running motion. */
void
GripperAX12AThread::WorkerThread::stop_motion()
{
  float left = 0, right = 0;
  get_gripper(left, right);
  goto_gripper(left, right);
}

/** Set move load pending. */
void
GripperAX12AThread::WorkerThread::set_move_load_pending(bool value)
{
  __move_load_pending = value;
}



/** Goto desired left/right values.
 * @param left left in radians
 * @param right right in radians
 */
void
GripperAX12AThread::WorkerThread::goto_gripper(float left, float right)
{
  ScopedRWLock lock(__value_rwlock);
  __target_left   = left;
  __target_right  = right;
  __move_pending = true;
  wakeup();
}


/** Goto desired left/right values in a specified time.
 * @param left left in radians
 * @param right right in radians
 * @param time_sec time when to reach the desired left/right values
 */
void
GripperAX12AThread::WorkerThread::goto_gripper_timed(float left, float right, float time_sec)
{
  __target_left   = left;
  __target_right  = right;
  __move_pending = true;

  float cleft=0, cright=0;
  get_gripper(cleft, cright);

  float left_diff  = fabs(left - cleft);
  float right_diff = fabs(right - cright);

  float req_left_vel  = left_diff  / time_sec;
  float req_right_vel = right_diff / time_sec;

  //__logger->log_debug(name(), "Current: %f/%f Des: %f/%f  Time: %f  Diff: %f/%f  ReqVel: %f/%f",
  //		      cleft, cright, left, right, time_sec, left_diff, right_diff, req_left_vel, req_right_vel);


  if (req_left_vel > __max_left_speed) {
    __logger->log_warn(name(), "Requested move to (%f, %f) in %f sec requires a "
		       "left speed of %f rad/s, which is greater than the maximum "
		       "of %f rad/s, reducing to max", left, right, time_sec,
		       req_left_vel, __max_left_speed);
    req_left_vel = __max_left_speed;
  }

  if (req_right_vel > __max_right_speed) {
    __logger->log_warn(name(), "Requested move to (%f, %f) in %f sec requires a "
		       "right speed of %f rad/s, which is greater than the maximum of "
		       "%f rad/s, reducing to max", left, right, time_sec,
		       req_right_vel, __max_right_speed);
    req_right_vel = __max_right_speed;
  }

  printf("set velocities from gripper timed\n");
  set_velocities(req_left_vel, req_right_vel);

  wakeup();
}

/** Set desired velocities.
 * @param left_vel left velocity
 * @param right_vel right velocity
 */
void
GripperAX12AThread::WorkerThread::set_velocities(float left_vel, float right_vel)
{
  ScopedRWLock lock(__value_rwlock);
  float left_tmp  = roundf((left_vel  / __max_left_speed)  * RobotisAX12A::MAX_SPEED);
  float right_tmp = roundf((right_vel / __max_right_speed) * RobotisAX12A::MAX_SPEED);

  // __logger->log_debug(name(), "old speed: %u/%u new speed: %f/%f", __left_vel,
  // 		      __right_vel, left_tmp, right_tmp);
  printf("old speed: %u/%u new speed: %f/%f", __left_vel,
	 __right_vel, left_tmp, right_tmp);
  if ((left_tmp >= 0) && (left_tmp <= RobotisAX12A::MAX_SPEED)) {
    __left_vel = (unsigned int)left_tmp;
    __velo_pending = true;
  } else {
    __logger->log_warn(name(), "Calculated left value out of bounds, min: 0  max: %u  des: %u",
		       RobotisAX12A::MAX_SPEED, (unsigned int)left_tmp);
  }

  if ((right_tmp >= 0) && (right_tmp <= RobotisAX12A::MAX_SPEED)) {
    __right_vel = (unsigned int)right_tmp;
    __velo_pending = true;
  } else {
    __logger->log_warn(name(), "Calculated right value out of bounds, min: 0  max: %u  des: %u",
		       RobotisAX12A::MAX_SPEED, (unsigned int)right_tmp);
  }
}


/** Get current loads.
 */
void
GripperAX12AThread::WorkerThread::get_loads(unsigned int &left, unsigned int &right)
{
  left  = __ax12a->get_load(__left_servo_id) & 0x1FF;
  right = __ax12a->get_load(__right_servo_id) & 0x1FF;
}

/** Get current velocities.
 * @param left_vel upon return contains current left velocity
 * @param right_vel upon return contains current right velocity
 */
void
GripperAX12AThread::WorkerThread::get_velocities(float &left_vel, float &right_vel)
{
  unsigned int left_velticks  = __ax12a->get_goal_speed(__left_servo_id);
  unsigned int right_velticks = __ax12a->get_goal_speed(__right_servo_id);

  left_velticks  = (unsigned int)(((float)left_velticks  / (float)RobotisAX12A::MAX_SPEED) * __max_left_speed);
  right_velticks = (unsigned int)(((float)right_velticks / (float)RobotisAX12A::MAX_SPEED) * __max_right_speed);
}


/** Set desired velocities.
 * @param left_margin left margin
 * @param right_margin right margin
 */
void
GripperAX12AThread::WorkerThread::set_margins(float left_margin, float right_margin)
{
  if (left_margin  > 0.0)  __left_margin  = left_margin;
  if (right_margin > 0.0)  __right_margin = right_margin;
  //__logger->log_warn(name(), "Margins set to %f, %f", __left_margin, __right_margin);
}


/** Get left/right value.
 * @param left upon return contains the current left value
 * @param right upon return contains the current right value
 */
void
GripperAX12AThread::WorkerThread::get_gripper(float &left, float &right)
{
  ScopedRWLock lock(__ax12a_rwlock, ScopedRWLock::LOCK_READ);

  int left_ticks  = ((int)__ax12a->get_position(__left_servo_id)  - (int)RobotisAX12A::CENTER_POSITION);
  int right_ticks = ((int)__ax12a->get_position(__right_servo_id) - (int)RobotisAX12A::CENTER_POSITION);

  left  = left_ticks *  RobotisAX12A::RAD_PER_POS_TICK + __left_offset;
  right = right_ticks * RobotisAX12A::RAD_PER_POS_TICK + __right_offset;
}


/** Get left/right value with time.
 * @param left upon return contains the current left value
 * @param right upon return contains the current right value
 * @param time upon return contains the time the left and right values were read
 */
void
GripperAX12AThread::WorkerThread::get_gripper(float &left, float &right,
                                             fawkes::Time &time)
{
  get_gripper(left, right);
  time = __gripper_time;
}


/** Check if motion is final.
 * @return true if motion is final, false otherwise
 */
bool
GripperAX12AThread::WorkerThread::is_final()
{
  float left, right;
  get_gripper(left, right);

  /*
  __logger->log_debug(name(), "P: %f  T: %f  TP: %f  TT: %f  PM: %f  TM: %f  PMov: %i  TMov: %i  Final: %s",
                      left, right, __target_left, __target_right, __left_margin, __right_margin,
                      __ax12a->is_moving(__left_servo_id), __ax12a->is_moving(__right_servo_id),
                      (( (fabs(left  - __target_left)  <= __left_margin) &&
                         (fabs(right - __target_right) <= __right_margin) ) ||
                       (! __ax12a->is_moving(__left_servo_id) &&
                        ! __ax12a->is_moving(__right_servo_id))) ? "YES" : "NO");
  */

  ScopedRWLock lock(__ax12a_rwlock, ScopedRWLock::LOCK_READ);

  return  ( (fabs(left  - __target_left)  <= __left_margin) &&
	    (fabs(right - __target_right) <= __right_margin) ) ||
          (! __ax12a->is_moving(__left_servo_id) &&
	   ! __ax12a->is_moving(__right_servo_id));
}


/** Check if Gripper is enabled.
 * @return true if torque is enabled for both servos, false otherwise
 */
bool
GripperAX12AThread::WorkerThread::is_enabled()
{
  return (__ax12a->is_torque_enabled(__left_servo_id) &&
	  __ax12a->is_torque_enabled(__right_servo_id));
}


/** Check is fresh sensor data is available.
 * Note that this method will return true at once per sensor update cycle.
 * @return true if fresh data is available, false otherwise
 */
bool
GripperAX12AThread::WorkerThread::has_fresh_data()
{
  MutexLocker lock(__fresh_data_mutex);

  bool rv = __fresh_data;
  __fresh_data = false;
  return rv;
}


void
GripperAX12AThread::WorkerThread::loop()
{
  if (__enable) {
    __value_rwlock->lock_for_write();
    __enable  = false;
    __value_rwlock->unlock();
    ScopedRWLock lock(__ax12a_rwlock);
    __ax12a->set_led_enabled(__right_servo_id, true);
    __ax12a->set_torques_enabled(true, 2, __left_servo_id, __right_servo_id);
  } else if (__disable) {
    __value_rwlock->lock_for_write();
    __disable = false;
    __value_rwlock->unlock();
    ScopedRWLock lock(__ax12a_rwlock);
    if (__led_enable || __led_disable || __velo_pending || __move_pending || __move_load_pending) usleep(3000);
  }

  if (__led_enable) {
    __value_rwlock->lock_for_write();
    __led_enable = false;
    __value_rwlock->unlock();    
    ScopedRWLock lock(__ax12a_rwlock);
    __ax12a->set_led_enabled(__left_servo_id, true);
    if (__velo_pending || __move_pending) usleep(3000);
  } else if (__led_disable) {
    __value_rwlock->lock_for_write();
    __led_disable = false;
    __value_rwlock->unlock();    
    ScopedRWLock lock(__ax12a_rwlock);
    __ax12a->set_led_enabled(__left_servo_id, false);    
    if (__velo_pending || __move_pending || __move_load_pending) usleep(3000);
  }

  if (__velo_pending) {
    __value_rwlock->lock_for_write();
    __velo_pending = false;
    unsigned int left_vel  = __left_vel;
    unsigned int right_vel = __right_vel;
    __value_rwlock->unlock();
    ScopedRWLock lock(__ax12a_rwlock);
    printf("__velo_pending! %d, %d\n", left_vel, right_vel);
    __ax12a->set_goal_speeds(2, __left_servo_id, left_vel, __right_servo_id, right_vel);
    if (__move_pending || __move_load_pending) usleep(3000);
  }

  if (__move_pending) {
    __value_rwlock->lock_for_write();
    __move_pending    = false;
    float target_left  = __target_left;
    float target_right = __target_right;
    __value_rwlock->unlock();
    // printf("trying to set: target_left: %f, target_right: %f\n", target_left, target_right);
    exec_goto_gripper(target_left, target_right);
    if (__move_load_pending) usleep(3000);
  }
  
  if (__move_load_pending) {
    stop_if_needed();
    // __move_load_pending();
  }

  try {
    ScopedRWLock lock(__ax12a_rwlock, ScopedRWLock::LOCK_READ);
    __ax12a->read_table_values(__left_servo_id);
    __ax12a->read_table_values(__right_servo_id);
    {
      MutexLocker lock_fresh_data(__fresh_data_mutex);
      __fresh_data = true;
      __gripper_time.stamp();
    }
  } catch (Exception &e) {
    // usually just a timeout, too noisy
    //__logger->log_warn(name(), "Error while reading table values from servos, exception follows");
    //__logger->log_warn(name(), e);
  }

  //if (! is_final() ||
  //    ! __ax12a->is_torque_enabled(__left_servo_id) ||
  //    ! __ax12a->is_torque_enabled(__right_servo_id)) {
    // while moving, and while the motor is off, wake us up to get new servo
    // position data
    //wakeup();
    //}

  __update_waitcond->wake_all();

  // Wakeup ourselves for faster updates
  wakeup();
}


/** Execute left/right motion.
 * @param left_rad left in rad to move to
 * @param right_rad right in rad to move to
 */
void
GripperAX12AThread::WorkerThread::exec_goto_gripper(float left_rad, float right_rad)
{
  if ( (left_rad < __left_min) || (left_rad > __left_max) ) {
    __logger->log_warn(name(), "Left value out of bounds, min: %f  max: %f  des: %f",
		       __left_min, __left_max, left_rad);
    return;
  }
  if ( (right_rad < __right_min) || (right_rad > __right_max) ) {
    __logger->log_warn(name(), "Right value out of bounds, min: %f  max: %f  des: %f",
		       __right_min, __right_max, right_rad);
    return;
  }

  unsigned int left_min = 0, left_max = 0, right_min = 0, right_max = 0;

  __ax12a->get_angle_limits(__left_servo_id, left_min, left_max);
  __ax12a->get_angle_limits(__right_servo_id, right_min, right_max);


  int left_pos  = (int)roundf(RobotisAX12A::POS_TICKS_PER_RAD * (left_rad - __left_offset))
                 + RobotisAX12A::CENTER_POSITION;
  int right_pos = (int)roundf(RobotisAX12A::POS_TICKS_PER_RAD * (right_rad - __right_offset))
                 + RobotisAX12A::CENTER_POSITION;

  if ( (left_pos < 0) || ((unsigned int)left_pos < left_min) || ((unsigned int)left_pos > left_max) ) {
    __logger->log_warn(name(), "Left position out of bounds, min: %u  max: %u  des: %i",
		       left_min, left_max, left_pos);
    return;
  }

  if ( (right_pos < 0) || ((unsigned int)right_pos < right_min) || ((unsigned int)right_pos > right_max) ) {
    __logger->log_warn(name(), "Right position out of bounds, min: %u  max: %u  des: %i",
		       right_min, right_max, right_pos);
    return;
  }

  ScopedRWLock lock(__ax12a_rwlock);
  __ax12a->goto_positions(2, __left_servo_id, left_pos, __right_servo_id, right_pos);
}

/** Execute closing gripper motion by not overriding the given load.
 * @param left_rad left in rad to move to
 * @param right_rad right in rad to move to
 * @param load max load from 0 to 1
 */
void
GripperAX12AThread::WorkerThread::goto_gripper_load(float left_rad, float right_rad)
{
  // __ax12a->goto_positions(2, __left_servo_id, left_rad, __right_servo_id, right_rad);
  __target_left  = left_rad;
  __target_right = right_rad;
  __move_load_pending = true;
  __left_servo_load_full = false;
  __right_servo_load_full = false;
  printf("goto_gripper_load: %f, %f\n", __target_left, __target_right);
  wakeup();
}


/** Stops movement of AX12 if necessary
 */
void
GripperAX12AThread::WorkerThread::stop_if_needed()
{
  printf("stop_if_needed");
  printf("load left: %d", __ax12a->get_load(__left_servo_id) & 0x1FF);
  printf(" load right: %d", __ax12a->get_load(__right_servo_id) & 0x1FF);
  printf(" desired max load: %f - %d", __max_load, ((unsigned int)__max_load * 1023));
  
  if (!__left_servo_load_full && (__ax12a->get_load(__left_servo_id) & 0x1FF) > __max_load * 1023)
    {
      printf("stop_left\n");
      __left_servo_load_full = true;
      stop_left();
    }
  if (!__right_servo_load_full && (__ax12a->get_load(__right_servo_id) & 0x1FF) > __max_load * 1023)
    {
      printf("stop_right\n");
      __right_servo_load_full = true;
      stop_right();
    }
  
  __move_load_pending = !(__left_servo_load_full && __right_servo_load_full);
  printf(" move load pending: %d\n", __move_load_pending);
}

// /** Is move until load pending?
//  */
// bool
// GripperAX12AThread::WorkerThread::is_move_load_pending()
// {
//   return move_load_pending;
// }

/** Wait for fresh data to be received.
 * Blocks the calling thread.
 */
void
GripperAX12AThread::WorkerThread::wait_for_fresh_data()
{
  __update_waitcond->wait();
}
