
/***************************************************************************
 *  rx28_thread.h - RX28 pan/tilt unit act thread
 *
 *  Created: Sat Feb 28 10:29:38 2015
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

#ifndef __PLUGINS_GRIPPER_ROBOTIS_RX28_THREAD_H_
#define __PLUGINS_GRIPPER_ROBOTIS_RX28_THREAD_H_

#include "../act_thread.h"

#ifdef HAVE_TF
#  include <aspect/tf.h>
#endif
#include <aspect/clock.h>
#include <blackboard/interface_listener.h>
#include <utils/time/time.h>

#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <string>
#include <memory>

namespace fawkes {
  class AX12GripperInterface;
  class LedInterface;
  class JointInterface;
  class ReadWriteLock;
  class WaitCondition;
}

class RobotisAX12A;

class GripperAX12AThread
: public AX12GripperActThread,
#ifdef HAVE_TF
  public fawkes::TransformAspect,
#endif
  public fawkes::BlackBoardInterfaceListener
{
 public:
  GripperAX12AThread(std::string &gripper_cfg_prefix);

  virtual void init();
  virtual bool prepare_finalize_user();
  virtual void finalize();
  virtual void loop();

  // For BlackBoardInterfaceListener
  virtual bool bb_interface_message_received(fawkes::Interface *interface,
					     fawkes::Message *message) throw();

  void update_sensor_values();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::AX12GripperInterface *__gripper_if;
  fawkes::LedInterface         *__led_if;
  fawkes::JointInterface       *__leftjoint_if;
  fawkes::JointInterface       *__rightjoint_if;

  fawkes::RefPtr<RobotisAX12A> __ax12a;
  std::string   __gripper_cfg_prefix;
  std::string   __ptu_cfg_prefix;
  std::string   __ptu_name;
  std::string   __cfg_device;
  unsigned char __cfg_left_servo_id;
  unsigned char __cfg_right_servo_id;
  /* unsigned char __cfg_left_servo_id; */
  /* unsigned char __cfg_right_servo_id; */
  unsigned int  __cfg_read_timeout_ms;
  unsigned int  __cfg_disc_timeout_ms;
  bool          __cfg_goto_zero_start;
  bool          __cfg_turn_off;
  unsigned int __cfg_cw_compl_margin;
  unsigned int __cfg_ccw_compl_margin;
  unsigned int __cfg_cw_compl_slope;
  unsigned int __cfg_ccw_compl_slope;
  float        __cfg_left_min;
  float        __cfg_left_max;
  float        __cfg_right_min;
  float        __cfg_right_max;
  float        __cfg_left_margin;
  float        __cfg_right_margin;
  float        __cfg_left_offset;
  float        __cfg_right_offset;
  float        __cfg_left_start;
  float        __cfg_right_start;
  float        __cfg_left_torque;
  float        __cfg_right_torque;
  float        __cfg_left_open_angle;
  float        __cfg_left_close_angle;
  float        __cfg_left_close_load_angle;
  float        __cfg_right_open_angle;
  float        __cfg_right_close_angle;
  float        __cfg_right_close_load_angle;
  float        __cfg_max_speed;

#ifdef HAVE_TF
  std::string  __cfg_base_frame;
  std::string  __cfg_left_link;
  std::string  __cfg_right_link;

  fawkes::tf::Vector3  __translation_left;
  fawkes::tf::Vector3  __translation_right;

  bool          __cfg_publish_transforms;
#endif

  float         __last_left;
  float         __last_right;

  class WorkerThread : public fawkes::Thread
  {
  public:
    WorkerThread(std::string ptu_name, fawkes::Logger *logger,
		 fawkes::RefPtr<RobotisAX12A> rx28,
		 unsigned char left_servo_id, unsigned char right_servo_id,
		 float &left_min, float &left_max, float &right_min, float &right_max,
		 float &left_offset, float &right_offset);

    ~WorkerThread();
    void goto_gripper(float left, float right);
    void goto_gripper_timed(float left, float right, float time_sec);
    void get_gripper(float &left, float &right);
    void get_gripper(float &left, float &right, fawkes::Time &time);
    void set_velocities(float left_vel, float right_vel);
    void get_velocities(float &left_vel, float &right_vel);
    void get_loads(unsigned int &left, unsigned int &right);
    void set_margins(float left_margin, float right_margin);
    bool is_final();
    bool is_enabled();
    void set_enabled(bool enabled);
    void set_led_enabled(bool enabled);
    void stop_motion();
    bool has_fresh_data();
    void wait_for_fresh_data();
    void stop_left();
    void stop_right();
    void set_servo_angle(unsigned int servo_id, float servo_angle);

    virtual void loop();

    /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
    protected: virtual void run() { Thread::run(); }

  private:
    void exec_goto_gripper(float left, float right);
    
  private:
    fawkes::ReadWriteLock       *__ax12a_rwlock;
    fawkes::RefPtr<RobotisAX12A>  __ax12a;
    fawkes::Logger              *__logger;
    fawkes::WaitCondition       *__update_waitcond;
    fawkes::Time                *__time;

    unsigned char __left_servo_id;
    unsigned char __right_servo_id;

    float         __left_min;
    float         __left_max;
    float         __right_min;
    float         __right_max;
    float         __left_offset;
    float         __right_offset;
    float         __max_left_speed;
    float         __max_right_speed;
    float         __left_margin;
    float         __right_margin;
    
    fawkes::ReadWriteLock *__value_rwlock;
    bool  __move_pending;
    bool  __set_servo_angle_pending;
    int   __set_servo_angle_id;
    unsigned int __set_servo_angle_position;
    float __target_left;
    float __target_right;
    bool  __enable;
    bool  __disable;
    bool  __velo_pending;
    unsigned int __left_vel;
    unsigned int __right_vel;
    bool  __led_enable;
    bool  __led_disable;
    fawkes::Time  __gripper_time;

    bool __fresh_data;
    fawkes::Mutex *__fresh_data_mutex;

  };

  WorkerThread *__wt;
};

#endif
