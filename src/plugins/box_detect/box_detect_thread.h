/***************************************************************************
 *  box_detect_thread.hpp - box_detect
 *
 *  Plugin created: May 30 21:54:46 2023
 * 
 *  Copyright  2023 Daniel Honies
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

#ifndef _PLUGINS_BOX_DETECT_BOX_DETECT_THREAD_H_
#define _PLUGINS_BOX_DETECT_BOX_DETECT_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/thread.h>
#include <interfaces/SwitchInterface.h>
#include <interfaces/TransformInterface.h>
#include <plugins/ros2/aspect/ros2.h>

#include <vector>
// config handling
#include <config/change_handler.h>
#include <interfaces/BoxInterface.h>

#include <std_srvs/srv/set_bool.hpp>

namespace fawkes {
}
class BoxDetectThread : public fawkes::Thread,
                        public fawkes::ClockAspect,
                        public fawkes::LoggingAspect,
                        public fawkes::ConfigurableAspect,
                        public fawkes::BlackBoardAspect,
                        public fawkes::TransformAspect,
                        public fawkes::BlackBoardInterfaceObserver,
                        public fawkes::BlackBoardInterfaceListener,
                        public fawkes::ROS2Aspect
{
public:
	BoxDetectThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	// for BlackBoardInterfaceObserver
	virtual void bb_interface_created(const char *type, const char *id) throw();

	// for BlackBoardInterfaceListener
	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) noexcept;
	virtual void bb_interface_writer_removed(fawkes::Interface *interface,
	                                         fawkes::Uuid       instance_serial) noexcept;
	virtual void bb_interface_reader_removed(fawkes::Interface *interface,
	                                         fawkes::Uuid       instance_serial) noexcept;

protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	std::vector<fawkes::BoxInterface *>    *box_detect_ifs_;
	std::list<fawkes::TransformInterface *> tfifs_;
	void                                    conditional_close(fawkes::Interface *interface) throw();
	fawkes::SwitchInterface                *switch_if_;
	bool                                    enabled_;
	rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};
#endif