/***************************************************************************
 *  tensorflow_thread.h - Thread to use tensorflow in Fawkes
 *
 *  Created: Thu May 2 10:31:00 2019
 *  Copyright  2019    Morian Sonnet
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

#ifndef __PLUGINS_TENSORFLOW_THREAD_H_
#define __PLUGINS_TENSORFLOW_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/vision.h>
#include <blackboard/interface_listener.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <string>

#include <interfaces/TensorflowInterface.h>

// config handling
#include <config/change_handler.h>

#include "tf_utils.h"

#include "loader.h"
#include "outputter.h"

namespace fawkes {
class TensorflowInterface;
}

class TensorflowThread : public fawkes::Thread,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::BlockedTimingAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::BlackBoardInterfaceListener,
                         // public fawkes::VisionAspect,
                         // public fawkes::ConfigurationChangeHandler,
                         public fawkes::ClockAspect {
public:
  /** Constructor */
  TensorflowThread(std::string cfg_name);

  /** Destructor */
  virtual ~TensorflowThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

private:
  std::string cfg_name_;
  TF_Graph *graph_;
  TF_Plugin_Loader *source_;
  TF_Plugin_Outputter *output_;

  std::string graph_file_name_;
  std::string graph_input_node_;
  std::string graph_output_node_;

  fawkes::TensorflowInterface *tensorflow_if_;

  void load_config();
  void load_graph(std::string);
  void set_source_image_shm(std::string shm_id, std::string their_hostname,
                            bool normalize, double norm_mean, double norm_std,
                            unsigned int width, unsigned int height,
                            unsigned int image_dtype);
  void set_source_image_file(std::string file_name, bool normalize,
                             double norm_mean, double norm_std,
                             unsigned int width, unsigned int height,
                             unsigned int image_dtype);
  void set_source_image_v4l2(std::string device_name, bool normalize,
                             double norm_mean, double norm_std,
                             unsigned int width, unsigned int height,
                             unsigned int image_dtype);

  void pre_set_output();
  void post_set_output();

  void pre_set_source();
  void post_set_source();
  void run_graph_once(unsigned int msg_id);
  void delete_graph();
};

#endif
