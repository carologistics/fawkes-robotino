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

#include "tensorflow_thread.h"
#include "interfaces/TensorflowInterface.h"
#include <unistd.h>

#include "image_file_loader.h"
#include "image_shm_loader.h"
#include "image_v4l2_loader.h"

using namespace fawkes;

/** @class TensorflowThread
 * Thread to use tensorflow in fawkes
 * @author Morian Sonnet
 */

/** Constructor.
 * @param cfg_name Prefix for the config loading
 */
TensorflowThread::TensorflowThread(std::string cfg_name)
    : Thread("TensorflowThread", Thread::OPMODE_WAITFORWAKEUP),
      BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
      BlackBoardInterfaceListener("TensorflowThread(%s)", cfg_name.c_str()),
      cfg_name_(cfg_name), graph_(nullptr), source_(nullptr) {}

void TensorflowThread::init() {
  tensorflow_if_ = blackboard->open_for_writing<TensorflowInterface>(
      "Tensorflow", cfg_name_.c_str());

  bbil_add_message_interface(tensorflow_if_);
  blackboard->register_listener(this);

  load_config();
}

void TensorflowThread::loop() {
  tensorflow_if_->read();
  while (!tensorflow_if_->msgq_empty()) {
    if (tensorflow_if_
            ->msgq_first_is<TensorflowInterface::LoadGraphFileMessage>()) {
      TensorflowInterface::LoadGraphFileMessage *msg =
          tensorflow_if_->msgq_first(msg);
      this->graph_input_node_ = std::string(msg->input_node());
      this->graph_output_node_ = std::string(msg->output_node());
      this->load_graph(std::string(msg->file_path()));

    } else if (tensorflow_if_->msgq_first_is<
                   TensorflowInterface::SetSourceImageSHMMessage>()) {
      TensorflowInterface::SetSourceImageSHMMessage *msg =
          tensorflow_if_->msgq_first(msg);
      this->set_source_image_shm(
          std::string(msg->shm_id()), std::string(msg->hostname()),
          msg->is_normalize(), msg->norm_mean(), msg->norm_std(), msg->width(),
          msg->height(), msg->image_dtype());

    } else if (tensorflow_if_
                   ->msgq_first_is<TensorflowInterface::TriggerRunMessage>()) {
      TensorflowInterface::TriggerRunMessage *msg =
          tensorflow_if_->msgq_first(msg);
      this->run_graph_once(msg->id());

    } else {
      logger->log_error(name(), "Unhandled message");
    }
    tensorflow_if_->msgq_pop();
  }
}

void TensorflowThread::load_graph(std::string file_name) {
  this->delete_graph();
  graph_ = tf_utils::LoadGraph(file_name.c_str());

  if (graph_ == nullptr) {
    logger->log_error(name(), "Could not import the graph");
  }

  this->graph_file_name_ = file_name;
}

void TensorflowThread::delete_graph() {
  if (graph_ != nullptr)
    TF_DeleteGraph(graph_);
}

void TensorflowThread::finalize() { this->delete_graph(); }

void TensorflowThread::load_config() {
  logger->log_info(name(), "load config");
}

void TensorflowThread::set_source_image_shm(std::string shm_id,
                                            std::string their_hostname,
                                            bool normalize, double norm_mean,
                                            double norm_std, unsigned int width,
                                            unsigned int height,
                                            unsigned int image_dtype) {
  // first step: check hostname
  char *my_hostname = new char[256];
  if (gethostname(my_hostname, 256) != 0) {
    logger->log_error(name(), "Error while fetching hostname");
  }
  if (their_hostname.compare(std::string(my_hostname)) !=
      0) { // the SHM should reside on the host on which this plugin runs
    logger->log_error(name(), "SHM resides not on my host!");
  }

  source_ = new TF_Plugin_Image_SHM_Loader(
      std::string(name()), logger, shm_id,
      (firevision::colorspace_t)image_dtype, width, height, normalize,
      norm_mean, norm_std);
  if (!source_->verify()) {
    source_ = nullptr;
  }
}

void TensorflowThread::run_graph_once(unsigned int msg_id) {
  if (this->graph_ == nullptr) {
    logger->log_error(name(), "Graph was not initialized");
    return;
  }
  if (this->source_ == nullptr) {
    logger->log_error(name(), "Source was not initialized");
    return;
  }
  TF_Output input_op = {
      TF_GraphOperationByName(graph_, graph_input_node_.c_str()), 0};
  TF_Output output_op = {
      TF_GraphOperationByName(graph_, graph_input_node_.c_str()), 0};

  if (input_op.oper == nullptr) {
    logger->log_error(name(), "Cannot init input_op");
  }
  if (output_op.oper == nullptr) {
    logger->log_error(name(), "Cannot init output_op");
  }

  const void *input_vals = source_->read();
  int64_t ndim[3] = {1, 2, 3};
  TF_Tensor *input_tensor =
      tf_utils::CreateTensor(TF_FLOAT, ndim, 3, input_vals, 400);
  source_->post_read();

  if (input_tensor == nullptr) {
    logger->log_error(name(), "Creating input tensor went wrong");
    return;
  }

  TF_Tensor *output_tensor = nullptr;

  TF_Session *sess = tf_utils::CreateSession(graph_);
  if (sess == nullptr) {
    logger->log_error(name(), "Creating session was not successful");
    return;
  }

  TF_Code session_run_code = tf_utils::RunSession(
      sess, &input_op, &input_tensor, 1, &output_op, &output_tensor, 1);
  if (session_run_code != TF_OK) {
    logger->log_error(name(), "Running session was not successful");
    return;
  }

  tf_utils::DeleteSession(sess);
  tf_utils::DeleteTensor(input_tensor);
  tf_utils::DeleteTensor(output_tensor);
}
