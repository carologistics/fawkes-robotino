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
      cfg_name_(cfg_name) {}

void TensorflowThread::init() { load_config(); }

void TensorflowThread::loop() {}

void TensorflowThread::finalize() {}

void TensorflowThread::load_config() {
  logger->log_info(name(), "load config");
}
