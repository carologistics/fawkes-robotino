/***************************************************************************
 *  machine_signal_thread.cpp - Detect signals using color thresholds
 *
 *  Copyright  2014 Victor Mataré
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

#include "machine_signal_thread.h"
#include <fvfilters/colorthreshold.h>
#include <aspect/logging.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/scalers/lossy.h>

using namespace fawkes;
using namespace firevision;

MachineSignalThread::MachineSignalThread()
    : Thread("PluginTemplateThread", Thread::OPMODE_WAITFORWAKEUP),
      VisionAspect(VisionAspect::CYCLIC)
{
  _filter_color_thresh = NULL;
  _camera = NULL;
  _scaled_buf = NULL;
  _filtered_buf = NULL;
}

void MachineSignalThread::init()
{
  _camera = vision_master->register_for_camera(
      config->get_string((_cfg_prefix + "/camera").c_str()).c_str(), this);
  _scaled_buf = new SharedMemoryImageBuffer("signal", YUV422_PLANAR, _resolution_x,
      _resolution_y);
  _filtered_buf = new SharedMemoryImageBuffer("signal_filtered", YUV422_PLANAR, _resolution_x,
      _resolution_y);


  std::vector<uint> cfg_ref_color = config->get_uints(
      (_cfg_prefix + "/reference_color").c_str());
  RGB_t ref_color;
  ref_color.R = (uint8_t) cfg_ref_color.at(0);
  ref_color.G = (uint8_t) cfg_ref_color.at(1);
  ref_color.B = (uint8_t) cfg_ref_color.at(2);
  _filter_color_thresh = new FilterColorThreshold(ref_color, 10, 10);
}

/*
bool MachineSignalThread::prepare_finalize_user()
{
  return true;
}//*/

void MachineSignalThread::finalize()
{
  vision_master->unregister_thread(this);
  delete _scaled_buf;
  delete _filter_color_thresh;
}

void MachineSignalThread::loop()
{
  unsigned char *scaled_buffer = _scaled_buf->buffer();
  LossyScaler *scaler = new LossyScaler();

  _camera->capture();
  scaler->set_original_dimensions(_camera->pixel_width(), _camera->pixel_height());
  scaler->set_scaled_dimensions(_scaled_buf->width(), _scaled_buf->height());
  scaler->set_original_buffer(_camera->buffer());
  scaler->set_scaled_buffer(scaled_buffer);
  scaler->scale();
  _camera->dispose_buffer();
  free(scaler);

  _filter_color_thresh->set_src_buffer(scaled_buffer,
      ROI::full_image(_resolution_x, _resolution_y));
  _filter_color_thresh->set_dst_buffer(_filtered_buf->buffer(),
      ROI::full_image(_resolution_x, _resolution_y));
  _filter_color_thresh->apply();
}

