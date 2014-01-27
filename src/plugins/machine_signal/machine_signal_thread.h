
/***************************************************************************
 *  machine_signal_thread.h - Detect signals using color thresholds
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

#ifndef __PLUGINS_MACHINE_SIGNAL_THREAD_H_
#define __PLUGINS_MACHINE_SIGNAL_THREAD_H_

// Superclasses/Aspects
#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/vision.h>

// Members
#include <fvutils/ipc/shm_image.h>
#include <fvfilters/colorthreshold.h>
#include <fvcams/fileloader.h>
#include <string>

namespace fawkes
{
class Position3DInterface;
class RobotinoLightInterface;
namespace tf
{
class TransformListener;
}
}

class MachineSignalThread :
  public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::VisionAspect
{

  private:
    uint32_t _resolution_x = 320;
    uint32_t _resolution_y = 240;

    std::string const _cfg_prefix = "/plugins/machine_signal";

    firevision::Camera *_camera;
    firevision::SharedMemoryImageBuffer *_scaled_buf;
    firevision::SharedMemoryImageBuffer *_filtered_buf;
    firevision::FilterColorThreshold *_filter_color_thresh;

  public:
    MachineSignalThread();

    virtual void init();
    virtual void loop();
//    virtual bool prepare_finalize_user();
    virtual void finalize();

  protected:
    /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
    virtual void run()
    {
      Thread::run();
    }
};

#endif
