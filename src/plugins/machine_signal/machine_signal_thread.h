
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
#include <config/change_handler.h>

// Members
#include <fvutils/ipc/shm_image.h>
//#include <fvutils/base/types.h>
#include <fvfilters/colorthreshold.h>
#include <fvmodels/color/similarity.h>
#include <fvclassifiers/simple.h>
#include <fvcams/fileloader.h>
#include <string>

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

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
  public fawkes::VisionAspect,
  public fawkes::ConfigurationChangeHandler
{
  public:
    MachineSignalThread();

    virtual void init();
    virtual void loop();
    virtual void finalize();

  private:
    typedef struct {
        firevision::SharedMemoryImageBuffer *shmbuf;
        firevision::FilterColorThreshold *filter;
        firevision::ColorModelSimilarity *colormodel;
        firevision::SimpleColorClassifier *classifier;
        firevision::color_t result;
        std::vector<unsigned int> cfg_ref_col;
        int cfg_chroma_thresh;
        int cfg_sat_thresh;
        unsigned int cfg_roi_min_points;
        unsigned int cfg_roi_basic_size;
        unsigned int cfg_roi_neighborhood_min_match;
        unsigned int cfg_roi_grow_by;
    } color_classifier_t_;

    volatile bool cfg_changed_;

    color_classifier_t_ cls_red_;
    color_classifier_t_ cls_green_;

    firevision::Camera *camera_;
    firevision::SharedMemoryImageBuffer *shmbuf_cam_;

    unsigned int width_, height_;

    // Abstracts inherited from ConfigurationChangeHandler
    virtual void config_tag_changed(const char *new_tag);
    virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);
    virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
    virtual void config_value_erased(const char *path);

    template<typename T>
    void test_set_cfg_value(T *cfg, T val) {
      if (*cfg == val) return;
      *cfg = val;
      cfg_changed_ = true;
    }


    void setup_classifier(color_classifier_t_ *classifier);

    void cleanup_classifiers();

  protected:
    /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
    virtual void run()
    {
      Thread::run();
    }
};

#endif
