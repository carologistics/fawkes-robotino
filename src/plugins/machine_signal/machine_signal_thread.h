
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
#include <fvfilters/colorthreshold.h>
#include <fvmodels/color/similarity.h>
#include <fvmodels/color/thresholds_luminance.h>
#include <fvclassifiers/simple.h>
//#include <fvcams/fileloader.h>
#include <fvcams/camera.h>
#include <string>
#include <core/threading/mutex.h>
#include <atomic>
#include <interfaces/RobotinoLightInterface.h>


#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

#define MAX_ROI_ASPECT_SKEW 1.5f

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
        firevision::ColorModelSimilarity *colormodel;
        firevision::SimpleColorClassifier *classifier;
        firevision::color_t color_expect;
        std::vector<unsigned int> cfg_ref_col;
        int cfg_chroma_thresh;
        int cfg_sat_thresh;
        unsigned int cfg_roi_min_points;
        unsigned int cfg_roi_basic_size;
        unsigned int cfg_roi_neighborhood_min_match;
        unsigned int cfg_roi_grow_by;
    } color_classifier_t_;

    struct sort_functor_ {
        bool operator() (firevision::ROI r1, firevision::ROI r2) {
          return r1.start.x <= r2.start.x;
        }
    } sort_rois_by_x_;

    typedef struct {
        // keep ROIs for a certain signal in a list, as well as in explicit struct members
        firevision::ROI *red_roi;
        firevision::ROI *yellow_roi;
        firevision::ROI *green_roi;
    } signal_rois_t_;

    std::list<firevision::ROI> all_rois_;

    inline bool rois_delivery_zone(std::list<firevision::ROI>::iterator red, std::list<firevision::ROI>::iterator green);
    inline bool roi_width_ok(std::list<firevision::ROI>::iterator);
    inline bool rois_similar_width(std::list<firevision::ROI>::iterator r1, std::list<firevision::ROI>::iterator r2);
    inline bool rois_x_aligned(std::list<firevision::ROI>::iterator r1, std::list<firevision::ROI>::iterator r2);
    inline bool roi_aspect_ok(std::list<firevision::ROI>::iterator);

    std::atomic_bool cfg_changed_;
    std::atomic_bool cam_changed_;
    firevision::SharedMemoryImageBuffer *shmbuf_;

    std::string cfg_camera_;

    /*firevision::SimpleColorClassifier *cls_black_cap_;
    unsigned int cfg_black_thresh_;*/

    color_classifier_t_ cls_red_;
    color_classifier_t_ cls_green_;

    firevision::Camera *camera_;

    unsigned int cam_width_, cam_height_;

    fawkes::Mutex cfg_mutex_;

    fawkes::RobotinoLightInterface::LightState *iface_red_;
    fawkes::RobotinoLightInterface::LightState *iface_yellow_;
    fawkes::RobotinoLightInterface::LightState *iface_green_;

    // Abstracts inherited from ConfigurationChangeHandler
    virtual void config_tag_changed(const char *new_tag);
    virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);
    virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
    virtual void config_value_erased(const char *path);

    template<typename T>
    bool test_set_cfg_value(T *cfg, T val) {
      if (*cfg == val) return false;
      *cfg = val;
      return true;
    }


    void setup_classifier(color_classifier_t_ *classifier);
    void setup_camera();

    void cleanup_classifiers();
    void cleanup_camera();

    std::list<signal_rois_t_> *create_signal_rois(
        std::list<firevision::ROI> *rois_R,
        std::list<firevision::ROI> *rois_G);

  protected:
    /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
    virtual void run()
    {
      Thread::run();
    }
};

#endif
