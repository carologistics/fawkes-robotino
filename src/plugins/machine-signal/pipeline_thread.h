
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
#include <aspect/tf.h>
#include <config/change_handler.h>
#include <aspect/clock.h>

// Members
#include <fvutils/ipc/shm_image.h>
#include <fvfilters/colorthreshold.h>
#include <fvfilters/roidraw.h>
#include <fvmodels/color/similarity.h>
#include <fvmodels/color/thresholds_luminance.h>
#include <fvmodels/color/thresholds_black.h>
#include <fvmodels/scanlines/grid.h>
#include <fvclassifiers/simple.h>
#include <fvcams/v4l2.h>
#include <string>
#include <core/threading/mutex.h>
#include <atomic>
#include <set>
#include <interfaces/SwitchInterface.h>
#include <interfaces/Position3DInterface.h>
#include <fvmodels/relative_position/position_to_pixel.h>
#include <utils/time/wait.h>

#include "state.h"

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)


#define MAX_SIGNALS 3
#define TRACKED_SIGNALS MAX_SIGNALS + 2
#define CFG_PREFIX "/plugins/machine_signal"


namespace fawkes
{
class Position3DInterface;
class RobotinoLightInterface;
namespace tf
{
class TransformListener;
}
}

class MachineSignalPipelineThread :
  public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::VisionAspect,
  public fawkes::ConfigurationChangeHandler,
  public fawkes::ClockAspect,
  public fawkes::TransformAspect
{
  public:
    MachineSignalPipelineThread();
    ~MachineSignalPipelineThread();

    virtual void init();
    virtual void loop();
    virtual void finalize();

    bool lock_if_new_data();
    bool get_delivery_mode();

    void unlock();

    std::list<SignalState> &get_known_signals();
    std::list<SignalState>::iterator get_best_signal();

    /** Check if pipeline thread is enabled.
     * @return true if pipeline thread is enabled, false otherwise. */
    bool is_enabled() const { return cfg_enable_switch_; }

  private:

    bool cfg_delivery_mode_;
    
    bool bb_switch_is_enabled(fawkes::SwitchInterface *sw);

    bool cfg_enable_switch_;

    fawkes::Mutex data_mutex_;
    bool new_data_;
    double desired_frametime_;
    fawkes::TimeWait *time_wait_;

    fawkes::SwitchInterface *bb_enable_switch_;
    fawkes::SwitchInterface *bb_delivery_switch_;

    typedef struct {
        firevision::ColorModelSimilarity *colormodel;
        firevision::SimpleColorClassifier *classifier;
        std::vector<firevision::ColorModelSimilarity::color_class_t *> color_class;
        firevision::ScanlineGrid *scanline_grid;
        std::vector<unsigned int> cfg_ref_col;
        std::vector<int> cfg_chroma_thresh;
        std::vector<int> cfg_sat_thresh;
        std::vector<int> cfg_luma_thresh;
        firevision::color_t color_expect;
        unsigned int cfg_roi_min_points;
        unsigned int cfg_roi_basic_size;
        unsigned int cfg_roi_neighborhood_min_match;
        unsigned int cfg_scangrid_x_offset;
        unsigned int cfg_scangrid_y_offset;
    } color_classifier_context_t_;

    color_classifier_context_t_ cfy_ctxt_red_;
    color_classifier_context_t_ cfy_ctxt_red_delivery_;
    color_classifier_context_t_ cfy_ctxt_green_;

    std::string cfg_camera_;
    firevision::Camera *camera_;
    unsigned int cam_width_, cam_height_;

    unsigned int cfg_fps_;

    class WorldROI : public firevision::ROI {
      public:
        std::shared_ptr<fawkes::tf::Stamped<fawkes::tf::Point>> world_pos;
        WorldROI() : ROI() {}
    };

    fawkes::Position3DInterface *bb_laser_clusters_[3];
    std::atomic<unsigned int> cfg_lasercluster_min_vis_hist_;
    std::string cfg_lasercluster_frame_;
    std::atomic<float> cfg_lasercluster_signal_radius_;
    std::atomic<float> cfg_lasercluster_signal_top_;
    std::atomic<float> cfg_lasercluster_signal_bottom_;

    float cfg_cam_aperture_x_;
    float cfg_cam_aperture_y_;
    float cfg_cam_angle_y_;
    std::string cfg_cam_frame_;
    firevision::PositionToPixel *pos2pixel_;

    std::atomic<unsigned int> cfg_roi_max_height_;
    std::atomic<unsigned int> cfg_roi_max_width_;
    std::atomic<float> cfg_roi_max_aspect_ratio_;
    std::atomic<float> cfg_roi_max_width_ratio_;
    std::atomic<float> cfg_roi_xalign_;
    std::atomic<unsigned int> cfg_roi_green_horizon;
    std::atomic_bool cfg_tuning_mode_;
    std::atomic_bool cfg_draw_processed_rois_;
    std::atomic<float> cfg_max_jitter_;
    std::atomic_bool cfg_debug_processing_;
    std::atomic_bool cfg_debug_blink_;

    std::string debug_proc_string_;

    fawkes::Mutex cfg_mutex_;
    std::atomic_bool cfg_changed_,
                     cam_changed_;

    fawkes::Time *last_second_;
    unsigned int buflen_;

    // Maybe we want to detect the black cap on top of the signal, too...?
    /*firevision::SimpleColorClassifier *cls_black_cap_;
    unsigned int cfg_black_thresh_;*/

    unsigned int cfg_light_on_threshold_;
    unsigned int cfg_light_on_min_points_;
    unsigned int cfg_light_on_min_neighborhood_;
    std::atomic<float> cfg_light_on_min_area_cover_;
    firevision::SimpleColorClassifier *light_classifier_;
    firevision::ColorModelLuminance *light_colormodel_;
    firevision::ScanlineGrid *light_scangrid_;

    unsigned int cfg_black_y_thresh_;
    unsigned int cfg_black_u_thresh_;
    unsigned int cfg_black_v_thresh_;
    unsigned int cfg_black_u_ref_;
    unsigned int cfg_black_v_ref_;
    unsigned int cfg_black_min_points_;
    unsigned int cfg_black_min_neighborhood_;
    firevision::SimpleColorClassifier *black_classifier_;
    firevision::ColorModelBlack *black_colormodel_;
    firevision::ScanlineGrid *black_scangrid_;

    firevision::FilterROIDraw *roi_drawer_;
    firevision::SharedMemoryImageBuffer *shmbuf_;
    firevision::SharedMemoryImageBuffer *shmbuf_cam_;
    firevision::FilterColorThreshold *color_filter_;
    firevision::ColorModelSimilarity *combined_colormodel_;

    void setup_color_classifier(color_classifier_context_t_ *classifier);
    void setup_camera();
    bool color_data_consistent(color_classifier_context_t_ *);
    void reinit_color_config();

    //*/
    struct compare_rois_by_x_ {
        bool operator() (firevision::ROI const &r1, firevision::ROI const &r2) {
          return r1.start.x <= r2.start.x;
        }
    }; // sort_rois_by_x_; //*/

    struct compare_rois_by_y_ {
        bool operator() (firevision::ROI const &r1, firevision::ROI const &r2) {
          return r1.start.y <= r2.start.y;
        }
    } sort_rois_by_y_;

    /*struct compare_signal_rois_by_x_ {
        bool operator() (SignalState::signal_rois_t_ const &signal1, SignalState::signal_rois_t_ const &signal2) {
          return signal1.red_roi->start.x <= signal2.red_roi->start.x;
        }
    } sort_signal_rois_by_x_; //*/

    // All ROIs we want to see painted in the tuning buffer
    std::list<firevision::ROI> drawn_rois_;

    std::list<SignalState::signal_rois_t_> *create_field_signals(
        std::list<firevision::ROI> *rois_R,
        std::list<firevision::ROI> *rois_G);

    std::list<SignalState::signal_rois_t_> *create_delivery_signals(std::list<firevision::ROI> *rois_R);
    std::list<SignalState::signal_rois_t_> *create_laser_signals(std::list<firevision::ROI> *rois_R);


    // Checks to weed out implausible ROIs
    inline bool rois_delivery_zone(firevision::ROI &red, firevision::ROI &green);
    inline bool roi_width_ok(firevision::ROI &r);
    inline bool rois_similar_width(firevision::ROI &r1, firevision::ROI &r2);
    inline bool rois_x_aligned(firevision::ROI &r1, firevision::ROI &r2);
    inline bool roi_aspect_ok(firevision::ROI &r);
    inline bool rois_vspace_ok(firevision::ROI &r1, firevision::ROI &r2);
    inline bool roi1_oversize(firevision::ROI &r1, firevision::ROI &r2);
    inline bool roi1_x_overlaps_below(firevision::ROI &r1, firevision::ROI &r2);
    inline bool roi1_x_intersects(firevision::ROI &r1, firevision::ROI &r2);


    //*/
    struct compare_rois_by_area_ {
        bool operator() (firevision::ROI const &r1, firevision::ROI const &r2) {
          unsigned int a1 = r1.width * r1.height;
          unsigned int a2 = r2.width * r2.height;
          return a1 >= a2;
        }
    }; //sort_rois_by_area_;

    struct compare_signal_states_by_visibility_ {
        bool operator() (SignalState const &s1, SignalState const &s2) {
          return s1.visibility > s2.visibility;
        }
    } sort_signal_states_by_visibility_;

    struct compare_signal_states_by_x_ {
        bool operator() (SignalState const &s1, SignalState const &s2) {
          return s1.pos.x <= s2.pos.x;
        }
    } sort_signal_states_by_x_;

    struct compare_signal_states_by_area_ {
        bool operator() (SignalState const &s1, SignalState const &s2) {
          if ((s1.visibility < 0) == (s2.visibility < 0)) {
            float size_ratio = (float)s1.area / (float)s2.area;
            if (size_ratio < 1.5 && size_ratio > 0.67)
              return s1.pos.x <= s2.pos.x;
            else return s1.area > s2.area;
          }
          else return s1.visibility > 0;
        }
    } sort_signal_states_by_area_;

    bool get_light_state(firevision::ROI *light);

    std::list<SignalState> known_signals_;
    std::list<SignalState>::iterator best_signal_;


    std::set<WorldROI, compare_rois_by_area_> *bb_get_laser_rois();
    std::set<WorldROI, compare_rois_by_area_> *cluster_rois_;


    // Implemented abstracts inherited from ConfigurationChangeHandler
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

  protected:
    /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
    virtual void run()
    {
      Thread::run();
    }
};

#endif
