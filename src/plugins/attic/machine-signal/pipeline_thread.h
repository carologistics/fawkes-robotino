
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
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/vision.h>
#include <config/change_handler.h>
#include <core/threading/thread.h>

// Members
#include "custom_rois.h"
#include "state.h"

#include <core/threading/mutex.h>
#include <fvcams/v4l2.h>
#include <fvclassifiers/simple.h>
#include <fvfilters/colorthreshold.h>
#include <fvfilters/roidraw.h>
#include <fvmodels/color/similarity.h>
#include <fvmodels/color/thresholds_black.h>
#include <fvmodels/color/thresholds_luminance.h>
#include <fvmodels/relative_position/position_to_pixel.h>
#include <fvmodels/scanlines/grid.h>
#include <fvutils/ipc/shm_image.h>
#include <interfaces/LaserLineInterface.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SignalHintInterface.h>
#include <interfaces/SwitchInterface.h>
#include <utils/time/wait.h>

#include <atomic>
#include <set>
#include <string>

#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

#define MAX_SIGNALS 3
#define TRACKED_SIGNALS MAX_SIGNALS + 2
#define NUM_LASER_LINES 5
#define CFG_PREFIX "/plugins/machine_signal"

namespace fawkes {
class Position3DInterface;
class RobotinoLightInterface;
namespace tf {
class TransformListener;
}
} // namespace fawkes

class MachineSignalPipelineThread : public fawkes::Thread,
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

	void unlock();

	std::list<SignalState> &         get_known_signals();
	std::list<SignalState>::iterator get_best_signal();

	/** Check if pipeline thread is enabled.
   * @return true if pipeline thread is enabled, false otherwise. */
	bool
	is_enabled() const
	{
		return cfg_enable_switch_;
	}

private:
	float signal_beauty(const SignalState::signal_rois_t_ &signal, const firevision::ROI &laser_roi);
	float compactness(const SignalState::signal_rois_t_ &s, const firevision::ROI &laser_roi);

	bool bb_switch_is_enabled(fawkes::SwitchInterface *sw);

	bool cfg_enable_switch_;

	fawkes::Mutex     data_mutex_;
	bool              new_data_;
	double            desired_frametime_;
	fawkes::TimeWait *time_wait_;

	fawkes::SwitchInterface *bb_enable_switch_;

	typedef struct
	{
		firevision::ColorModelSimilarity *                             colormodel;
		firevision::SimpleColorClassifier *                            classifier;
		std::vector<firevision::ColorModelSimilarity::color_class_t *> color_class;
		firevision::ScanlineGrid *                                     scanline_grid;
		std::vector<unsigned int>                                      cfg_ref_col;
		std::vector<int>                                               cfg_chroma_thresh;
		std::vector<int>                                               cfg_sat_thresh;
		std::vector<int>                                               cfg_luma_thresh;
		firevision::color_t                                            color_expect;
		unsigned int                                                   cfg_roi_min_points;
		unsigned int                                                   cfg_roi_basic_size;
		unsigned int                                                   cfg_roi_neighborhood_min_match;
		unsigned int                                                   cfg_scangrid_x_offset;
		unsigned int                                                   cfg_scangrid_y_offset;
		bool                                                           visualize;
	} color_classifier_context_t_;

	color_classifier_context_t_ cfy_ctxt_red_1_;
	color_classifier_context_t_ cfy_ctxt_red_0_;
	color_classifier_context_t_ cfy_ctxt_green_1_;
	color_classifier_context_t_ cfy_ctxt_green_0_;

	std::string         cfg_camera_;
	firevision::Camera *camera_;
	unsigned int        cam_width_, cam_height_;

	unsigned int cfg_fps_;

	fawkes::Position3DInterface *bb_laser_clusters_[3];
	std::atomic<unsigned int>    cfg_lasercluster_min_vis_hist_;
	std::string                  cfg_lasercluster_frame_;
	std::atomic<float>           cfg_lasercluster_signal_radius_;
	std::atomic<float>           cfg_lasercluster_signal_top_;
	std::atomic<float>           cfg_lasercluster_signal_bottom_;
	std::atomic_bool             cfg_lasercluster_enabled_;

	fawkes::LaserLineInterface *bb_laser_lines_[5];
	std::atomic<unsigned int>   cfg_laser_lines_min_vis_hist_;
	std::atomic_bool            cfg_laser_lines_enabled_;
	std::atomic_bool            cfg_laser_lines_moving_avg_;
	void                        bb_open_laser_lines();
	void                        bb_close_laser_lines();

	float                        cfg_cam_aperture_x_;
	float                        cfg_cam_aperture_y_;
	float                        cfg_cam_angle_y_;
	std::string                  cfg_cam_frame_;
	firevision::PositionToPixel *pos2pixel_;

	std::atomic<unsigned int> cfg_roi_max_height_;
	std::atomic<unsigned int> cfg_roi_max_width_;
	std::atomic<float>        cfg_roi_max_aspect_ratio_;
	std::atomic<float>        cfg_roi_max_width_ratio_;
	std::atomic<float>        cfg_roi_xalign_;
	std::atomic<unsigned int> cfg_roi_green_horizon;
	std::atomic_bool          cfg_tuning_mode_;
	std::atomic_bool          cfg_draw_processed_rois_;
	std::atomic<float>        cfg_max_jitter_;
	std::atomic_bool          cfg_debug_processing_;
	std::atomic_bool          cfg_debug_blink_;
	std::atomic_bool          cfg_debug_tf_;

	std::string debug_proc_string_;

	fawkes::Mutex    cfg_mutex_;
	std::atomic_bool cfg_changed_, cam_changed_;

	fawkes::Time *last_second_;
	unsigned int  buflen_;

	// Maybe we want to detect the black cap on top of the signal, too...?
	/*firevision::SimpleColorClassifier *cls_black_cap_;
  unsigned int cfg_black_thresh_;*/

	unsigned int                       cfg_light_on_threshold_;
	unsigned int                       cfg_light_on_min_points_;
	unsigned int                       cfg_light_on_min_neighborhood_;
	std::atomic<float>                 cfg_light_on_min_area_cover_;
	firevision::SimpleColorClassifier *light_classifier_;
	firevision::ColorModelLuminance *  light_colormodel_;
	firevision::ScanlineGrid *         light_scangrid_;

	unsigned int                       cfg_black_y_thresh_;
	unsigned int                       cfg_black_u_thresh_;
	unsigned int                       cfg_black_v_thresh_;
	unsigned int                       cfg_black_u_ref_;
	unsigned int                       cfg_black_v_ref_;
	unsigned int                       cfg_black_min_points_;
	unsigned int                       cfg_black_min_neighborhood_;
	firevision::SimpleColorClassifier *black_classifier_;
	firevision::ColorModelBlack *      black_colormodel_;
	firevision::ScanlineGrid *         black_scangrid_;

	firevision::FilterROIDraw *          roi_drawer_;
	firevision::SharedMemoryImageBuffer *shmbuf_;
	firevision::SharedMemoryImageBuffer *shmbuf_cam_;
	firevision::FilterColorThreshold *   color_filter_;
	firevision::ColorModelSimilarity *   combined_colormodel_;

	void setup_color_classifier(color_classifier_context_t_ *classifier,
	                            firevision::ROI *            roi = nullptr);
	void setup_camera();
	bool color_data_consistent(color_classifier_context_t_ *);
	void reinit_color_config();

	//*/
	struct compare_rois_by_x_
	{
		bool
		operator()(firevision::ROI const &r1, firevision::ROI const &r2)
		{
			return r1.start.x <= r2.start.x;
		}
	}; // sort_rois_by_x_; //*/

	struct compare_rois_by_y_
	{
		bool
		operator()(firevision::ROI const &r1, firevision::ROI const &r2)
		{
			return r1.start.y <= r2.start.y;
		}
	} sort_rois_by_y_;

	/*struct compare_signal_rois_by_x_ {
      bool operator() (SignalState::signal_rois_t_ const &signal1,
  SignalState::signal_rois_t_ const &signal2) { return signal1.red_roi->start.x
  <= signal2.red_roi->start.x;
      }
  } sort_signal_rois_by_x_; //*/

	// All ROIs we want to see painted in the tuning buffer
	std::list<firevision::ROI> drawn_rois_;

	firevision::ROI *red_green_match(firevision::ROI *roi_R, firevision::ROI *roi_G);
	std::list<SignalState::signal_rois_t_> *create_field_signals(std::list<firevision::ROI> *rois_R,
	                                                             std::list<firevision::ROI> *rois_G);
	SignalState::signal_rois_t_ *           create_laser_signals(const firevision::ROI &,
	                                                             std::list<firevision::ROI> *rois_R,
	                                                             std::list<firevision::ROI> *rois_G);

	// Checks to weed out implausible ROIs
	inline bool roi_width_ok(firevision::ROI &r);
	inline bool rois_similar_width(firevision::ROI &r1, firevision::ROI &r2);
	inline bool rois_x_aligned(firevision::ROI &r1, firevision::ROI &r2);
	inline bool roi_aspect_ok(firevision::ROI &r);
	inline bool rois_vspace_ok(firevision::ROI &r1, firevision::ROI &r2);
	inline bool roi1_oversize(firevision::ROI &r1, firevision::ROI &r2);
	inline bool roi1_x_overlaps_below(firevision::ROI &r1, firevision::ROI &r2);
	inline bool roi1_x_intersects(firevision::ROI &r1, firevision::ROI &r2);

	bool get_light_state(firevision::ROI *light);

	std::list<SignalState>           known_signals_;
	std::list<SignalState>::iterator best_signal_;

	std::set<firevision::WorldROI, SignalState::compare_rois_by_area> *bb_get_laser_rois();
	firevision::WorldROI pos3d_to_roi(const fawkes::tf::Stamped<fawkes::tf::Point> &pos3d);
	std::set<firevision::WorldROI, SignalState::compare_rois_by_area> *cluster_rois_;

	fawkes::SignalHintInterface *          bb_signal_position_estimate_;
	fawkes::tf::Stamped<fawkes::tf::Point> signal_hint_;

	/*std::map<firevision::ROI, SignalState::signal_rois_t_, compare_rois_by_x_>
    *merge_rois_in_laser( std::set<firevision::WorldROI,
    SignalState::compare_rois_by_area> *laser_rois, std::list<firevision::ROI>
    *rois_R, std::list<firevision::ROI> *rois_G); //*/

	firevision::ROI *merge_rois_in_roi(const firevision::ROI &     outer_roi,
	                                   std::list<firevision::ROI> *rois);

	// Implemented abstracts inherited from ConfigurationChangeHandler
	virtual void config_tag_changed(const char *new_tag);
	virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);
	virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
	virtual void config_value_erased(const char *path);

	template <typename T>
	bool
	test_set_cfg_value(T *cfg, T val)
	{
		if (*cfg == val)
			return false;
		*cfg = val;
		return true;
	}

protected:
	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	virtual void
	run()
	{
		Thread::run();
	}
};

#endif
