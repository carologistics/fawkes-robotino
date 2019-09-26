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

#ifndef puck_vision_THREAD_H_
#define puck_vision_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/vision.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>

//#include <interfaces/PuckVisionInterface.h>
#include <config/change_handler.h>
#include <fvcams/camera.h>
#include <fvclassifiers/simple.h>
#include <fvfilters/roidraw.h>
#include <fvmodels/color/lookuptable.h>
#include <fvmodels/color/similarity.h>
#include <fvmodels/color/thresholds_black.h>
#include <fvmodels/scanlines/grid.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>
#include <interfaces/Position3DInterface.h>
#include <tf/exceptions.h>
#include <tf/transformer.h>
#include <tf/types.h>
#include <tf/utils.h>

#include <algorithm> // sort
#include <list>
#include <math.h>
#include <string>

namespace firevision {
class Camera;
class ScanlineModel;
class ColorModel;
class MirrorModel;
class SimpleColorClassifier;
class RelativePositionModel;
class SharedMemoryImageBuffer;
class Drawer;
} // namespace firevision

namespace fawkes {
class Position3DInterface;
class PuckVisionInterface;
class SwitchInterface;
namespace tf {
class TransformListener;
}
} // namespace fawkes

typedef fawkes::tf::Stamped<fawkes::tf::Point> Point3d;

struct camera_info
{
	std::string     connection;
	std::string     frame;
	float           opening_angle_horizontal;
	float           opening_angle_vertical;
	unsigned int    img_width;
	unsigned int    img_height;
	float           position_x;
	float           position_y;
	float           position_z;
	float           position_pitch;
	float           offset_cam_x_to_groundplane_;
	float           angle_horizontal_to_opening_;
	float           visible_lenght_x_in_m_;
	float           visible_lenght_y_in_m_;
	firevision::ROI fullimage;
};

typedef struct
{
	firevision::ColorModel *                                       colormodel;
	firevision::SimpleColorClassifier *                            classifier;
	std::vector<firevision::ColorModelSimilarity::color_class_t *> color_classes;
	firevision::ScanlineGrid *                                     scanline_grid;
	std::vector<unsigned int>                                      cfg_ref_col;
	int                                                            cfg_chroma_thresh;
	int                                                            cfg_sat_thresh;
	firevision::color_t                                            color_expect;
	unsigned int                                                   cfg_roi_min_points;
	unsigned int                                                   cfg_roi_basic_size;
	unsigned int                                                   cfg_roi_neighborhood_min_match;
	unsigned int                                                   cfg_scangrid_x_offset;
	unsigned int                                                   cfg_scangrid_y_offset;
} color_classifier_context_t_;

struct puck_features
{
	float                       radius;
	float                       height;
	color_classifier_context_t_ main;
	color_classifier_context_t_ top_dots;
	// color holes;
	// color top_center;
};

struct puck
{
	Point3d                  cart;
	fawkes::polar_coord_2d_t pol;
	double                   radius;
	int                      visibility_history;
	firevision::ROI          roi;
};

class PuckVisionThread : public fawkes::Thread,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::VisionAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::TransformAspect,
                         public fawkes::ConfigurationChangeHandler
{
public:
	PuckVisionThread();
	virtual ~PuckVisionThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	Point3d apply_tf(const char *target_frame, Point3d src_point);

	fawkes::Mutex cfg_mutex_;

	std::string cfg_prefix_;
	std::string cfg_prefix_static_transforms_;
	std::string cfg_frame_target;
	bool        cfg_use_new_pucks_;

	// Camera
	camera_info   camera_info_;
	puck_features puck_info_;

	bool cfg_debugMessagesActivated_;
	bool cfg_paintROIsActivated_;
	bool cfg_changed_;
	bool cfg_check_yellow_dots_;

	std::string cfg_frame_;

	firevision::Camera *                 cam_;
	firevision::SharedMemoryImageBuffer *shm_buffer_;

	// interfaces
	std::vector<fawkes::Position3DInterface *> puck_interfaces_;
	std::vector<firevision::ROI>               detected_pucks;
	fawkes::SwitchInterface *                  switchInterface_;

	unsigned char *buffer_; // reference to the buffer of shm_buffer_YCbCr (to use in code)

	firevision::colorspace_t cspaceFrom_;
	firevision::colorspace_t cspaceTo_;

	firevision::FilterROIDraw *drawer_;

	firevision::ROI search_area;

	// Helper functions
	float getX(firevision::ROI *roi);
	float getY(firevision::ROI *roi);

	firevision::ROI *getBiggestRoi(std::list<firevision::ROI> *roiList);
	firevision::ROI *getRoiContainingRoi(std::list<firevision::ROI> *roiList,
	                                     firevision::ROI             containing);
	void             mergeWithColorInformation(firevision::color_t         color,
	                                           std::list<firevision::ROI> *rois_color_,
	                                           std::list<firevision::ROI> *rois_all);

	void setup_color_classifier(color_classifier_context_t_ *color_data,
	                            const char *                 prefix,
	                            firevision::color_t          expected);

	std::list<firevision::ROI> *classifyInRoi(firevision::ROI              searchArea,
	                                          color_classifier_context_t_ *color_data);

	void printRoi(firevision::ROI roi);
	void fitROI(firevision::ROI &roi, int x, int y, int w, int h);

	/** detectPucks
   *  Returns a list of ROIs. Each ROI contians a puck
   */
	std::list<firevision::ROI> detectPucks();

	void loadConfig();
	void polToCart(float &x, float &y, fawkes::polar_coord_2d_t pol);
	void createPuckInterface();
	void cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y);
	void drawRois(std::list<firevision::ROI> *rois_red_);
	void drawROIIntoBuffer(
	  firevision::ROI                           roi,
	  firevision::FilterROIDraw::border_style_t borderStyle = firevision::FilterROIDraw::DASHED_HINT);
	void calculatePuckPositions(std::vector<puck> *pucks, std::list<firevision::ROI> pucks_in_view);
	void getPuckPosition(puck *p, firevision::ROI roi);
	void sortPucks(std::vector<puck> *pucks);

	void updatePos3dInferface(fawkes::Position3DInterface *interface, puck *p);
	void updateInterface(std::vector<puck> *puck);
	void init_with_config();

	virtual void config_value_erased(const char *path);
	virtual void config_tag_changed(const char *new_tag);
	virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
	virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);
	void         deleteClassifier(color_classifier_context_t_ *color_data);

protected:
	virtual void
	run()
	{
		Thread::run();
	}
};

#endif /* puck_vision_THREAD_H_ */
