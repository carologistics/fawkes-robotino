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

#ifndef visual_position_THREAD_H_
#define visual_position_THREAD_H_

#define CFG_PREFIX "/plugins/visual_position/"

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/vision.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>

//#include <interfaces/PuckVisionInterface.h>
#include <fvcams/camera.h>
#include <interfaces/Position3DInterface.h>
//#include <fvcams/fileloader.h>
#include "fvutils/adapters/iplimage.h"
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <config/change_handler.h>
#include <fvclassifiers/simple.h>
#include <fvfilters/roidraw.h>
#include <fvfilters/sobel.h>
#include <fvmodels/color/lookuptable.h>
#include <fvmodels/color/similarity.h>
#include <fvmodels/scanlines/grid.h>
#include <fvmodels/shape/ht_lines.h>
#include <fvmodels/shape/line.h>
#include <fvmodels/shape/rht_lines.h>
#include <fvutils/base/roi.h>
#include <fvutils/color/conversions.h>
#include <fvutils/ipc/shm_image.h>
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
	std::string     cfg_camera_;
	float           opening_angle_horizontal_;
	float           opening_angle_vertical_;
	unsigned int    img_width_;
	unsigned int    img_height_;
	float           position_x_;
	float           position_y_;
	float           position_z_;
	float           position_pitch_;
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

struct myLine
{
	firevision::ROI lower_point;
	firevision::ROI upper_point;
	double          angle;
	myLine(firevision::ROI p1, firevision::ROI p2, unsigned int size = 2)
	{
		set(p1, p2, size);
	}
	myLine(cv::Vec4i vec4, unsigned int size = 2)
	{
		set(vec4, size);
	}
	void
	set(firevision::ROI p1, firevision::ROI p2, unsigned int size = 2)
	{
		if (p1.start.y > p2.start.y) {
			lower_point = p1;
			upper_point = p2;
		} else {
			lower_point = p2;
			upper_point = p1;
		}
		calculateAngle();
		setDefaultColors();
		setSize(size, size);
	}
	void
	set(cv::Vec4i vec4, unsigned int size = 2)
	{
		if (vec4.val[1] > vec4.val[3]) { // verify that the lower point is in p1
			// val 1 is bigger... so it is the lower point
			lower_point.start.x = vec4.val[0];
			lower_point.start.y = vec4.val[1];
			upper_point.start.x = vec4.val[2];
			upper_point.start.y = vec4.val[3];
		} else {
			upper_point.start.x = vec4.val[0];
			upper_point.start.y = vec4.val[1];
			lower_point.start.x = vec4.val[2];
			lower_point.start.y = vec4.val[3];
		}
		calculateAngle();
		setDefaultColors();
		setSize(size, size);
	}

	void
	setDefaultColors()
	{ //
		lower_point.color = firevision::C_BLUE;
		upper_point.color = firevision::C_RED;
	}

	void
	setSize(unsigned int h, unsigned int w)
	{
		lower_point.width  = w;
		lower_point.height = h;
		upper_point.width  = w;
		upper_point.height = h;
	}

	double
	calculateAngle()
	{
		int deltaX = lower_point.start.x - upper_point.start.x;
		int deltaY = lower_point.start.y - upper_point.start.y;
		angle      = atan2(deltaY, deltaX) * 180 / M_PI;
		return angle;
	}

	void
	drawCV(cv::Mat image, cv::Scalar color = cv::Scalar(0, 0, 255))
	{
		cv::line(image,
		         cv::Point(upper_point.start.x, upper_point.start.y),
		         cv::Point(lower_point.start.x, lower_point.start.y),
		         color,
		         3,
		         CV_AA);
	}
};

firevision::ROI getCenterRoi(firevision::ROI p1, firevision::ROI p2);

struct puck
{
	fawkes::cart_coord_3d_t  cart;
	fawkes::polar_coord_2d_t pol;
	double                   radius;
	int                      visibiity_history;
	firevision::ROI          roi;
};

class VisualPositionThread : public fawkes::Thread,
                             public fawkes::LoggingAspect,
                             public fawkes::ConfigurableAspect,
                             public fawkes::VisionAspect,
                             public fawkes::BlackBoardAspect,
                             public fawkes::TransformAspect,
                             public fawkes::ConfigurationChangeHandler
{
public:
	VisualPositionThread();
	virtual ~VisualPositionThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	fawkes::Mutex cfg_mutex_;

	std::string cfg_prefix_;
	std::string cfg_prefix_static_transforms_;
	std::string cfg_colormodel_mode_;

	// Camera
	camera_info   camera_info_;
	puck_features puck_info_;

	bool cfg_debugMessagesActivated_;
	bool cfg_paintROIsActivated_;
	bool cfg_changed_;

	std::string cfg_frame_camera_pos_;

	firevision::Camera *                 cam_;
	puck *                               no_pucK_;
	firevision::SharedMemoryImageBuffer *shm_buffer_color_;
	firevision::SharedMemoryImageBuffer *shm_buffer_gray_;
	firevision::SharedMemoryImageBuffer *shm_buffer_gray_edgy_;

	// interfaces
	std::vector<fawkes::Position3DInterface *> puck_interfaces_;
	std::vector<firevision::ROI>               detected_pucks;
	fawkes::SwitchInterface *                  switchInterface_;

	unsigned char *buffer_color_; // reference to the buffer of shm_buffer_YCbCr
	                              // (to use in code)
	unsigned char *buffer_gray_;  // reference to the buffer of shm_buffer_YCbCr
	                              // (to use in code)
	unsigned char *buffer_gray_edgy_;

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

	Point3d apply_tf(const char *target_frame, Point3d src_point);

	std::list<firevision::ROI> *classifyInRoi(firevision::ROI              searchArea,
	                                          color_classifier_context_t_ *color_data);

	void            printRoi(firevision::ROI roi);
	void            fitROI(firevision::ROI &roi, int x, int y, int w, int h);
	firevision::ROI getCenterRoi(firevision::ROI p1, firevision::ROI p2);

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

#endif /* visual_position_THREAD_H_ */
