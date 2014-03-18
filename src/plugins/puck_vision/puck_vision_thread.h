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

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/vision.h>
#include <aspect/tf.h>

#include <interfaces/PuckVisionInterface.h>

#include <fvcams/camera.h>
#include <fvcams/fileloader.h>

#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include <fvmodels/color/similarity.h>
#include <fvmodels/color/lookuptable.h>
#include <fvmodels/scanlines/grid.h>

#include <fvclassifiers/simple.h>

#include <fvutils/base/roi.h>
#include <fvfilters/roidraw.h>

#include <string>
#include <list>
#include <cmath>


namespace firevision {
  class Camera;
  class ScanlineModel;
  class ColorModel;
  class MirrorModel;
  class SimpleColorClassifier;
  class RelativePositionModel;
  class SharedMemoryImageBuffer;
  class Drawer;
}

namespace fawkes {
	class Position3DInterface;
	class PuckVisionInterface;
	class SwitchInterface;
	namespace tf {
		class TransformListener;
	}
}

class PuckVisionThread
:	public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::VisionAspect,
	public fawkes::BlackBoardAspect,
	public fawkes::TransformAspect

{


private:
	typedef struct {
		firevision::ColorModelSimilarity *colormodel;
		firevision::SimpleColorClassifier *classifier;
		firevision::ColorModelSimilarity::color_class_t *color_class;
		firevision::ScanlineGrid *scanline_grid;
		std::vector<unsigned int> cfg_ref_col;
		int cfg_chroma_thresh;
		int cfg_sat_thresh;
		firevision::color_t color_expect;
		unsigned int cfg_roi_min_points;
		unsigned int cfg_roi_basic_size;
		unsigned int cfg_roi_neighborhood_min_match;
		unsigned int cfg_scangrid_x_offset;
		unsigned int cfg_scangrid_y_offset;
	} color_classifier_context_t_;
	color_classifier_context_t_ cls_red_;

	std::string cfg_prefix_;
	std::string cfg_prefix_static_transforms_;
	std::string cfg_colormodel_mode_;

	//Camera
	std::string cfg_camera_;
	float cfg_camera_opening_angle_horizontal_;
	float cfg_camera_opening_angle_vertical_;
	unsigned int img_width_;
	unsigned int img_height_;

	float cfg_width_top_in_m_;
	float cfg_width_bottem_in_m_;

	float m_per_pixel_height_;

	float offset_cam_x_to_groundplane_;
	float angle_horizontal_to_opening_;


	float visible_lenght_x_in_m_;
	float visible_lenght_y_in_m_;

	float cfg_camera_position_x_;
	float cfg_camera_position_y_;
	float cfg_camera_position_z_;
	float cfg_camera_position_pitch_;

	float cfg_puck_radius_;
	float cfg_distance_function_a_;
	float cfg_distance_function_b_;

	bool cfg_debugMessagesActivated_;
	bool cfg_paintROIsActivated_;

	std::string cfg_colormap_file_yellow_;
	std::string	cfg_colormap_file_red_;
	std::string	cfg_colormap_file_green_;
	std::string	cfg_colormap_file_blue_ ;

	std::string cfg_frame_;

	firevision::Camera *cam_;
	firevision::ScanlineModel *scanline_;

	firevision::ColorModel *cm_yellow_;
	firevision::ColorModel *cm_red_;
	firevision::ColorModel *cm_green_;
	firevision::ColorModel *cm_blue_;

	//firevision::RelativePositionModel *rel_pos_;
	firevision::SimpleColorClassifier *classifier_yellow_;
	firevision::SimpleColorClassifier *classifier_red_;
	firevision::SimpleColorClassifier *classifier_green_;
	firevision::SimpleColorClassifier *classifier_blue_;
	firevision::SimpleColorClassifier *classifier_similarity_;

	firevision::SharedMemoryImageBuffer *shm_buffer_;

	unsigned int cfg_nr_puck_interfaces_;
	//interfaces
	fawkes::SwitchInterface *switchInterface_;
	fawkes::PuckVisionInterface *puckInterface_;

	firevision::ROI roi_center_;
	unsigned char *buffer_;													//reference to the buffer of shm_buffer_YCbCr (to use in code)

	firevision::colorspace_t cspaceFrom_;
	firevision::colorspace_t cspaceTo_;

	firevision::FilterROIDraw *drawer_;

	//Functions

	fawkes::PuckVisionInterface::PuckColor
	getPuckInterfaceColor(firevision::ROI* roi);

	int
	getVisibilityHistory(fawkes::polar_coord_2d_t polar,
				fawkes::PuckVisionInterface::PuckColor colorRoi,
				fawkes::PuckVisionInterface::PuckColor colorInterface,
				float interface_phi,
				float interface_r,
				int interface_visibility);


	void
	drawROIIntoBuffer(firevision::ROI roi, firevision::FilterROIDraw::border_style_t borderStyle = firevision::FilterROIDraw::DASHED_HINT);

	fawkes::polar_coord_2d_t
	transformCoordinateSystem(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to);

	void
	cartToPol(fawkes::polar_coord_2d_t &pol, float x, float y);

	float
	getX(firevision::ROI* roi);

	float
	getY(firevision::ROI* roi);

	void
	polToCart(float &x, float &y, fawkes::polar_coord_2d_t pol);

	firevision::ROI*
	getBiggestRoi(std::list<firevision::ROI>* roiList);

	void
	drawRois(std::list<firevision::ROI>* rois_red_);

	void
	mergeWithColorInformation(firevision::color_t color,
			std::list<firevision::ROI>* rois_color_,
			std::list<firevision::ROI>* rois_all);

	std::list<firevision::ROI>*
	classifyInRoi(firevision::ROI searchArea, firevision::Classifier *classifier);

	void
	updateInterface(std::list<firevision::ROI>* puck);

protected:
	virtual void run() { Thread::run(); }

public:
	PuckVisionThread();
	virtual ~PuckVisionThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();
};

#endif /* puck_vision_THREAD_H_ */
