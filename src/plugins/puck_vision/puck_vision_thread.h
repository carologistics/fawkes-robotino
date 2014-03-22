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
#include <core/threading/mutex.h>

#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/vision.h>
#include <aspect/tf.h>

//#include <interfaces/PuckVisionInterface.h>
#include <interfaces/Position3DInterface.h>

#include <fvcams/camera.h>
#include <fvcams/fileloader.h>
#include <fvclassifiers/simple.h>
#include <fvutils/base/roi.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/conversions.h>
#include <fvmodels/color/similarity.h>
#include <fvmodels/color/lookuptable.h>
#include <fvmodels/scanlines/grid.h>
#include <fvfilters/roidraw.h>

#include <config/change_handler.h>

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

struct camera_info{
	std::string cfg_camera_;
	float opening_angle_horizontal_;
	float opening_angle_vertical_;
	unsigned int img_width_;
	unsigned int img_height_;
	float position_x_;
	float position_y_;
	float position_z_;
	float position_pitch_;
	float offset_cam_x_to_groundplane_;
	float angle_horizontal_to_opening_;
	float visible_lenght_x_in_m_;
	float visible_lenght_y_in_m_;
};

struct color_classifier_context_t_{
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
};

struct puck_info{
	float radius_;
	std::vector<unsigned int> color_main;
//	std::vector<unsigned int> color_top_dots;
//	std::vector<unsigned int> color_holes;
//	std::vector<unsigned int> color_top_center;
};

struct puck{
	double x,y,z;
	double radius;
	int visibiity_history;
};

class PuckVisionThread
:	public fawkes::Thread,
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
	color_classifier_context_t_ cls_red_;
	fawkes::Mutex cfg_mutex_;

	std::string cfg_prefix_;
	std::string cfg_prefix_static_transforms_;
	std::string cfg_colormodel_mode_;

	//Camera
	camera_info camera_info_;
	puck_info puck_info_;

	bool cfg_debugMessagesActivated_;
	bool cfg_paintROIsActivated_;
	bool cfg_changed_;

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

	puck* no_pucK_;

	//interfaces
	std::vector<fawkes::Position3DInterface*> puck_interfaces_;
	std::vector<firevision::ROI> detected_pucks;
	fawkes::SwitchInterface* switchInterface_;
	//fawkes::PuckVisionInterface* puckInterface_;

	firevision::ROI roi_center_;
	unsigned char *buffer_;													//reference to the buffer of shm_buffer_YCbCr (to use in code)

	firevision::colorspace_t cspaceFrom_;
	firevision::colorspace_t cspaceTo_;

	firevision::FilterROIDraw *drawer_;

	//Functions

	//fawkes::PuckVisionInterface::PuckColor
	//getPuckInterfaceColor(firevision::ROI* roi);

//	int
//	getVisibilityHistory(fawkes::polar_coord_2d_t polar,
//				fawkes::PuckVisionInterface::PuckColor colorRoi,
//				fawkes::PuckVisionInterface::PuckColor colorInterface,
//				float interface_phi,
//				float interface_r,
//				int interface_visibility);

	void
	drawROIIntoBuffer(firevision::ROI roi, firevision::FilterROIDraw::border_style_t borderStyle = firevision::FilterROIDraw::DASHED_HINT);

	fawkes::polar_coord_2d_t
	transformCoordinateSystem(fawkes::cart_coord_3d_t cartFrom, std::string from, std::string to);

	void
	createPuckInterface();

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
	loadConfig();

	void
	updatePos3dInferface(fawkes::Position3DInterface* interface, puck* p);
	void
	updateInterface(std::list<puck>* puck);

	virtual void config_value_erased(const char *path);
	virtual void config_tag_changed(const char *new_tag);
	virtual void config_comment_changed(const fawkes::Configuration::ValueIterator *v);
	virtual void config_value_changed(const fawkes::Configuration::ValueIterator *v);

protected:
	virtual void run() { Thread::run(); }
};

#endif /* puck_vision_THREAD_H_ */
