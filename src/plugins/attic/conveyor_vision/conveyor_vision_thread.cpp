/***************************************************************************
 *  tag_vision_thread.cpp - Thread to print the robot's position to the log
 *
 *  Created: Thu Sep 27 14:31:11 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include "conveyor_vision_thread.h"

#include <interfaces/Position3DInterface.h>
#include <tf/types.h>

//#include <opencv/cv.h>
#include <math.h>

#define CFG_PREFIX "/plugins/conveyor_vision/"
#define IMAGE_CHANNELS 3

using namespace fawkes;
using namespace std;
using namespace cv;

/** @class ConveyorVisionThread "tag_vision_thread.h"
 * Thread to identify AR Tags and provid e their position
 * @author Nicolas Limpert & Randolph Maaßen
 */

/** Constructor. */
ConveyorVisionThread::ConveyorVisionThread()
: Thread("ConveyorVisionThread", Thread::OPMODE_WAITFORWAKEUP),
  VisionAspect(VisionAspect::CYCLIC),
  ConfigurationChangeHandler(CFG_PREFIX),
  fawkes::TransformAspect(fawkes::TransformAspect::ONLY_PUBLISHER, "conveyor")
{
	fv_cam           = NULL;
	shm_buffer       = NULL;
	image_buffer     = NULL;
	ipl              = NULL;
	use_hough_lines_ = false;
	//    this->markers_ = NULL;
}

void
ConveyorVisionThread::init()
{
	std::string prefix           = CFG_PREFIX;
	std::string mps_cascade_name = (string)config->get_string((prefix + "classifier_file"));
	if (!mps_cascade.load(std::string(CONFDIR) + "/" + mps_cascade_name)) {
		printf("--(!)Error loading\n");
		return;
	};

	//   fawkes::Position3DInterface* puck_if_ = NULL;
	try {
		mps_conveyor_if_ = blackboard->open_for_writing<fawkes::Position3DInterface>("mps_conveyor");
		//            puck_if_->set_frame(camera_info_.frame.c_str());
		mps_conveyor_if_->write();
		//            puck_interfaces_.push_back(puck_if_);
	} catch (std::exception &e) {
		finalize();
		throw;
	}

	config->add_change_handler(this);
	// load config
	// config prefix in string for concatinating
	//    std::string prefix = CFG_PREFIX;
	// log, that we open load the config
	logger->log_info(name(), "loading config");
	load_config();
	// load alvar camera calibration
	//    if(!alvar_cam.SetCalib(config->get_string((prefix +
	//    "classifier_file").c_str()).c_str(),0,0,FILE_FORMAT_DEFAULT))
	//    {
	//      this->logger->log_warn(this->name(),"Faild to load calibration file");
	//    }
	//    // load marker size and apply it
	//    marker_size = config->get_uint((prefix + "marker_size").c_str());
	//    detector.SetMarkerSize(marker_size);

	// Image Buffer ID

	// init firevision camera
	// CAM swapping not working (??)
	if (fv_cam != NULL) {
		// free the camera
		fv_cam->stop();
		fv_cam->flush();
		fv_cam->dispose_buffer();
		fv_cam->close();
		delete fv_cam;
		fv_cam = NULL;
	}
	if (fv_cam == NULL) {
		std::string connection = this->config->get_string((prefix + "camera").c_str());
		fv_cam                 = vision_master->register_for_camera(connection.c_str(), this);
		fv_cam->start();
		fv_cam->open();
		this->img_width  = fv_cam->pixel_width();
		this->img_height = fv_cam->pixel_height();
	}

	// set camera resolution
	//    alvar_cam.SetRes(this->img_width, this->img_height);

	// SHM image buffer
	if (shm_buffer != NULL) {
		delete shm_buffer;
		shm_buffer   = NULL;
		image_buffer = NULL;
	}

	shm_buffer = new firevision::SharedMemoryImageBuffer(shm_id.c_str(),
	                                                     firevision::YUV422_PLANAR,
	                                                     this->img_width,
	                                                     this->img_height);
	if (!shm_buffer->is_valid()) {
		delete shm_buffer;
		delete fv_cam;
		shm_buffer = NULL;
		fv_cam     = NULL;
		throw fawkes::Exception("Shared memory segment not valid");
	}
	std::string frame = this->config->get_string((prefix + "frame").c_str());
	shm_buffer->set_frame_id(frame.c_str());

	image_buffer = shm_buffer->buffer();
	ipl = cvCreateImage(cvSize(this->img_width, this->img_height), IPL_DEPTH_8U, IMAGE_CHANNELS);

	// set up marker
	world_pos_z_average = 0.;
	max_marker          = 16;
	//    this->markers_ = new std::vector<alvar::MarkerData>();
	//    this->tag_interfaces = new
	//    TagPositionList(this->blackboard,this->max_marker,frame,this->name(),this->logger,
	//    this->clock, this->tf_publisher);
}

void
ConveyorVisionThread::finalize()
{
	vision_master->unregister_thread(this);
	config->rem_change_handler(this);
	// free the markers
	//  this->markers_->clear();
	//  delete this->markers_;
	delete fv_cam;
	fv_cam = NULL;
	delete shm_buffer;
	shm_buffer   = NULL;
	image_buffer = NULL;
	//  cvReleaseImage(&ipl);
	ipl = NULL;
	//  delete this->tag_interfaces;
}

void
ConveyorVisionThread::loop()
{
	if (!cfg_mutex.try_lock()) {
		// logger->log_info(name(),"Skipping loop");
		return;
	}
	if (fv_cam == NULL || !fv_cam->ready()) {
		logger->log_info(name(), "Camera not ready");
		init();
		return;
	}
	// logger->log_info(name(),"entering loop");
	// get img form fv
	fv_cam->capture();
	firevision::convert(fv_cam->colorspace(),
	                    firevision::YUV422_PLANAR,
	                    fv_cam->buffer(),
	                    image_buffer,
	                    this->img_width,
	                    this->img_height);
	fv_cam->dispose_buffer();
	// convert img
	firevision::IplImageAdapter::convert_image_bgr(image_buffer, ipl);
	frame = cvarrToMat(ipl);
	detect();
	mps_conveyor_if_->write();
	// get marker from img
	//    get_marker();

	//    this->tag_interfaces->update_blackboard(this->markers_);

	cfg_mutex.unlock();
}

// void
// ConveyorVisionThread::get_marker()
//{
//    // detect makres on image
////    detector.Detect(ipl,&alvar_cam);
//    // reset currently saved markers
//    this->markers_->clear();
//    // fill output array
//    for(alvar::MarkerData &tmp_marker: *(this->detector.markers))
//    {
//        Pose tmp_pose = tmp_marker.pose;
//        //skip the marker, if the pose is directly on the camera (error)
//        if(tmp_pose.translation[0]<1 && tmp_pose.translation[1]<1 &&
//        tmp_pose.translation[2]<1){
//            continue;
//        }
//        this->markers_->push_back(tmp_marker);
//        // add up to markers
//        tmp_marker.Visualize(ipl,&alvar_cam);
//    }
//    firevision::IplImageAdapter::convert_image_yuv422_planar(ipl,image_buffer);
//}

// config handling
void ConveyorVisionThread::config_value_erased(const char *path){};
void ConveyorVisionThread::config_tag_changed(const char *new_tag){};
void ConveyorVisionThread::config_comment_changed(const fawkes::Configuration::ValueIterator *v){};
void
ConveyorVisionThread::config_value_changed(const fawkes::Configuration::ValueIterator *v)
{
	load_config();
}

void
ConveyorVisionThread::load_config()
{
	if (cfg_mutex.try_lock()) {
		try {
			std::string prefix = CFG_PREFIX;
			// log, that we open load the config
			logger->log_info(name(), "loading config");
			shm_id                 = config->get_string((prefix + "shm_image_id").c_str());
			obj_realworld_distance = config->get_float((prefix + "obj_realworld_distance").c_str());
			obj_realworld_pixels   = config->get_float((prefix + "obj_realworld_pixels").c_str());
			obj_realworld_width    = config->get_float((prefix + "obj_realworld_width").c_str());
			num_frames_for_average = config->get_int((prefix + "num_frames_for_average").c_str());
			conveyor_distance_threshold =
			  config->get_float((prefix + "conveyor_distance_threshold").c_str());
			visualization_enabled = config->get_bool((prefix + "visualization_enabled").c_str());

			use_hough_lines_ =
			  config->get_bool((prefix + "/houghlines/use_hough_line_optimization").c_str());
			if (use_hough_lines_) {
				hough_lines_averaging_count_ =
				  config->get_uint((prefix + "houghlines/hough_lines_averaging_count").c_str());

				binary_threshold_min_ =
				  config->get_int((prefix + "houghlines/binary_threshold_min").c_str());
				binary_threshold_max_ =
				  config->get_int((prefix + "houghlines/binary_threshold_max").c_str());
				binary_threshold_average_min_ =
				  config->get_int((prefix + "houghlines/binary_threshold_average_min").c_str());
				binary_threshold_average_max_ =
				  config->get_int((prefix + "houghlines/binary_threshold_average_max").c_str());
				max_binary_value_ = config->get_int((prefix + "houghlines/max_binary_value").c_str());
				threshold_type_   = config->get_int((prefix + "houghlines/threshold_type").c_str());
				morph_element_    = config->get_int((prefix + "houghlines/morph_element").c_str());
				morph_size_       = config->get_int((prefix + "houghlines/morph_size").c_str());
				morph_operator_   = config->get_int((prefix + "houghlines/morph_operator").c_str());
				canny_threshold_  = config->get_int((prefix + "houghlines/canny_threshold").c_str());
				line_mean_        = config->get_uint((prefix + "houghlines/line_mean").c_str());
				conveyor_realworld_distance =
				  config->get_float((prefix + "houghlines/conveyor_realworld_distance").c_str());
				conveyor_realworld_pixels =
				  config->get_float((prefix + "houghlines/conveyor_realworld_pixels").c_str());
				conveyor_realworld_width =
				  config->get_float((prefix + "houghlines/conveyor_realworld_width").c_str());
			}
			// load alvar camera calibration
			//      alvar_cam.SetCalib(config->get_string((prefix +
			//      "alvar_camera_calib_file").c_str()).c_str(),0,0,FILE_FORMAT_DEFAULT);
			// load marker size and apply it
			//      marker_size = config->get_uint((prefix + "marker_size").c_str());
			//      detector.SetMarkerSize(marker_size);
		} catch (fawkes::Exception &e) {
			logger->log_error(name(), e);
		}
	}
	// gets called for every changed entry... so init is called once per change.
	cfg_mutex.unlock();
}

/** @function detectAndDisplay */
void
ConveyorVisionThread::detect()
{
	std::vector<Rect> faces;
	Mat               frame_gray;

	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	//-- Detect faces
	mps_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
	int visibility_history = mps_conveyor_if_->visibility_history();

	//  logger->log_info(name(), "got %d faces", faces.size());
	// ignore images that probably contain more than one conveyor
	if (faces.size() == 1) {
		// width of object at one meters distance

		float focal_length = (obj_realworld_pixels * obj_realworld_distance) / obj_realworld_width;
		float d_dash       = (focal_length * obj_realworld_width) / faces[0].width;

		//    float beta = (asin(W/d_dash) * 180) / M_PI;
		float beta = asin(obj_realworld_width / d_dash);

		float multiplier         = frame.cols / faces[0].width;
		float overall_open_angle = beta * multiplier;

		// center position of predRect relative to the middle of the image.
		float center_x            = faces[0].x + faces[0].width / 2 - frame.cols / 2;
		float pixels_x_per_degree = frame.cols / overall_open_angle;
		float center_x_angle      = center_x / pixels_x_per_degree;
		float world_pos_x         = (sin(center_x_angle) * d_dash) / 100;

		// center position of predRect relative to the middle of the image.
		float center_y            = faces[0].y + faces[0].height / 2 - frame.rows / 2;
		float pixels_y_per_degree = frame.rows / overall_open_angle;
		float center_y_angle      = center_y / pixels_y_per_degree;
		float world_pos_y         = -(sin(center_y_angle) * d_dash) / 100;

		float world_pos_z = sqrt(d_dash * d_dash + world_pos_x * world_pos_x) / 100;
		if (world_pos_z > conveyor_distance_threshold)
			return;

		world_pos_z_measurements.push_front(world_pos_z);
		if (world_pos_z_measurements.size() > num_frames_for_average) {
			world_pos_z_measurements.pop_back();
		}
		for (size_t i = 0; i < world_pos_z_measurements.size(); i++) {
			world_pos_z_average += world_pos_z_measurements[i];
		}
		world_pos_z_average /= world_pos_z_measurements.size();

		world_pos_y_measurements.push_front(world_pos_y);
		if (world_pos_y_measurements.size() > num_frames_for_average) {
			world_pos_y_measurements.pop_back();
		}
		for (size_t i = 0; i < world_pos_y_measurements.size(); i++) {
			world_pos_y_average += world_pos_y_measurements[i];
		}
		world_pos_y_average /= world_pos_y_measurements.size();

		world_pos_x_measurements.push_front(world_pos_y);
		if (world_pos_x_measurements.size() > num_frames_for_average) {
			world_pos_x_measurements.pop_back();
		}
		for (size_t i = 0; i < world_pos_x_measurements.size(); i++) {
			world_pos_x_average += world_pos_x_measurements[i];
		}
		world_pos_x_average /= world_pos_x_measurements.size();

		//    float focal_length_mm = 50;
		//    float obj_height_mm = 50;
		//    float im_height_px = frame.rows;
		//    float obj_height_px = faces[0].height;
		//    float sensor_height_mm = 2; // 1/4" in mm
		//    float height_width_sqrt = sqrt(frame.rows * frame.rows + frame.cols *
		//    frame.cols); float opening_angle = 0.86325; float pixels_per_rad =
		//    opening_angle / height_width_sqrt;
		////    float norm_width = 0.47 / 78;
		////    float norm_height = 0.4 / 85;
		//    Point center( faces[0].x + faces[0].width*0.5, faces[0].y +
		//    faces[0].height*0.5 ); float dist_from_center_px = frame.cols / 2 -
		//    center.x; float obj_deg = -(dist_from_center_px * pixels_per_rad);
		//    float distance = (focal_length_mm * obj_height_mm * im_height_px) /
		//    (obj_height_px * sensor_height_mm);

		//    printf("found face: x: %d, y: %d, width: %d, height: %d distance:%f,
		//    obj_deg: %f\n", faces[0].x, faces[0].y, faces[0].width,
		//    faces[0].height, distance, obj_deg); logger->log_info(name(), "Found
		//    conveyor: x= %f, y= %f, z= %f", world_pos_z_average, world_pos_z,
		//    world_pos_y);

		if (use_hough_lines_) {
			Rect myROI(faces[0].x, faces[0].y, faces[0].width, faces[0].height);
			Mat  croppedImage = frame(myROI);
			cvtColor(croppedImage, croppedImage, CV_BGR2GRAY);

			// calculate binarization threshold depending on average gray-value in the
			// image
			Scalar the_mean   = mean(croppedImage);
			float  float_mean = the_mean.val[0];

			int calculated_threshold = binary_threshold_min_;
			if (float_mean >= binary_threshold_average_min_
			    && float_mean <= binary_threshold_average_max_) {
				calculated_threshold += (float_mean - binary_threshold_average_min_);
			} else if (float_mean > binary_threshold_average_max_) {
				calculated_threshold = binary_threshold_max_;
			}

			int thresh = 200;
			/// Detect edges using canny
			Mat dst, cdst;
			threshold(croppedImage, dst, calculated_threshold, max_binary_value_, threshold_type_);
			Canny(dst, dst, thresh, thresh * 2, 7);

			int operation = morph_operator_ + 2;

			Mat element = getStructuringElement(morph_element_,
			                                    Size(2 * morph_size_ + 1, 2 * morph_size_ + 1),
			                                    Point(morph_size_, morph_size_));

			// Apply the specified morphology operation
			morphologyEx(dst, dst, operation, element);
			int roi_width_limiter  = dst.cols / 4.2; // 10 percent of width of image
			int roi_height_limiter = dst.rows / 4.8; // 10 percent of width of image

			// fill left side
			rectangle(dst, Point(0, 0), Point(roi_width_limiter, dst.rows), Scalar(0, 0, 0), CV_FILLED);
			// fill right side
			rectangle(dst,
			          Point(dst.cols - roi_width_limiter, 0),
			          Point(dst.cols, dst.rows),
			          Scalar(0, 0, 0),
			          CV_FILLED);
			// fill top
			rectangle(dst, Point(0, 0), Point(dst.cols, roi_height_limiter), Scalar(0, 0, 0), CV_FILLED);
			// fill middle
			rectangle(dst,
			          Point(dst.cols / 2 - 1.5 * (dst.cols / 8), 0),
			          Point(dst.cols / 2 + 1.5 * (dst.cols / 8), dst.rows),
			          Scalar(0, 0, 0),
			          CV_FILLED);

			vector<Vec4i> lines;
			cvtColor(dst, cdst, CV_GRAY2BGR);

			double rho             = 1;
			double theta           = CV_PI / 180;
			double minLineLength   = 10;
			double maxLineGap      = 25;
			int    hough_threshold = 20;

			HoughLinesP(dst, lines, rho, theta, hough_threshold, minLineLength, maxLineGap);

			int nearest_line_left_offset  = 0;
			int nearest_line_right_offset = dst.cols;
			int nearest_line_left_index   = 0;
			int nearest_line_right_index  = 0;

			vector<Vec4i> vertical_lines;

			for (size_t j = 0; j < lines.size(); j++) {
				Vec4i l = lines[j];

				// is vertically oriented line?
				if (abs(l[1] - l[3]) > 20) {
					if (l[0] > l[2]) {
						int diff = (l[0] - l[2]) / 2;
						l[0] -= diff;
						l[2] += diff;
					} else {
						int diff = (l[2] - l[0]) / 2;
						l[0] += diff;
						l[2] -= diff;
					}
					vertical_lines.push_back(l);
				}
			}
			for (size_t iV = 0; iV < vertical_lines.size(); iV++) {
				Vec4i l = vertical_lines[iV];
				if (l[0] > nearest_line_left_offset && !(l[0] > dst.cols / 2)) {
					nearest_line_left_offset = l[0];
					nearest_line_left_index  = iV;
				}
				if (l[0] < nearest_line_right_offset && !(l[0] < dst.cols / 2)) {
					nearest_line_right_offset = l[0];
					nearest_line_right_index  = iV;
				}
			}

			// fail if no vertical lines detected or if horizontal distance between
			// lines is too low
			if (vertical_lines.size() > 0
			    && abs(vertical_lines[nearest_line_left_index][0]
			           - vertical_lines[nearest_line_right_index][0])
			         > 20) {
				int horizontal_diff  = abs(vertical_lines[nearest_line_left_index][0]
                                  - vertical_lines[nearest_line_right_index][0]);
				int horizontal_start = vertical_lines[nearest_line_left_index][0];
				int vertical_start   = abs(vertical_lines[nearest_line_left_index][1]
                                 - vertical_lines[nearest_line_left_index][3]);
				conveyor_average_horizontal_diff.push_front(horizontal_diff);
				conveyor_average_horizontal_start.push_front(horizontal_start);
				conveyor_average_vertical_start.push_front(vertical_start);
			}
			if (conveyor_average_horizontal_diff.size() > line_mean_) {
				conveyor_average_horizontal_diff.pop_back();
			}
			if (conveyor_average_horizontal_start.size() > line_mean_) {
				conveyor_average_horizontal_start.pop_back();
			}
			if (conveyor_average_vertical_start.size() > line_mean_) {
				conveyor_average_vertical_start.pop_back();
			}

			int vert_start = 0;
			int hor_start  = 0;
			int hor_diff   = 0;

			for (size_t i = 0; i < conveyor_average_horizontal_diff.size(); i++) {
				vert_start += conveyor_average_vertical_start[i];
				hor_start += conveyor_average_horizontal_start[i];
				hor_diff += conveyor_average_horizontal_diff[i];
			}

			if (conveyor_average_vertical_start.size() > 0) {
				vert_start = vert_start / conveyor_average_vertical_start.size();
				hor_start  = hor_start / conveyor_average_horizontal_start.size();
				hor_diff   = hor_diff / conveyor_average_horizontal_diff.size();

				line(frame,
				     Point(faces[0].x + hor_start, faces[0].y + vert_start),
				     Point(faces[0].x + hor_start + hor_diff, faces[0].y + vert_start),
				     Scalar(255, 255, 0),
				     3,
				     CV_AA);
			}

			float focal_length =
			  (conveyor_realworld_pixels * conveyor_realworld_distance) / conveyor_realworld_width;
			float d_dash = (focal_length * conveyor_realworld_width) / hor_diff;

			float beta = asin(conveyor_realworld_width / d_dash);

			float multiplier         = frame.cols / faces[0].width;
			float overall_open_angle = beta * multiplier;
			// center position of predRect relative to the middle of the image.
			float center_x            = faces[0].x + hor_start + hor_diff / 2;
			float pixels_x_per_degree = frame.cols / overall_open_angle;
			float center_x_angle      = center_x / pixels_x_per_degree;
			world_pos_x               = sin(center_x_angle) * d_dash;

			world_pos_z_average = sqrt(d_dash * d_dash + world_pos_x * world_pos_x) / 100.;
			world_pos_x /= 100.;
		}
		mps_conveyor_if_->set_translation(0, world_pos_z_average);
		mps_conveyor_if_->set_translation(1, -world_pos_x);
		mps_conveyor_if_->set_translation(2, world_pos_y_average);
		mps_conveyor_if_->set_rotation(0, 0);
		mps_conveyor_if_->set_rotation(1, 0);
		// interface->set_timestamp(&p->cart.stamp);
		mps_conveyor_if_->set_frame("base_conveyor");
		mps_conveyor_if_->set_visibility_history(visibility_history);
		if (visibility_history >= 0) {
			mps_conveyor_if_->set_visibility_history(visibility_history + 1);
		} else {
			mps_conveyor_if_->set_visibility_history(1);
		}
		if (visualization_enabled) {
			cv::rectangle(frame,
			              Point(faces[0].x, faces[0].y),
			              Point(faces[0].x + faces[0].width, faces[0].y + faces[0].height),
			              Scalar(255, 0, 0),
			              1);
			cvReleaseImage(&ipl);
			ipl      = cvCreateImage(cvSize(frame.cols, frame.rows), 8, 3);
			Mat newC = cvarrToMat(ipl);
			*ipl     = newC;
			firevision::IplImageAdapter::convert_image_yuv422_planar(ipl, image_buffer);
		}
	} else {
		// conveyor not visible
		if (visibility_history <= 0) {
			mps_conveyor_if_->set_visibility_history(visibility_history - 1);
		} else {
			mps_conveyor_if_->set_visibility_history(-1);
			world_pos_z_measurements.clear();
			world_pos_x_measurements.clear();
			world_pos_y_measurements.clear();
		}
	}
}
