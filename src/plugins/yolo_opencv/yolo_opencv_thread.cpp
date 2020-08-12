/***************************************************************************
 *  yolo_opencv_thread.cpp - Thread to run Yolo image detection
 *  Created: Wed Aug 12 14:04:00 2020
 *  Copyright  2020 Niklas Sebastian Eltester
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

#include "yolo_opencv_thread.h"

#include <interfaces/YoloOpenCVInterface.h>

#include <fstream>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pthread.h>

#define CFG_PREFIX "/plugins/yolo_opencv/"

using namespace fawkes;
using namespace std;
using namespace cv;
using namespace dnn;

/** @class YoloOpenCVThread 'yolo_opencv_thread.h'
 * Thread to run Yolo image detection
 * @author Niklas Sebastian Eltester
 */

/** Constructor. */
YoloOpenCVThread::YoloOpenCVThread() : Thread("YoloOpenCVThread", Thread::OPMODE_WAITFORWAKEUP){};

void
YoloOpenCVThread::finalize()
{
	blackboard->close(yolo_opencv_if_);
}

void
YoloOpenCVThread::init()
{
	yolo_opencv_if_    = blackboard->open_for_writing<YoloOpenCVInterface>("YoloOpenCV");
	std::string prefix = CFG_PREFIX;
	//get NN params
	std::string model_path     = this->config->get_string((prefix + "model_path").c_str());
	std::string config_path    = this->config->get_string((prefix + "config_path").c_str());
	std::string classes_path   = this->config->get_string((prefix + "classes_path").c_str());
	std::string framework      = this->config->get_string((prefix + "framework").c_str());
	yolo_opencv::confThreshold = this->config->get_float((prefix + "confThreshold"));
	yolo_opencv::nmsThreshold  = this->config->get_float((prefix + "nmsThreshold"));
	float  scale               = this->config->get_float((prefix + "scale"));
	bool   swapRB              = this->config->get_bool((prefix + "rgb"));
	int    inpWidth            = this->config->get_int((prefix + "width"));
	int    inpHeigth           = this->config->get_int((prefix + "height"));
	int    backend             = this->config->get_int((prefix + "backend"));
	int    target              = this->config->get_int((prefix + "target"));
	size_t asyncNumReq         = this->config->get_int((prefix + "async"));
	Scalar mean                = Scalar(); //TODO

	std::ifstream ifs(classes_path);
	if (!ifs.is_open())
		CV_Error(Error::StsError, "File " + file + " not found");
	std::string line;
	while (std::getline(ifs, line)) {
		yolo_opencv::classes.push_back(line);
	}

	//create NN
	Net net = readNet(model_path, config_path, framework);
	net.setPreferableBackend(backend);
	net.setPreferableTarget(target);
	std::vector<String> outNames = net.getUnconnectedOutLayersName();

	VideoCapture cap;
}

void
YoloOpenCVThread::loop()
{
	//get picture/video
	yolo_opencv_if_->read();
	while (!yolo_opencv_if_->msgq_empty()) {
		if (yolo_opencv_if_->msgq_first_is<YoloOpenCVInterface::DetectObjectMessage>()) {
			YoloOpenCVInterface::DetectObjectMessage *msg =
			  yolo_opencv_if_->msgq_first<YoloOpenCVInterface::DetectObjectMessage>();
			if (msg->path_to_picture()) {
				cap.open(msg->path_to_picture());
			} else if (msg->video_device()) {
				cap.open(msg->video_device());
			}
		} else {
			logger->log_warn(name(), "Unknown message received");
		}
		yolo_opencv_if_->msgq_pop();
	}

	//run picture through NN
}
