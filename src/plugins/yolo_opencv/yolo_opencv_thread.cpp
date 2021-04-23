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
YoloOpenCVThread::YoloOpenCVThread()
: Thread("YoloOpenCVThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("YoloOpenCVwrite")
{
	set_coalesce_wakeups(false);
};

void
YoloOpenCVThread::finalize()
{
	blackboard->close(yolo_opencv_if_write);
}

void
YoloOpenCVThread::init()
{
	yolo_opencv_if_write = blackboard->open_for_writing<YoloOpenCVInterface>("YoloOpenCVwrite");
	std::string prefix   = CFG_PREFIX;
	//get NN params
	model_path    = this->config->get_string((prefix + "model_path").c_str());
	config_path   = this->config->get_string((prefix + "config_path").c_str());
	classes_path  = this->config->get_string((prefix + "classes_path").c_str());
	framework     = this->config->get_string((prefix + "framework").c_str());
	confThreshold = this->config->get_float((prefix + "confThreshold"));
	nmsThreshold  = this->config->get_float((prefix + "nmsThreshold"));
	scale         = this->config->get_float((prefix + "scale"));
	swapRB        = this->config->get_bool((prefix + "rgb"));
	inpWidth      = this->config->get_int((prefix + "width"));
	inpHeight     = this->config->get_int((prefix + "height"));
	backend       = this->config->get_int((prefix + "backend"));
	target        = this->config->get_int((prefix + "target"));

	std::ifstream classes2detect(classes_path);
	if (!classes2detect.is_open())
		CV_Error(Error::StsError, "File " + classes_path + " not found");
	std::string line;
	while (std::getline(classes2detect, line)) {
		classes.push_back(line);
	}

	//create NN
	net = readNet(model_path, config_path, framework);
	net.setPreferableBackend(backend);
	net.setPreferableTarget(target);
	outNames = net.getUnconnectedOutLayersNames();

	bbil_add_message_interface(yolo_opencv_if_write);

	blackboard->register_listener(this);
	yolo_opencv_if_write->write();

	logger->log_info(name(), "Loaded Yolo Object Detection plugin");
}

void
YoloOpenCVThread::loop()
{
	std::string path2img = read_image_path();
	for (uint i = 0; i < yolo_opencv_if_write->maxlenof_classId(); i++) {
		yolo_opencv_if_write->set_classId(i, 1024);
		yolo_opencv_if_write->set_confidence(i, 0);
		yolo_opencv_if_write->set_centerX(i, 0);
		yolo_opencv_if_write->set_centerY(i, 0);
		yolo_opencv_if_write->set_height(i, 0);
		yolo_opencv_if_write->set_width(i, 0);
	}
	yolo_opencv_if_write->set_msgid(msgid);
	// empty string = reading the interface failed
	if (path2img == "") {
		logger->log_warn(name(), "Got empty string as filename");
		yolo_opencv_if_write->set_detection_successful(false);
		yolo_opencv_if_write->set_error("Got empty string as filename");
		return;
	} else {
		// check if path to image exists
		Mat frame = imread(path2img, IMREAD_COLOR);
		Mat blob;
		//run picture through NN
		if (!frame.empty()) {
			// Process frames.
			preprocess(frame, net, Size(inpWidth, inpHeight), scale, Scalar(), swapRB);
			std::vector<Mat> outs;
			net.forward(outs, outNames);
			std::vector<int>   class_id;
			std::vector<float> confidences;
			std::vector<Rect>  bounding_boxes;
			postprocess(frame, outs, net, class_id, confidences, bounding_boxes, backend);
			// detected something
			if (!class_id.empty()) {
				for (uint i = 0; i < class_id.size(); i++) {
					yolo_opencv_if_write->set_detection_successful(true);
					yolo_opencv_if_write->set_classId(i, class_id[i]);
					yolo_opencv_if_write->set_confidence(i, confidences[i]);
					yolo_opencv_if_write->set_centerX(i, bounding_boxes[i].x);
					yolo_opencv_if_write->set_centerY(i, bounding_boxes[i].y);
					yolo_opencv_if_write->set_height(i, bounding_boxes[i].height);
					yolo_opencv_if_write->set_width(i, bounding_boxes[i].width);
				};
				yolo_opencv_if_write->set_msgid(msgid);
			} else { //didn't detect anything
				logger->log_warn(name(), "Did not detect anything in: %s", path2img.c_str());
				yolo_opencv_if_write->set_detection_successful(false);
				yolo_opencv_if_write->set_error("Did not detect anything");
				return;
			};
		} else {
			logger->log_warn(name(), "Cannot open %s", path2img.c_str());
			yolo_opencv_if_write->set_detection_successful(false);
			yolo_opencv_if_write->set_error("Cannot open file");
			return;
		};
	};
	yolo_opencv_if_write->write();
}
std::string
YoloOpenCVThread::read_image_path()
{ //get picture/video
	yolo_opencv_if_write->read();
	std::string path2img = "";
	while (!yolo_opencv_if_write->msgq_empty()) {
		if (yolo_opencv_if_write->msgq_first_is<YoloOpenCVInterface::DetectObjectMessage>()) {
			YoloOpenCVInterface::DetectObjectMessage *msg =
			  yolo_opencv_if_write->msgq_first<YoloOpenCVInterface::DetectObjectMessage>();
			if (msg->path_to_picture()) {
				path2img = msg->path_to_picture();
				msgid    = msg->id();
			}
		} else {
			logger->log_warn(name(), "Unknown message received");
			path2img = "";
		}
		yolo_opencv_if_write->msgq_pop();
	}
	return path2img;
}

inline void
YoloOpenCVThread::preprocess(const Mat &   frame,
                             Net &         net,
                             Size          inpSize,
                             float         scale,
                             const Scalar &mean,
                             bool          swapRB)
{
	static Mat blob;
	// Create a 4D blob from a frame.
	if (inpSize.width <= 0)
		inpSize.width = frame.cols;
	if (inpSize.height <= 0)
		inpSize.height = frame.rows;
	blobFromImage(frame, blob, 1.0, inpSize, Scalar(), swapRB, false, CV_8U);

	// Run a model.
	net.setInput(blob, "", scale, mean);
	if (net.getLayer(0)->outputNameToIndex("im_info") != -1) // Faster-RCNN or R-FCN
	{
		resize(frame, frame, inpSize);
		Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
		net.setInput(imInfo, "im_info");
	}
}

void
YoloOpenCVThread::postprocess(Mat &                   frame,
                              const std::vector<Mat> &outs,
                              Net &                   net,
                              std::vector<int> &      classIds,
                              std::vector<float> &    confidences,
                              std::vector<Rect> &     boxes,
                              int                     backend)
{
	static std::vector<int> outLayers    = net.getUnconnectedOutLayers();
	static std::string      outLayerType = net.getLayer(outLayers[0])->type;

	if (outLayerType == "DetectionOutput") {
		// Network produces output blob with a shape 1x1xNx7 where N is a number of
		// detections and an every detection is a vector of values
		// [batchId, classId, confidence, left, top, right, bottom]
		CV_Assert(outs.size() > 0);
		for (size_t k = 0; k < outs.size(); k++) {
			float *data = (float *)outs[k].data;
			for (size_t i = 0; i < outs[k].total(); i += 7) {
				float confidence = data[i + 2];
				if (confidence > confThreshold) {
					int left   = (int)data[i + 3];
					int top    = (int)data[i + 4];
					int right  = (int)data[i + 5];
					int bottom = (int)data[i + 6];
					int width  = right - left + 1;
					int height = bottom - top + 1;
					if (width <= 2 || height <= 2) {
						left   = (int)(data[i + 3] * frame.cols);
						top    = (int)(data[i + 4] * frame.rows);
						right  = (int)(data[i + 5] * frame.cols);
						bottom = (int)(data[i + 6] * frame.rows);
						width  = right - left + 1;
						height = bottom - top + 1;
					}
					int centerX = left + width / 2;
					int centerY = top + height / 2;
					classIds.push_back((int)(data[i + 1]) - 1); // Skip 0th background class id.
					boxes.push_back(Rect(centerX, centerY, width, height));
					confidences.push_back(confidence);
				}
			}
		}
	} else if (outLayerType == "Region") {
		for (size_t i = 0; i < outs.size(); ++i) {
			// Network produces output blob with a shape NxC where N is a number of
			// detected objects and C is a number of classes + 4 where the first 4
			// numbers are [center_x, center_y, width, height]
			float *data = (float *)outs[i].data;
			for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
				Mat    scores = outs[i].row(j).colRange(5, outs[i].cols);
				Point  classIdPoint;
				double confidence;
				minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
				if (confidence > confThreshold) {
					int centerX = (int)(data[0] * frame.cols);
					int centerY = (int)(data[1] * frame.rows);
					int width   = (int)(data[2] * frame.cols);
					int height  = (int)(data[3] * frame.rows);
					classIds.push_back(classIdPoint.x);
					confidences.push_back((float)confidence);
					boxes.push_back(Rect(centerX, centerY, width, height));
				}
			}
		}
	} else
		CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);

	// NMS is used inside Region layer only on DNN_BACKEND_OPENCV for another backends we need NMS in sample
	// or NMS is required if number of outputs > 1
	if (outLayers.size() > 1 || (outLayerType == "Region" && backend != DNN_BACKEND_OPENCV)) {
		std::map<int, std::vector<size_t>> class2indices;
		for (size_t i = 0; i < classIds.size(); i++) {
			if (confidences[i] >= confThreshold) {
				class2indices[classIds[i]].push_back(i);
			}
		}
		std::vector<Rect>  nmsBoxes;
		std::vector<float> nmsConfidences;
		std::vector<int>   nmsClassIds;
		for (std::map<int, std::vector<size_t>>::iterator it = class2indices.begin();
		     it != class2indices.end();
		     ++it) {
			std::vector<Rect>   localBoxes;
			std::vector<float>  localConfidences;
			std::vector<size_t> classIndices = it->second;
			for (size_t i = 0; i < classIndices.size(); i++) {
				localBoxes.push_back(boxes[classIndices[i]]);
				localConfidences.push_back(confidences[classIndices[i]]);
			}
			std::vector<int> nmsIndices;
			NMSBoxes(localBoxes, localConfidences, confThreshold, nmsThreshold, nmsIndices);
			for (size_t i = 0; i < nmsIndices.size(); i++) {
				size_t idx = nmsIndices[i];
				nmsBoxes.push_back(localBoxes[idx]);
				nmsConfidences.push_back(localConfidences[idx]);
				nmsClassIds.push_back(it->first);
			}
		}
		boxes       = nmsBoxes;
		classIds    = nmsClassIds;
		confidences = nmsConfidences;
	}
}

bool
YoloOpenCVThread::bb_interface_message_received(Interface *interface, Message *message) throw()
{
	wakeup();
	return true;
}
