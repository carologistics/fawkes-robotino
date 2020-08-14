/***************************************************************************
 *  yolo_opencv_thread.h - Thread to run Yolo image detection
 *  Created: Wed Aug 12 13:23:00 2020
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

#ifndef __PLUGINS_YOLO_OPENCV_THREAD_H__
#define __PLUGINS_YOLO_OPENCV_THREAD_H__

//Fawkes
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>

//OpenCV modules
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <queue>
#include <string>

namespace fawkes {
class YoloOpenCVInterface;
};

namespace yolo_opencv {

float                    confThreshold;
float                    nmsThreshold;
std::vector<std::string> classes;

void preprocess(const cv::Mat &   frame,
                cv::dnn::Net &    net,
                cv::Size          inpSize,
                float             scale,
                const cv::Scalar &mean,
                bool              swapRB);

void postprocess(cv::Mat &frame, const std::vector<Mat> &out, cv::dnn::Net &net, int backend);

/**
 * Queueing frames for image detection
 * @author Alessandro de Oliveira Faria
 */

template <typename T>
class QueueFPS : public std::queue<T>
{
public:
	/** Constructor. */
	QueueFPS() : counter(0)
	{
	}
	
	/** Push frame into queue */
	void
	push(const T &entry)
	{
		std::lock_guard<std::mutex> lock(mutex);

		std::queue<T>::push(entry);
		counter += 1;
		if (counter == 1) {
			// Start counting from a second frame (warmup).
			tm.reset();
			tm.start();
		}
	}
	
	/** Get frame from queue*/
	T
	get()
	{
		std::lock_guard<std::mutex> lock(mutex);
		T                           entry = this->front();
		this->pop();
		return entry;
	}
	
	/** Get fps*/
	float
	getFPS()
	{
		tm.stop();
		double fps = counter / tm.getTimeSec();
		tm.start();
		return static_cast<float>(fps);
	}

	/** Clear queue*/
	void
	clear()
	{
		std::lock_guard<std::mutex> lock(mutex);
		while (!this->empty())
			this->pop();
	}

	unsigned int counter;

private:
	cv::TickMeter tm;
	std::mutex    mutex;
};

} // namespace yolo_opencv

class YoloOpenCVThread : public fawkes::Thread,
                         public fawkes::LoggingAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::ClockAspect
{
public:
	YoloOpenCVThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	fawkes::YoloOpenCVInterface *yolo_opencv_if_;
};

#endif
