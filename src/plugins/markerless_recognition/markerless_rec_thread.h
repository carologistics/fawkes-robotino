/***************************************************************************
 *  markerless_rec_thread.h - Thread to print recognized MPS
 *
 *  Created: Thu May 7 10:10:00 2017
 *  Copyright  2017  Sebastian Schönitz, Daniel Habering, Carsten Stoffels
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


#ifndef __PLUGINS_MARKERLESS_MARKERLESS_RECOGNITION_THREAD_H_
#define __PLUGINS_MARKERLESS_MARKERLESS_RECOGNITION_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <string>


namespace fawkes {
  class MPSRecognitionInterface;
}

struct Probability
{
	float p[5];
};

enum MPSType {
	BS,
	CS,
	DS,
	RS,
	SS,
	NoStationDetected
};

class MarkerlessRecognitionThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect
{


public:
  MarkerlessRecognitionThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
  
 private:
  void clear_data();
  Probability recognize_current_pic(const std::string image);
  void recognize_mps();
  void estimate_mps_type(const Probability &prob) const;
  void readImage();	

  fawkes::MPSRecognitionInterface *mps_rec_if_;

  std::string path_prefix_;
  std::vector<std::string> imageSet_;

  float th_first = 0.8;
  float th_sec = 0.5;
};

#endif
