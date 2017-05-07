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

typedef struct
{
	float s[5];
}Score;

namespace fawkes {
  class MPSRecognitionInterface;
}

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
  virtual void readImage();
 private:
  void writeTotalProbability(Score prob);
  void writeWinProbability(float prob);
  void writeState(bool state);
  void estimateMPStype();


  fawkes::MPSRecognitionInterface *mps_rec_if_;
};

#endif
