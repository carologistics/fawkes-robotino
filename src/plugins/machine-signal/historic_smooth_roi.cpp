/*
 * HistoricSmoothROI.cpp
 *
 *  Created on: 16.07.2014
 *      Author: ich
 */

#include "historic_smooth_roi.h"
#include <climits>

namespace firevision
{

/** @class HistoricSmoothROI
 * A ROI with a history buffer that can be used to smooth out changes in geometry.
 */

HistoricSmoothROI::HistoricSmoothROI(unsigned int history_len)
: ROI(),
  history_(history_len)
{}

HistoricSmoothROI::HistoricSmoothROI(HistoricSmoothROI const &other)
: ROI(other),
  history_(other.history_)
{}

HistoricSmoothROI::HistoricSmoothROI(ROI const &other, unsigned int history_len)
: ROI(other),
  history_(history_len)
{}


void HistoricSmoothROI::update(ROI const &next_roi) {
  history_.push_front(next_roi);
  unsigned long int start_x_sum = start.x;
  unsigned long int start_y_sum = start.y;
  unsigned long int width_sum = width;
  unsigned long int height_sum = height;
  for (ROI const &roi : history_) {
    start_x_sum += roi.start.x;
    start_y_sum += roi.start.y;
    width_sum += roi.width;
    height_sum += roi.height;
  }
  start.x = start_x_sum / (history_.size() + 1);
  start.y = start_y_sum / (history_.size() + 1);
  width = width_sum / (history_.size() + 1);
  height = height_sum / (history_.size() + 1);
}

HistoricSmoothROI &HistoricSmoothROI::operator=(HistoricSmoothROI const &roi)
{
  this->start.x         = roi.start.x;
  this->start.y         = roi.start.y;
  this->width           = roi.width;
  this->height          = roi.height;
  this->image_width     = roi.image_width;
  this->image_height    = roi.image_height;
  this->line_step       = roi.line_step;
  this->pixel_step      = roi.pixel_step;
  this->hint            = roi.hint;
  this->color           = roi.color;
  this->num_hint_points = roi.num_hint_points;

  this->history_ = boost::circular_buffer<ROI>(roi.history_);

  return *this;
}

} /* namespace firevision */
