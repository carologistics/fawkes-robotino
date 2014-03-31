
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

#include "state.h"

SignalState::SignalState(unsigned int buflen, fawkes::Logger *logger)
{
  logger_ = logger;
  buflen_ = buflen;
  state_buflen_ = buflen*2;
  history_R_.frames.set_capacity(buflen);
  history_Y_.frames.set_capacity(buflen);
  history_G_.frames.set_capacity(buflen);
  history_R_.state.set_capacity(state_buflen_);
  history_Y_.state.set_capacity(state_buflen_);
  history_G_.state.set_capacity(state_buflen_);
  area = 0;
  visibility = -1;
  ready = false;
  unseen = 0;
  red = fawkes::RobotinoLightInterface::UNKNOWN;
  yellow = fawkes::RobotinoLightInterface::UNKNOWN;
  green = fawkes::RobotinoLightInterface::UNKNOWN;
}


void SignalState::inc_unseen() {
  if (++unseen > 2) {
    if (visibility >= 0) visibility = -1;
    else visibility--;
    ready = false;
  }
}

char const *SignalState::get_debug_R() {
  return debug_R_.c_str();
}

char const *SignalState::get_debug_Y() {
  return debug_Y_.c_str();
}

char const *SignalState::get_debug_G() {
  return debug_G_.c_str();
}

float SignalState::distance(std::list<signal_rois_t_>::iterator const &s) {
  int dx = s->yellow_roi->start.x + s->yellow_roi->width/2 - pos.x;
  int dy = s->yellow_roi->start.y + s->yellow_roi->height/2 - pos.y;
  return (float)sqrt(dx*dx + dy*dy);
}


fawkes::RobotinoLightInterface::LightState
SignalState::eval_history(light_history_t_ &history, std::string &debug_string)
{
  int count = 0;
  debug_string = "";
  for (boost::circular_buffer<bool>::const_iterator it = history.frames.begin();
      it != history.frames.end(); it++) {
    debug_string += *it ? "#" : ".";
    count += *it ? 1 : -1;
  }

  if (count < 0) {
    history.state.push_front(false);
  }
  else if (count > 0) {
    history.state.push_front(true);
  }
  else {
    history.state.push_front(history.state.front());
  }

  debug_string += " -> ";

  unsigned int num_changes = 0;
  if (!history.state.empty()) {
    boost::circular_buffer<bool>::const_iterator it = history.state.begin();
    bool last_state = *it;
    while (it != history.state.end()) {
      debug_string += *it ? "#" : ".";
      if (*it != last_state) {
        num_changes++;
      }
      last_state = *it;
      ++it;
    }
  }

  if (history.state.size() < state_buflen_) {
    return fawkes::RobotinoLightInterface::UNKNOWN;
  }
  if (num_changes >= 2) return fawkes::RobotinoLightInterface::BLINKING;
  else return history.state.front() ?
      fawkes::RobotinoLightInterface::ON :
      fawkes::RobotinoLightInterface::OFF;
}


void SignalState::update(frame_state_t_ const &s, std::list<signal_rois_t_>::iterator const &rois) {
  area = rois->red_roi->width * rois->red_roi->height
      + rois->yellow_roi->width * rois->yellow_roi->height
      + rois->green_roi->width * rois->green_roi->height;
  history_R_.frames.push_front(s.red);
  history_Y_.frames.push_front(s.yellow);
  history_G_.frames.push_front(s.green);
  pos.x = rois->yellow_roi->start.x + rois->yellow_roi->width/2;
  pos.y = rois->yellow_roi->start.y + rois->yellow_roi->height/2;

  unseen = 0;

  fawkes::RobotinoLightInterface::LightState new_red, new_yellow, new_green;
  new_red = eval_history(history_R_, debug_R_);
  new_yellow = eval_history(history_Y_, debug_Y_);
  new_green = eval_history(history_G_, debug_G_);

  // decrease visibility history if:
  // - All lights are off or
  // - One is unknown or
  // - A light changes from something other than unknown
  if (
    (new_red == fawkes::RobotinoLightInterface::OFF
        && new_yellow == fawkes::RobotinoLightInterface::OFF
        && new_green == fawkes::RobotinoLightInterface::OFF
    )
    || new_red == fawkes::RobotinoLightInterface::UNKNOWN
    || new_yellow == fawkes::RobotinoLightInterface::UNKNOWN
    || new_green == fawkes::RobotinoLightInterface::UNKNOWN
    || (red != fawkes::RobotinoLightInterface::UNKNOWN && new_red != red)
    || (yellow != fawkes::RobotinoLightInterface::UNKNOWN && new_yellow != yellow)
    || (green != fawkes::RobotinoLightInterface::UNKNOWN && new_green != green)
  ) {
    if (visibility >= 0) visibility = -1;
    else visibility--;
  }
  else {
    if (unlikely(visibility < 0)) visibility = 1;
    else visibility++;
  }
  red = new_red;
  yellow = new_yellow;
  green = new_green;

  ready = (visibility >= (long int) state_buflen_);
}

