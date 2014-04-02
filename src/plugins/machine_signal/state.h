/*
 * SignalState1.h
 *
 *  Created on: 29.03.2014
 *      Author: ich
 */

#ifndef __PLUGINS_MACHINE_SIGNAL_STATE_H_
#define __PLUGINS_MACHINE_SIGNAL_STATE_H_

#include <boost/circular_buffer.hpp>
#include <interfaces/RobotinoLightInterface.h>
#include <fvutils/base/roi.h>
#include <logging/logger.h>

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

class SignalState {
  public:
    SignalState(unsigned int buflen, fawkes::Logger *logger);

    typedef struct {
        bool red;
        bool yellow;
        bool green;
    } frame_state_t_;

    typedef struct {
        firevision::ROI *red_roi;
        firevision::ROI *yellow_roi;
        firevision::ROI *green_roi;
    } signal_rois_t_;

  private:
    typedef struct {
      boost::circular_buffer<bool> frames;
      boost::circular_buffer<bool> state;
    } light_history_t_;

    light_history_t_ history_R_;
    light_history_t_ history_Y_;
    light_history_t_ history_G_;
    unsigned int buflen_;
    unsigned int state_buflen_;
    std::string debug_R_;
    std::string debug_Y_;
    std::string debug_G_;
    fawkes::Logger *logger_;

    fawkes::RobotinoLightInterface::LightState
    eval_history(light_history_t_ &history, std::string &debug_str);

  public:
    fawkes::RobotinoLightInterface::LightState red;
    fawkes::RobotinoLightInterface::LightState yellow;
    fawkes::RobotinoLightInterface::LightState green;
    fawkes::upoint_t pos;
    int visibility;
    bool ready;
    int unseen;
    unsigned int area;

    char const *get_debug_R();
    char const *get_debug_Y();
    char const *get_debug_G();
    void inc_unseen();
    void update(frame_state_t_ const &s, std::list<signal_rois_t_>::iterator const &rois);
    float distance(std::list<signal_rois_t_>::iterator const &s);
};

#endif /* __PLUGINS_MACHINE_SIGNAL_STATE_H_ */
