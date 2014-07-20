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
#include <tf/transformer.h>
#include "historic_smooth_roi.h"

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

class SignalState {
  public:
    typedef struct {
        std::shared_ptr<firevision::ROI> red_roi;
        std::shared_ptr<firevision::ROI> yellow_roi;
        std::shared_ptr<firevision::ROI> green_roi;
        std::shared_ptr<fawkes::tf::Stamped<fawkes::tf::Point>> world_pos;
    } signal_rois_t_;

    typedef struct {
        std::shared_ptr<firevision::HistoricSmoothROI> red_roi;
        std::shared_ptr<firevision::HistoricSmoothROI> yellow_roi;
        std::shared_ptr<firevision::HistoricSmoothROI> green_roi;
        std::shared_ptr<fawkes::tf::Stamped<fawkes::tf::Point>> world_pos;
    } historic_signal_rois_t_;

    SignalState(unsigned int buflen, fawkes::Logger *logger, historic_signal_rois_t_ &signal);

    typedef struct {
        bool red;
        bool yellow;
        bool green;
    } frame_state_t_;

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
    historic_signal_rois_t_ signal_rois_history_;

  public:
    fawkes::RobotinoLightInterface::LightState red;
    fawkes::RobotinoLightInterface::LightState yellow;
    fawkes::RobotinoLightInterface::LightState green;
    fawkes::upoint_t pos;
    int visibility;
    bool ready;
    int unseen;
    unsigned int area;
    std::shared_ptr<fawkes::tf::Stamped<fawkes::tf::Point>> world_pos;

    char const *get_debug_R();
    char const *get_debug_Y();
    char const *get_debug_G();
    void inc_unseen();
    void update_geometry(std::list<signal_rois_t_>::iterator const &rois);
    void update_state(frame_state_t_ const &s);
    float distance(std::list<signal_rois_t_>::iterator const &s);
};

#endif /* __PLUGINS_MACHINE_SIGNAL_STATE_H_ */
