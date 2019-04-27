/*
 * SignalState.h
 *
 *  Created on: 29.03.2014
 *      Author: Victor Mataré
 */

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

#ifndef __PLUGINS_MACHINE_SIGNAL_STATE_H_
#define __PLUGINS_MACHINE_SIGNAL_STATE_H_

#include "custom_rois.h"

#include <fvutils/base/roi.h>
#include <interfaces/RobotinoLightInterface.h>
#include <logging/logger.h>
#include <tf/transformer.h>

#include <boost/circular_buffer.hpp>
#include <set>

#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

class SignalState
{
public:
	typedef struct
	{
		std::shared_ptr<firevision::ROI>                        red_roi;
		std::shared_ptr<firevision::ROI>                        yellow_roi;
		std::shared_ptr<firevision::ROI>                        green_roi;
		std::shared_ptr<fawkes::tf::Stamped<fawkes::tf::Point>> world_pos;
		float                                                   truth;
	} signal_rois_t_;

	typedef struct
	{
		std::shared_ptr<firevision::HistoricSmoothROI>          red_roi;
		std::shared_ptr<firevision::HistoricSmoothROI>          yellow_roi;
		std::shared_ptr<firevision::HistoricSmoothROI>          green_roi;
		std::shared_ptr<fawkes::tf::Stamped<fawkes::tf::Point>> world_pos;
		float                                                   truth;
	} historic_signal_rois_t_;

	SignalState(unsigned int buflen, fawkes::Logger *logger, historic_signal_rois_t_ &signal);

	typedef struct
	{
		bool red;
		bool yellow;
		bool green;
	} frame_state_t_;

	struct compare_rois_by_area
	{
		bool
		operator()(firevision::ROI const &r1, firevision::ROI const &r2)
		{
			unsigned int a1 = r1.width * r1.height;
			unsigned int a2 = r2.width * r2.height;
			return a1 >= a2;
		}
	}; // sort_rois_by_area_;

	struct compare_signal_states_by_visibility
	{
		bool
		operator()(SignalState const &s1, SignalState const &s2)
		{
			return s1.visibility > s2.visibility;
		}
	}; // sort_signal_states_by_visibility_;

	struct compare_signal_states_by_x
	{
		bool
		operator()(SignalState const &s1, SignalState const &s2)
		{
			return s1.pos.x <= s2.pos.x;
		}
	}; // sort_signal_states_by_x_;

	struct compare_signal_states_by_area
	{
		bool
		operator()(SignalState const &s1, SignalState const &s2)
		{
			if ((s1.visibility < 0) == (s2.visibility < 0)) {
				float size_ratio = (float)s1.area / (float)s2.area;
				if (size_ratio < 1.5 && size_ratio > 0.67)
					return s1.pos.x <= s2.pos.x;
				else
					return s1.area > s2.area;
			} else
				return s1.visibility > 0;
		}
	}; // sort_signal_states_by_area_;

private:
	typedef struct
	{
		boost::circular_buffer<bool> frames;
		boost::circular_buffer<bool> state;
	} light_history_t_;

	light_history_t_ history_R_;
	light_history_t_ history_Y_;
	light_history_t_ history_G_;
	unsigned int     buflen_;
	unsigned int     state_buflen_;
	std::string      debug_R_;
	std::string      debug_Y_;
	std::string      debug_G_;
	fawkes::Logger * logger_;

	fawkes::RobotinoLightInterface::LightState eval_history(light_history_t_ &history,
	                                                        std::string &     debug_str);

public:
	fawkes::RobotinoLightInterface::LightState              red;
	fawkes::RobotinoLightInterface::LightState              yellow;
	fawkes::RobotinoLightInterface::LightState              green;
	fawkes::upoint_t                                        pos;
	int                                                     visibility;
	bool                                                    ready;
	int                                                     unseen;
	unsigned int                                            area;
	std::shared_ptr<fawkes::tf::Stamped<fawkes::tf::Point>> world_pos;
	historic_signal_rois_t_                                 signal_rois_history_;

	char const *get_debug_R();
	char const *get_debug_Y();
	char const *get_debug_G();
	void        inc_unseen(std::set<firevision::WorldROI, compare_rois_by_area> &laser_rois);
	void        update_geometry(std::list<signal_rois_t_>::iterator const &rois);
	void        update_state(frame_state_t_ const &s);
	float       distance(std::list<signal_rois_t_>::iterator const &s);
};

#endif /* __PLUGINS_MACHINE_SIGNAL_STATE_H_ */
