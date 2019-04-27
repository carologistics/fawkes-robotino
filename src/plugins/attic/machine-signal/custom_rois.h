/*
 * HistoricSmoothROI.h
 *
 *  Created on: 16.07.2014
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

#ifndef __FIREVISION_HISTORICSMOOTHROI_H_
#define __FIREVISION_HISTORICSMOOTHROI_H_

#include <fvutils/base/roi.h>
#include <tf/types.h>

#include <boost/circular_buffer.hpp>

namespace firevision {

class HistoricSmoothROI : public ROI
{
private:
	HistoricSmoothROI();
	boost::circular_buffer<ROI> history_;

public:
	HistoricSmoothROI(unsigned int history_length);
	HistoricSmoothROI(ROI const &other, unsigned int history_length = 0);
	HistoricSmoothROI(HistoricSmoothROI const &other);
	HistoricSmoothROI &operator=(HistoricSmoothROI const &other);
	void               update(ROI const &next_roi);
};

class WorldROI : public firevision::ROI
{
public:
	std::shared_ptr<fawkes::tf::Stamped<fawkes::tf::Point>> world_pos;
	WorldROI() : ROI()
	{
	}
};

} /* namespace firevision */

#endif /* __FIREVISION__HISTORICSMOOTHROI_H_ */
