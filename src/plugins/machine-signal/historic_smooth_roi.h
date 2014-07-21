/*
 * HistoricSmoothROI.h
 *
 *  Created on: 16.07.2014
 *      Author: ich
 */

#ifndef __FIREVISION_HISTORICSMOOTHROI_H_
#define __FIREVISION_HISTORICSMOOTHROI_H_

#include <boost/circular_buffer.hpp>
#include <fvutils/base/roi.h>

namespace firevision
{

class HistoricSmoothROI :
  public ROI
{
  private:
    HistoricSmoothROI();
    boost::circular_buffer<ROI> history_;
  public:
    HistoricSmoothROI(unsigned int history_length);
    HistoricSmoothROI(ROI const &other, unsigned int history_length = 0);
    HistoricSmoothROI(HistoricSmoothROI const &other);
    HistoricSmoothROI &operator=(HistoricSmoothROI const &other);
    void update(ROI const &next_roi);
};

} /* namespace firevision */

#endif /* __FIREVISION__HISTORICSMOOTHROI_H_ */
