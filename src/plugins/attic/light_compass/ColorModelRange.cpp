
/***************************************************************************
 *  thresholds.cpp - Implementation of a thresholds color model
 *
 *  Created: Wed May 18 13:59:18 2005
 *  Copyright  2005  Tim Niemueller  [www.niemueller.de]
 *                   Matrin Heracles <martin.heracles@rwth-aachen.de>
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "ColorModelRange.h"

#include <iostream>

using namespace std;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ColorModelThresholds <fvmodels/color/thresholds.h>
 * Really simple thresholds-based model with some hard-coded thresholds. Was
 * just for initial development of color models.
 */

ColorModelRange::ColorModelRange(unsigned int y_min,
                                 unsigned int y_max,
                                 unsigned int u_min,
                                 unsigned int u_max,
                                 unsigned int v_min,
                                 unsigned int v_max)
{
	threshold_y_low_  = y_min;
	threshold_y_high_ = y_max;
	threshold_u_low_  = u_min;
	threshold_v_low_  = u_max;
	threshold_u_high_ = v_min;
	threshold_v_high_ = v_max;
}

color_t
ColorModelRange::determine(unsigned int y, unsigned int u, unsigned int v) const
{
	if (y >= threshold_y_low_ && y <= threshold_y_high_ && u >= threshold_u_low_
	    && u <= threshold_u_high_ && v >= threshold_v_low_ && v <= threshold_v_high_) {
		return C_WHITE;
	} else {
		return C_OTHER;
	}
}

const char *
ColorModelRange::get_name()
{
	return "ColorModelRange";
}

/** Print the thresholds to stdout.
 */
void
ColorModelRange::print_thresholds()
{
	cout << get_name() << endl
	     << "==========================================================" << endl
	     << "y: " << threshold_y_low_ << " - " << threshold_y_high_ << "u: " << threshold_y_low_
	     << " - " << threshold_y_high_ << "v: " << threshold_y_low_ << " - " << threshold_y_high_
	     << endl;
}

} // end namespace firevision
