
/**************************************************************************
 *  thresholds.h - This header defines thresholds for color classification
 *
 *  Created: Wed May 11 11:22:00 2005
 *  Copyright  2005  Martin Heracles <Martin.Heracles@rwth-aachen.de>
 *                   Tim Niemueller  [www.niemueller.de]
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_COLORMODEL_RANGE_H_
#define __FIREVISION_COLORMODEL_RANGE_H_

#include <fvmodels/color/colormodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/* The following thresholds define certain color regions in the
   two-dimensional UV-Colorspace (ignoring the Y-component).
   It is assumed that the Y-, U- and V-components range from 0 to 255 each,
   and that (0, 0) is at the lower left corner. */


class ColorModelRange : public ColorModel
{
 public:
  ColorModelRange(unsigned int y_min, unsigned int y_max, unsigned int u_min, unsigned int u_max, unsigned int v_min, unsigned int v_max );

  color_t       determine(unsigned int y,
			  unsigned int u,
			  unsigned int v) const;

  const char *  get_name();
  void          print_thresholds();
 private:
  unsigned int threshold_y_low_;
  unsigned int threshold_u_low_;
  unsigned int threshold_v_low_;
  unsigned int threshold_y_high_;
  unsigned int threshold_u_high_;
  unsigned int threshold_v_high_;


};

} // end namespace firevision

#endif
