/***************************************************************************
 *  image_v4l2_loader.h - Loader for image from V4L2 device
 *
 *  Created: Thu May 5 10:23:50 2019
 *  Copyright  2019 Morian Sonnet
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

#ifndef IMAGE_V4L2_LOADER
#define IMAGE_V4L2_LOADER
#include "image_loader.h"
#include <opencv/cv.hpp>

/** Class for loading Image data from V4L2 device
 */
class TF_Plugin_Image_V4L2_Loader : public TF_Plugin_Image_Loader {
public:
  /** Constructor
   * @param name Name of the calling thread, used for logging functionalities
   * @param logger Logger of the calling thread
   * @param device_name The name of the file
   * @param expected_colorspace Colorspace of output image
   * @param width Pixel width of output image
   * @param height Pixel height of output image
   * @param normalize Whether the output image shall be normalized
   * @param norm_mean Mean value for normalization
   * @param norm_std StD value for normalization
   */
  TF_Plugin_Image_V4L2_Loader(std::string name, fawkes::Logger *logger,std::string device_name,
                             firevision::colorspace_t expected_colorspace,
                             unsigned int width, unsigned int height,
                             bool normalize = false, double norm_mean = 0.0,
                             double norm_std = 0.0 );

protected:
private:
};

#endif
