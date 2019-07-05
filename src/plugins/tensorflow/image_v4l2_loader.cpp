/***************************************************************************
 *  image_v4l2_loader.cpp - Loader for image from V4L2 device
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

#include "image_v4l2_loader.h"
#include <exception>
#include <fvcams/v4l2.h>
#include <memory>

#include <fvutils/color/conversions.h>

TF_Plugin_Image_V4L2_Loader::TF_Plugin_Image_V4L2_Loader(
    std::string name, fawkes::Logger *logger, std::string device_name,
    firevision::colorspace_t expected_colorspace, unsigned int width,
    unsigned int height, bool normalize, double norm_mean, double norm_std)
    : TF_Plugin_Image_Loader(name, logger, expected_colorspace, width, height,
                             normalize, norm_mean, norm_std, false, false) {
  cam_ = new firevision::V4L2Camera(device_name.c_str());
}
