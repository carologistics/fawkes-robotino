/***************************************************************************
 *  image_shm_loader.cpp - Loader for image from SHM
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

#include "image_shm_loader.h"
#include <exception>
#include <memory>

#include <fvutils/color/conversions.h>

TF_Plugin_Image_SHM_Loader::TF_Plugin_Image_SHM_Loader(
    std::string name, fawkes::Logger *logger, std::string shm_id,
    firevision::colorspace_t expected_colorspace, unsigned int width,
    unsigned int height, bool normalize, double norm_mean, double norm_std)
    : TF_Plugin_Loader(name, logger), should_colorspace_(expected_colorspace),
      width_(width), height_(height), own_final_buffer_(false),
      normalize_(normalize), normalize_mean_(norm_mean),
      normalize_std_(norm_std) {}

TF_Plugin_Image_SHM_Loader::~TF_Plugin_Image_SHM_Loader() {}

bool TF_Plugin_Image_SHM_Loader::verify() { return true; }

void TF_Plugin_Image_SHM_Loader::resize(
    const unsigned char *in_buffer, unsigned char *out_buffer,
    firevision::colorspace_t colorspace, unsigned int old_width,
    unsigned int old_height, unsigned int new_width, unsigned int new_height) {
  int type = colorspace_to_cv_type(colorspace);                 // wonderful
  cv::Mat src(old_width, old_height, type, (void *)in_buffer);  // use AUTO_STEP
  cv::Mat dst(new_width, new_height, type, (void *)out_buffer); // use AUTO_STEP
  cv::resize(src, dst, dst.size(), /* compute scaling factors by dstsize */ 0,
             0, /*interpolation*/ cv::INTER_LINEAR);
}

void TF_Plugin_Image_SHM_Loader::convert(
    const unsigned char *in_buffer, unsigned char *out_buffer,
    firevision::colorspace_t old_colorspace,
    firevision::colorspace_t new_colorspace, unsigned int width,
    unsigned int height) {
  firevision::convert(old_colorspace, new_colorspace, in_buffer, out_buffer,
                      width, height);
}

int TF_Plugin_Image_SHM_Loader::colorspace_to_cv_type(
    firevision::colorspace_t colorspace) {
  switch (colorspace) {
  case firevision::colorspace_t::CS_UNKNOWN:
  case firevision::colorspace_t::YUV411_PLANAR:
  case firevision::colorspace_t::YUV411_PACKED:
  case firevision::colorspace_t::YUV420_PLANAR:
  case firevision::colorspace_t::YUY2:
  case firevision::colorspace_t::YVY2:
  case firevision::colorspace_t::YUV422_PACKED:
  case firevision::colorspace_t::YUV444_PACKED:
  case firevision::colorspace_t::YVU444_PACKED:
  case firevision::colorspace_t::YUV422_PLANAR:
  case firevision::colorspace_t::YUV422_PLANAR_QUARTER:
  case firevision::colorspace_t::BAYER_MOSAIC_RGGB:
  case firevision::colorspace_t::BAYER_MOSAIC_GBRG:
  case firevision::colorspace_t::BAYER_MOSAIC_GRBG:
  case firevision::colorspace_t::BAYER_MOSAIC_BGGR:
  case firevision::colorspace_t::RAW16:
  case firevision::colorspace_t::RGB_PLANAR:
  case firevision::colorspace_t::CARTESIAN_3D_FLOAT:
  case firevision::colorspace_t::CARTESIAN_3D_DOUBLE:
  case firevision::colorspace_t::CARTESIAN_3D_FLOAT_RGB:
  default:
    logger_->log_error(name_.c_str(),
                       "Should not use %s for anything here right now",
                       colorspace_to_string(colorspace));
    return -1;

  case firevision::colorspace_t::BGR:
  case firevision::colorspace_t::RGB:
    return CV_8UC3;

  case firevision::colorspace_t::MONO8:
  case firevision::colorspace_t::GRAY8:
    return CV_8UC1;

  case firevision::colorspace_t::RGB_WITH_ALPHA:
  case firevision::colorspace_t::BGR_WITH_ALPHA:
    return CV_8UC4;
  }
  return -1;
}

TF_Plugin_Image_SHM_Loader::BASE_TYPE
TF_Plugin_Image_SHM_Loader::colorspace_to_base_type(
    firevision::colorspace_t colorspace) {
  switch (colorspace) {
  case firevision::colorspace_t::CS_UNKNOWN:
  case firevision::colorspace_t::YUV411_PLANAR:
  case firevision::colorspace_t::YUV411_PACKED:
  case firevision::colorspace_t::YUV420_PLANAR:
  case firevision::colorspace_t::YUY2:
  case firevision::colorspace_t::YVY2:
  case firevision::colorspace_t::YUV422_PACKED:
  case firevision::colorspace_t::YUV444_PACKED:
  case firevision::colorspace_t::YVU444_PACKED:
  case firevision::colorspace_t::YUV422_PLANAR:
  case firevision::colorspace_t::YUV422_PLANAR_QUARTER:
  case firevision::colorspace_t::BAYER_MOSAIC_RGGB:
  case firevision::colorspace_t::BAYER_MOSAIC_GBRG:
  case firevision::colorspace_t::BAYER_MOSAIC_GRBG:
  case firevision::colorspace_t::BAYER_MOSAIC_BGGR:
  case firevision::colorspace_t::RAW16:
  case firevision::colorspace_t::RGB_PLANAR:
  case firevision::colorspace_t::CARTESIAN_3D_FLOAT:
  case firevision::colorspace_t::CARTESIAN_3D_DOUBLE:
  case firevision::colorspace_t::CARTESIAN_3D_FLOAT_RGB:
  default:
    logger_->log_error(name_.c_str(),
                       "Should not use %s for anything here right now",
                       colorspace_to_string(colorspace));
    return BASE_TYPE::TYPE_UNSUPPORTED;

  case firevision::colorspace_t::RGB_WITH_ALPHA:
  case firevision::colorspace_t::BGR_WITH_ALPHA:
  case firevision::colorspace_t::MONO8:
  case firevision::colorspace_t::GRAY8:
  case firevision::colorspace_t::BGR:
  case firevision::colorspace_t::RGB:
    return BASE_TYPE::TYPE_UCHAR;
  }
  return BASE_TYPE::TYPE_UNSUPPORTED;
}

template <typename T>
void TF_Plugin_Image_SHM_Loader::normalize(T *buffer, size_t size) {
  size /= sizeof(T);
  T mean = static_cast<T>(normalize_mean_);
  T std = static_cast<T>(normalize_std_);
  for (size_t i = 0; i < size; i++) {
    buffer[i] = ((buffer[i] - mean) / std);
  }
}
